#include <ros/ros.h>

#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "parameters.h"
#include "parameters_ros.h"
#include "common.h"

#include <pelican_catherine/UAVState.h>
#include <autopilot_hinf.h>

#include <dynamic_reconfigure/server.h>
#include <pelican_catherine/controllerDynConfig.h>

autopilot_hinfModelClass gController;

// Definition des variables
bool gPublish;
bool gInit_flag;
bool gLanding_flag;
bool gEmergency_status; // Les flags sont initialises dans le main

int  gTest_mode;

Eigen::VectorXd gY0(4);        // initial position (equilibrium)
Eigen::VectorXd gRef(4);       // references (x, y, z, yaw)
Eigen::VectorXd gGain(20);

double gPsi;                   /*heading (rad) il est defini ici et non dans
le main parce qu'il est utilise dans d'autres fonctions pas comme phi et theta*/

mav_msgs::EigenOdometry gOdometry;

bool gCommand_active;

ros::Time gCommand_time;

// Appel de l'odometrie
void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom) {

  mav_msgs::eigenOdometryFromMsg(*odom, &gOdometry);

  if ((gOdometry.position_W.x() > 2.5)||(gOdometry.position_W.x() < -2.5)||(gOdometry.position_W.y() > 2.5)||(gOdometry.position_W.y() < -2.5)||(gOdometry.position_W.z() > 1.75))
  {
    if (!gEmergency_status){
      gEmergency_status = true;
    }
  }
}

void controller_dyn_callback(pelican_catherine::controllerDynConfig &config, uint32_t level) {
  if (config.RESET) {
    // TO DO : reset parameters, gains
    config.RESET = false;
  }
  else if (level & pelican_catherine::controllerDyn_ENABLE_CTRL){
      if (config.new_controller_gains){
        // Initialisation des gains à partir du fichier config
        gGain[ 0] = config.kx;
        gGain[ 1] = config.kvx;
        gGain[ 2] = config.kix;
        gGain[ 3] = config.kpx;

        gGain[ 4] = config.ky;
        gGain[ 5] = config.kvy;
        gGain[ 6] = config.kiy;
        gGain[ 7] = config.kpy;

        gGain[ 8] = config.kz;
        gGain[ 9] = config.kvz;
        gGain[10] = config.kiz;
        gGain[11] = config.kpz;

        gGain[12] = config.kphi;
        gGain[13] = config.kp;

        gGain[14] = config.ktheta;
        gGain[15] = config.kq;

        gGain[16] = config.kpsi;
        gGain[17] = config.kr;
        gGain[18] = config.kipsi;
        gGain[19] = config.kpsi;

        ROS_INFO("New controller gains");      // Affiche dans le terminal
        config.new_controller_gains   = false; // Remet la valeur de l'entree à 0, pas de hold
      }

      // Voir le fichier de configuration pour les differents modes de test
      gTest_mode = config.test_mode;

      if (config.enable_take_off && !gInit_flag){     // only once
        gY0[0]    = gOdometry.position_W.x();
        gY0[1]    = gOdometry.position_W.y();
        gY0[2]    = gOdometry.position_W.z();
        gY0[3]    = gPsi;
      //  if (config.test_mode == pelican_catherine::controllerDyn_TEST_MANUAL){
        gRef[0]   = gY0[0];
        gRef[1]   = gY0[1];
        gRef[2]   = config.ref_z;
        gRef[3]   = gY0[3];
      //  }

        gGain[ 0] = config.kx;
        gGain[ 1] = config.kvx;
        gGain[ 2] = config.kix;
        gGain[ 3] = config.kpx;

        gGain[ 4] = config.ky;
        gGain[ 5] = config.kvy;
        gGain[ 6] = config.kiy;
        gGain[ 7] = config.kpy;

        gGain[ 8] = config.kz;
        gGain[ 9] = config.kvz;
        gGain[10] = config.kiz;
        gGain[11] = config.kpz;

        gGain[12] = config.kphi;
        gGain[13] = config.kp;

        gGain[14] = config.ktheta;
        gGain[15] = config.kq;

        gGain[16] = config.kpsi;
        gGain[17] = config.kr;
        gGain[18] = config.kipsi;
        gGain[19] = config.kppsi;

        // Une fois que le truc est lance il ne passera plus ici meme si
        // enable_take_off est a nouveau active
        gInit_flag = true;
        ROS_INFO("Take-off Request: %s with Test_mode = %d",config.enable_take_off?"True":"False",gTest_mode);

        config.enable_take_off = false;

      }
      else if (config.enable_landing && !gLanding_flag){   // only once
        // Sera active seulement une fois apres que enable_landing ait ete
        // active pour la premiere fois dans quel cas on garde la meme position
        // et orientation mais z est renvoye a 0
        gRef[0]  = gOdometry.position_W.x();
        gRef[1]  = gOdometry.position_W.y();
        gRef[2]  = 0.0;
        gRef[3]  = gPsi;

        gTest_mode = pelican_catherine::controllerDyn_TEST_MANUAL; // Pas trop sure ???

        gLanding_flag = true;
        ROS_INFO("Landing Request: %s at x_ref = %f, y_ref = %f,psi_ref = %f(deg)",config.enable_landing?"True":"False",gRef[0],gRef[1],gRef[3]*180.0/3.14159);
        config.enable_landing  = false;
      }
      else if (gTest_mode == pelican_catherine::controllerDyn_TEST_MANUAL){
        // gTest_mode = config.test_mode Pourquoi pelican_catherine::controllerDyn_TEST_MANUAL
        if (config.send_waypoint){
          gRef[0]   = config.ref_x;
          gRef[1]   = config.ref_y;
          gRef[2]   = config.ref_z;
          gRef[3]   = config.ref_yaw_deg*3.14159/180.0;
          ROS_INFO("Waypoint Request: x_ref = %f, y_ref = %f, z_ref = %f, psi_ref = %f(deg)",gRef[0],gRef[1],gRef[2],gRef[3]);
          config.send_waypoint   = false;
        }
      }
  }
}

void timmerCallback(const ros::TimerEvent&)
{
  if (!gPublish){
    gPublish = true;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "matlab_autopilot_hinf_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("matlab_autopilot_hinf_node main started");

  ros::Subscriber odometry_sub_;
  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, OdometryCallback);


  ros::Publisher motor_RPM_reference_pub_;          // motor speed RPM   => Asctec pelican test
  motor_RPM_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
        pelican_catherine::default_topics::COMMAND_RPM, 1);

  ros::Publisher motor_velocity_reference_pub_;     // motor speed rad/s => Gazebo test
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
        mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  ros::Publisher uav_state_pub_;
  uav_state_pub_ = nh.advertise<pelican_catherine::UAVState>(pelican_catherine::default_topics::UAV_STATE, 1);

  ros::Timer timer = nh.createTimer(ros::Duration(0.01),timmerCallback);

  ros::Rate r(1000);

  // Initialisation des parametres
  gInit_flag        = false;
  gLanding_flag     = false;
  gEmergency_status = false;
  gPublish          = false;

  // Tant que la commande de decollage n'a pas ete envoyee, les entrees sont a 0
  gRef  << 0.0, 0.0, 0.0, 0.0;
  gGain = Eigen::VectorXd::Zero(20);
  gY0   << 0.0, 0.0, 0.0, 0.0;

  // pas familiere avec cette forme
  dynamic_reconfigure::Server<pelican_catherine::controllerDynConfig> server;
  dynamic_reconfigure::Server<pelican_catherine::controllerDynConfig>::CallbackType f;
  f = boost::bind(&controller_dyn_callback, _1, _2);
  server.setCallback(f);

  // Defintiion des variables internes
  double phi, theta;
  Eigen::Matrix3d R_W_B;
  Eigen::Vector3d velocity_W ;

  Eigen::VectorXd motor_RPM(4);           // range 0 .. 10000 RPM
  Eigen::VectorXd motor_command(4);       // range 0 .. 200
  Eigen::VectorXd motor_speed(4);         // range 0 .. 1047 rad/s

  bool control_actived = false;
  bool end_mission  = false;

  while(ros::ok()) {
    // Transpose u,v,w to x_dot,y_dot,z_dot using Euler matrix
    R_W_B = gOdometry.orientation_W_B.toRotationMatrix();
    velocity_W =  R_W_B * gOdometry.velocity_B;

    // Get phi,theta,psi using Euler matrix careful to the axis change ENU/NED
    double phi, theta;
    gPsi = atan2(R_W_B(1,0),R_W_B(0,0));
    phi = atan2(R_W_B(2,1),R_W_B(2,2));
    theta = asin(-R_W_B(2,0));

    if (gInit_flag && !control_actived) {                     // only once when controller is not actived
        gController.initialize();
        control_actived = true;

        for (unsigned int i=0; i< 20; i++) {
          ROS_INFO("Controller gain k[%d] = %f",i,gGain[i]);
        }
    }

    if (control_actived) {                                    // controller active after take-off request
        /*// Initialization before Step
        gController.autopilot_hinf_U.mode = gTest_mode;*/ // Je n'en ai pas

        // Assignation des entree a partir des valeurs initiales ou de dynamic
        // reconfigure
        for (unsigned int i=0; i< 4; i++) {
          gController.autopilot_hinf_U.ref[i]  = gRef[i];
        }
        /*for (unsigned int i=0; i< 6; i++) {
          gController.autopilot_hinf_U.Y0[i]   = gY0[i];
        }*/

        /*gController.autopilot_hinf_U.Kfz[0] = gGain[9];  // Kvz
        gController.autopilot_hinf_U.Kfz[1] = gGain[8];  // Kz

        gController.autopilot_hinf_U.Kmxi[0] = gGain[5];  //Kvy
        gController.autopilot_hinf_U.Kmxb[0] = gGain[13]; //Kp
        gController.autopilot_hinf_U.Kmxi[1] = gGain[4];  //Ky
        gController.autopilot_hinf_U.Kmxb[1] = gGain[12]; //Kphi

        gController.autopilot_hinf_U.Kmyi[0] = gGain[1];  //Kvx
        gController.autopilot_hinf_U.Kmyb[0] = gGain[15]; //Kq
        gController.autopilot_hinf_U.Kmyi[1] = gGain[0];  //Kx
        gController.autopilot_hinf_U.Kmyb[1] = gGain[14]; //Ktheta

        gController.autopilot_hinf_U.Kmz[0] = gGain[17]; //Kr
        gController.autopilot_hinf_U.Kmz[1] = gGain[16]; //Kpsi

        gController.autopilot_hinf_U.Ki[0] = gGain[10];  //Kiz
        gController.autopilot_hinf_U.Ki[1] = gGain[6];   //Kiy
        gController.autopilot_hinf_U.Ki[2] = gGain[2];   //Kix
        gController.autopilot_hinf_U.Ki[3] = gGain[18];  //Kipsi

        gController.autopilot_hinf_U.Kp[0] = gGain[11];  //Kpz
        gController.autopilot_hinf_U.Kp[1] = gGain[7];   //Kpy
        gController.autopilot_hinf_U.Kp[2] = gGain[3];   //Kpx
        gController.autopilot_hinf_U.Kp[3] = gGain[19];  //Kppsi*/

        // Get x,y,z
        gController.autopilot_hinf_U.X[6 ]  = gOdometry.position_W.x();
        gController.autopilot_hinf_U.X[7 ]  = gOdometry.position_W.y();
        gController.autopilot_hinf_U.X[8 ]  = gOdometry.position_W.z();
        // Get u,v,w
        gController.autopilot_hinf_U.X[0 ]  = velocity_W.x();
        gController.autopilot_hinf_U.X[1 ]  = velocity_W.y();
        gController.autopilot_hinf_U.X[2 ]  = velocity_W.z();

        /*ROS_INFO("z position = %f",gOdometry.position_W.z());
        ROS_INFO("z velocity = %f",gOdometry.velocity_B.z());*/

        // Assign phi,theta,psi
        gController.autopilot_hinf_U.X[9 ]  = phi;
        gController.autopilot_hinf_U.X[10]  = theta;
        gController.autopilot_hinf_U.X[11]  = gPsi;
        // Get p,q,r
        gController.autopilot_hinf_U.X[3 ]  = gOdometry.angular_velocity_B.x();
        gController.autopilot_hinf_U.X[4 ]  = gOdometry.angular_velocity_B.y();
        gController.autopilot_hinf_U.X[5 ]  = gOdometry.angular_velocity_B.z();

        // Run Matlab controller
        gController.step();

        // Received data from Matlab
        for(unsigned int i=0; i< 4; i++) {
            if (gEmergency_status || (gLanding_flag && (gOdometry.position_W.z() <= 0.175)))
            {
              motor_command[i] = 1.0;
              motor_RPM[i]     = 1075.0;
              motor_speed[i]   = 130.0;
            }
            else
            {
              /*motor_command[i] = gController.tunning_pelican_Y.motor_command[i];        // normalized [1 .. 200] => Asctec pelican
              motor_RPM[i]     = 1250.0 + motor_command[i]*43.75;                       // real RPM
              motor_speed[i]   = motor_RPM[i]/9.5493;    */
              //motor_Va[i] = gController.autopilot_hinf_Y.Va[i]; // ??? => Gazebo
              motor_speed[i]   = gController.autopilot_hinf_Y.omega_d[i]; // rad/s => Gazebo
              motor_RPM[i]     = motor_speed[i]*9.549296596425384;        // Real RPM
              motor_command[i] = (motor_RPM[i]-1075.0)/37.625;             // normalized [1 .. 200] => Asctec pelican
            }
        }
        /*ROS_INFO("omega_t = %f",gController.autopilot_hinf_Y.omega_t[1]);
        ROS_INFO("omega = %f",gController.autopilot_hinf_Y.omega[1]);
        ROS_INFO("Va = %f",gController.autopilot_hinf_Y.Va[1]);*/
        // Send command: RPM and normalized command in 0 .. 200
        mav_msgs::ActuatorsPtr motorRPM_msg(new mav_msgs::Actuators);
        motorRPM_msg->angular_velocities.clear();
        motorRPM_msg->normalized.clear();
        for (int i = 0; i < 4; i++) {
          motorRPM_msg->angular_velocities.push_back(motor_RPM[i]);
          motorRPM_msg->normalized.push_back(motor_command[i]);
        }
        motorRPM_msg->header.stamp =  ros::Time::now();
        motor_RPM_reference_pub_.publish(motorRPM_msg);

        // Send command: Rotor speed (rad/s)
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        actuator_msg->angular_velocities.clear();
        //actuator_msg->normalized.clear();
        for (int i = 0; i < 4; i++) {
          actuator_msg->angular_velocities.push_back(motor_speed[i]);
        }
        actuator_msg->header.stamp =  ros::Time::now();
        motor_velocity_reference_pub_.publish(actuator_msg);
        /*ROS_INFO("motor_speed [%d] = %f",1,motor_speed[1]);
        ROS_INFO("motor_Va [%d] = %f",1,motor_Va[1]);
        ROS_INFO("kfz = %f",kfz_out);
        ROS_INFO("kmx = %f",kmx_out);
        ROS_INFO("kmy = %f",kmy_out);
        ROS_INFO("kmz = %f",kmz_out);
        for (int i = 0; i < 4; i++) {
          ROS_INFO("ki [%d] = %f",i,ki_out[i]);
        }*/

    } else { // control_actived = false ( before take-off)
      mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
      actuator_msg->angular_velocities.clear();

      for (int i = 0; i < 4; i++) {
        actuator_msg->angular_velocities.push_back(0.0);
        actuator_msg->normalized.push_back(0.0);
      }
      actuator_msg->header.stamp =  ros::Time::now();
      motor_RPM_reference_pub_.publish(actuator_msg);
      motor_velocity_reference_pub_.publish(actuator_msg);
    }

    if (gEmergency_status)
    {
      ROS_ERROR("matlab_autopilot_hinf_node emergency status");
      ROS_INFO("x = %f, y = %f, z = %f",gOdometry.position_W.x(),gOdometry.position_W.y(),gOdometry.position_W.z());
      ros::Duration(0.5).sleep();
      gController.terminate();
      break;
    }

    if (gLanding_flag && (gOdometry.position_W.z() <= 0.175))
    {
      ROS_INFO("Controller desactivated");
      ROS_INFO("x = %f, y = %f, z = %f",gOdometry.position_W.x(),gOdometry.position_W.y(),gOdometry.position_W.z());
      ros::Duration(0.5).sleep();
      gController.terminate();
      break;
    }

    // Publish data: UAV state in World frame
    if (gPublish){
      pelican_catherine::UAVStatePtr uav_state_msg(new pelican_catherine::UAVState);

      uav_state_msg->position_W.x  = gOdometry.position_W.x();
      uav_state_msg->position_W.y  = gOdometry.position_W.y();
      uav_state_msg->position_W.z  = gOdometry.position_W.z();
      uav_state_msg->velocity_B.x  = velocity_W.x();
      uav_state_msg->velocity_B.y  = velocity_W.y();
      uav_state_msg->velocity_B.z  = velocity_W.z();
      uav_state_msg->euler_angle.x = phi;
      uav_state_msg->euler_angle.y = theta;
      uav_state_msg->euler_angle.z = gPsi;
      uav_state_msg->rotation_speed_B.x  = gOdometry.angular_velocity_B.x();
      uav_state_msg->rotation_speed_B.y  = gOdometry.angular_velocity_B.y();
      uav_state_msg->rotation_speed_B.z  = gOdometry.angular_velocity_B.z();

      uav_state_msg->header.stamp  =  ros::Time::now();
      uav_state_pub_.publish(uav_state_msg);
      gPublish = false;
    }

    ros::spinOnce();
    r.sleep();
  } // end while

  return 0;
}
