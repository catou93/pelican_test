/*
 * autopilot_hinf.cpp
 *
 * Code generation for model "autopilot_hinf".
 *
 * Model version              : 1.494
 * Simulink Coder version : 8.11 (R2016b) 25-Aug-2016
 * C++ source code generated on : Fri Oct 19 14:10:38 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Execution efficiency
 * Validation result: Passed (9), Warnings (3), Error (0)
 */

#include "autopilot_hinf.h"
#include "autopilot_hinf_private.h"

/*
 * This function updates continuous states using the ODE5 fixed-step
 * solver algorithm
 */
void autopilot_hinfModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si
  )
{
  /* Solver Matrices */
  static const real_T rt_ODE5_A[6] = {
    1.0/5.0, 3.0/10.0, 4.0/5.0, 8.0/9.0, 1.0, 1.0
  };

  static const real_T rt_ODE5_B[6][6] = {
    { 1.0/5.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    { 3.0/40.0, 9.0/40.0, 0.0, 0.0, 0.0, 0.0 },

    { 44.0/45.0, -56.0/15.0, 32.0/9.0, 0.0, 0.0, 0.0 },

    { 19372.0/6561.0, -25360.0/2187.0, 64448.0/6561.0, -212.0/729.0, 0.0, 0.0 },

    { 9017.0/3168.0, -355.0/33.0, 46732.0/5247.0, 49.0/176.0, -5103.0/18656.0,
      0.0 },

    { 35.0/384.0, 0.0, 500.0/1113.0, 125.0/192.0, -2187.0/6784.0, 11.0/84.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE5_IntgData *id = (ODE5_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T *f4 = id->f[4];
  real_T *f5 = id->f[5];
  real_T hB[6];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  autopilot_hinf_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE5_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  autopilot_hinf_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE5_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  autopilot_hinf_derivatives();

  /* f(:,4) = feval(odefile, t + hA(3), y + f*hB(:,3), args(:)(*)); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE5_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[2]);
  rtsiSetdX(si, f3);
  this->step();
  autopilot_hinf_derivatives();

  /* f(:,5) = feval(odefile, t + hA(4), y + f*hB(:,4), args(:)(*)); */
  for (i = 0; i <= 3; i++) {
    hB[i] = h * rt_ODE5_B[3][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[3]);
  rtsiSetdX(si, f4);
  this->step();
  autopilot_hinf_derivatives();

  /* f(:,6) = feval(odefile, t + hA(5), y + f*hB(:,5), args(:)(*)); */
  for (i = 0; i <= 4; i++) {
    hB[i] = h * rt_ODE5_B[4][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f5);
  this->step();
  autopilot_hinf_derivatives();

  /* tnew = t + hA(6);
     ynew = y + f*hB(:,6); */
  for (i = 0; i <= 5; i++) {
    hB[i] = h * rt_ODE5_B[5][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4] + f5[i]*hB[5]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void autopilot_hinfModelClass::step()
{
  real_T rtb_VectorConcatenate_k[16];
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_TrigonometricFunction_o1;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4[96];
  real_T tmp_5[16];
  real_T tmp_6[96];
  real_T tmp_7[16];
  real_T tmp_8[4];
  real_T tmp_9[4];
  real_T tmp_a[384];
  real_T tmp_b[32];
  real_T tmp_c[8];
  int32_T i;
  int32_T i_0;
  real_T rtb_VectorConcatenate_l[4];
  real_T rtb_VectorConcatenate_l_0[4];
  int32_T i_1;
  if (rtmIsMajorTimeStep((&autopilot_hinf_M))) {
    /* set solver stop time */
    if (!((&autopilot_hinf_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&autopilot_hinf_M)->solverInfo,
                            (((&autopilot_hinf_M)->Timing.clockTickH0 + 1) *
        (&autopilot_hinf_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&autopilot_hinf_M)->solverInfo,
                            (((&autopilot_hinf_M)->Timing.clockTick0 + 1) *
        (&autopilot_hinf_M)->Timing.stepSize0 + (&autopilot_hinf_M)
        ->Timing.clockTickH0 * (&autopilot_hinf_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&autopilot_hinf_M))) {
    (&autopilot_hinf_M)->Timing.t[0] = rtsiGetT(&(&autopilot_hinf_M)->solverInfo);
  }

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn1' incorporates:
   *  Constant: '<S7>/Constant'
   */
  rtb_VectorConcatenate_k[0] = 1.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn2' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[1] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn3' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[2] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn4' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[3] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn5' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[4] = 0.0;

  /* Trigonometry: '<S7>/Trigonometric Function' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o1 = std::sin(autopilot_hinf_U.X[11]);
  rtb_TrigonometricFunction_o2 = std::cos(autopilot_hinf_U.X[11]);

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn6' */
  rtb_VectorConcatenate_k[5] = rtb_TrigonometricFunction_o2;

  /* Gain: '<S7>/Gain' */
  rtb_VectorConcatenate_k[6] = -rtb_TrigonometricFunction_o1;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn8' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[7] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn9' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[8] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn10' */
  rtb_VectorConcatenate_k[9] = rtb_TrigonometricFunction_o1;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn11' */
  rtb_VectorConcatenate_k[10] = rtb_TrigonometricFunction_o2;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn12' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[11] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn13' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[12] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn14' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[13] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn15' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  rtb_VectorConcatenate_k[14] = 0.0;

  /* SignalConversion: '<S7>/ConcatBufferAtVector ConcatenateIn16' incorporates:
   *  Constant: '<S7>/Constant'
   */
  rtb_VectorConcatenate_k[15] = 1.0;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/X'
   *  Inport: '<Root>/ref'
   */
  autopilot_hinf_B.e[0] = autopilot_hinf_U.ref[0] - autopilot_hinf_U.X[6];
  autopilot_hinf_B.e[1] = autopilot_hinf_U.ref[1] - autopilot_hinf_U.X[7];
  autopilot_hinf_B.e[2] = autopilot_hinf_U.ref[2] - autopilot_hinf_U.X[8];
  autopilot_hinf_B.e[3] = autopilot_hinf_U.ref[3] - autopilot_hinf_U.X[11];

  /* Product: '<S4>/Product5' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o1 = autopilot_hinf_U.X[9];

  /* Product: '<S4>/Product4' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o2 = autopilot_hinf_U.X[10];

  /* Product: '<S4>/Product1' incorporates:
   *  Inport: '<Root>/X'
   */
  tmp = autopilot_hinf_U.X[9];
  tmp_0 = autopilot_hinf_U.X[10];

  /* Product: '<S4>/Product3' incorporates:
   *  Inport: '<Root>/X'
   */
  tmp_1 = autopilot_hinf_U.X[9];
  tmp_2 = autopilot_hinf_U.X[9];

  /* Product: '<S4>/Product2' incorporates:
   *  Inport: '<Root>/X'
   */
  tmp_3 = autopilot_hinf_U.X[10] * autopilot_hinf_U.X[10];

  /* Concatenate: '<S4>/Matrix Concatenate' incorporates:
   *  Constant: '<S4>/Constant1'
   *  Constant: '<S4>/Constant2'
   *  Constant: '<S4>/Constant3'
   *  Constant: '<S4>/Constant4'
   *  Product: '<S4>/Product1'
   *  Product: '<S4>/Product2'
   *  Product: '<S4>/Product3'
   *  Product: '<S4>/Product4'
   *  Product: '<S4>/Product5'
   *  Product: '<S4>/Product6'
   */
  for (i_1 = 0; i_1 < 4; i_1++) {
    tmp_4[24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2];
    tmp_4[4 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2] *
      rtb_TrigonometricFunction_o1;
    tmp_4[8 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2] *
      rtb_TrigonometricFunction_o2;
    tmp_4[12 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2] * tmp * tmp_0;
    tmp_4[16 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2] * tmp_1 *
      tmp_2;
    tmp_4[20 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[i_1 << 2] * tmp_3;
    tmp_4[1 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1];
    tmp_4[5 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1] *
      rtb_TrigonometricFunction_o1;
    tmp_4[9 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1] *
      rtb_TrigonometricFunction_o2;
    tmp_4[13 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1] * tmp *
      tmp_0;
    tmp_4[17 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1] * tmp_1
      * tmp_2;
    tmp_4[21 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 1] * tmp_3;
    tmp_4[2 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2];
    tmp_4[6 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2] *
      rtb_TrigonometricFunction_o1;
    tmp_4[10 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2] *
      rtb_TrigonometricFunction_o2;
    tmp_4[14 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2] * tmp *
      tmp_0;
    tmp_4[18 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2] * tmp_1
      * tmp_2;
    tmp_4[22 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 2] * tmp_3;
    tmp_4[3 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3];
    tmp_4[7 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3] *
      rtb_TrigonometricFunction_o1;
    tmp_4[11 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3] *
      rtb_TrigonometricFunction_o2;
    tmp_4[15 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3] * tmp *
      tmp_0;
    tmp_4[19 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3] * tmp_1
      * tmp_2;
    tmp_4[23 + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + 3] * tmp_3;
  }

  /* End of Concatenate: '<S4>/Matrix Concatenate' */

  /* Product: '<S1>/Product2' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o1 = autopilot_hinf_U.X[10] * autopilot_hinf_U.X[10];
  for (i_1 = 0; i_1 < 4; i_1++) {
    for (i = 0; i < 4; i++) {
      /* Product: '<S4>/Product6' incorporates:
       *  Constant: '<Root>/Constant7'
       *  Product: '<Root>/Product1'
       */
      tmp_5[i_1 + (i << 2)] = 0.0;
      for (i_0 = 0; i_0 < 24; i_0++) {
        tmp_5[i_1 + (i << 2)] += autopilot_hinf_ConstP.Constant7_Value[(i_0 << 2)
          + i_1] * tmp_4[24 * i + i_0];
      }

      /* Concatenate: '<S1>/Matrix Concatenate' incorporates:
       *  Constant: '<S1>/Constant1'
       *  Constant: '<S1>/Constant2'
       *  Constant: '<S1>/Constant3'
       *  Constant: '<S1>/Constant4'
       *  Inport: '<Root>/X'
       *  Product: '<S1>/Product1'
       *  Product: '<S1>/Product2'
       *  Product: '<S1>/Product3'
       *  Product: '<S1>/Product4'
       *  Product: '<S1>/Product5'
       *  Product: '<S1>/Product6'
       */
      tmp_6[i + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i];
      tmp_6[(i + 24 * i_1) + 4] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i] *
        autopilot_hinf_U.X[9];
      tmp_6[(i + 24 * i_1) + 8] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i] *
        autopilot_hinf_U.X[10];
      tmp_6[(i + 24 * i_1) + 12] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[10];
      tmp_6[(i + 24 * i_1) + 16] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[9];
      tmp_6[(i + 24 * i_1) + 20] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * rtb_TrigonometricFunction_o1;
    }
  }

  /* SignalConversion: '<Root>/TmpSignal ConversionAtProduct3Inport2' incorporates:
   *  Inport: '<Root>/X'
   */
  tmp_8[0] = autopilot_hinf_U.X[1];
  tmp_8[1] = autopilot_hinf_U.X[7];
  tmp_8[2] = autopilot_hinf_U.X[0];
  tmp_8[3] = autopilot_hinf_U.X[6];

  /* Product: '<S3>/Product2' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o1 = autopilot_hinf_U.X[10] * autopilot_hinf_U.X[10];
  for (i_1 = 0; i_1 < 4; i_1++) {
    /* Product: '<Root>/Product1' incorporates:
     *  Sum: '<Root>/Sum2'
     */
    tmp_9[i_1] = 0.0;

    /* Product: '<Root>/Product3' incorporates:
     *  Sum: '<Root>/Sum2'
     */
    rtb_VectorConcatenate_l[i_1] = 0.0;
    for (i = 0; i < 4; i++) {
      /* Product: '<S1>/Product6' incorporates:
       *  Constant: '<Root>/Constant5'
       *  Product: '<Root>/Product3'
       */
      tmp_7[i_1 + (i << 2)] = 0.0;
      for (i_0 = 0; i_0 < 24; i_0++) {
        tmp_7[i_1 + (i << 2)] += autopilot_hinf_ConstP.Constant5_Value[(i_0 << 2)
          + i_1] * tmp_6[24 * i + i_0];
      }

      /* Concatenate: '<S3>/Matrix Concatenate' incorporates:
       *  Constant: '<S3>/Constant1'
       *  Constant: '<S3>/Constant2'
       *  Constant: '<S3>/Constant3'
       *  Constant: '<S3>/Constant4'
       *  Inport: '<Root>/X'
       *  Product: '<S3>/Product1'
       *  Product: '<S3>/Product2'
       *  Product: '<S3>/Product3'
       *  Product: '<S3>/Product4'
       *  Product: '<S3>/Product5'
       *  Product: '<S3>/Product6'
       */
      tmp_4[i + 24 * i_1] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i];
      tmp_4[(i + 24 * i_1) + 4] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i] *
        autopilot_hinf_U.X[9];
      tmp_4[(i + 24 * i_1) + 8] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i] *
        autopilot_hinf_U.X[10];
      tmp_4[(i + 24 * i_1) + 12] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[10];
      tmp_4[(i + 24 * i_1) + 16] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[9];
      tmp_4[(i + 24 * i_1) + 20] = autopilot_hinf_ConstP.pooled4[(i_1 << 2) + i]
        * rtb_TrigonometricFunction_o1;

      /* Product: '<Root>/Product1' incorporates:
       *  Sum: '<Root>/Sum2'
       */
      tmp_9[i_1] += tmp_5[(i << 2) + i_1] * autopilot_hinf_B.e[i];

      /* Product: '<Root>/Product3' incorporates:
       *  Sum: '<Root>/Sum2'
       */
      rtb_VectorConcatenate_l[i_1] += tmp_7[(i << 2) + i_1] * tmp_8[i];
    }
  }

  for (i_1 = 0; i_1 < 4; i_1++) {
    /* Sum: '<Root>/Sum2' */
    rtb_TrigonometricFunction_o1 = 0.0;
    for (i = 0; i < 4; i++) {
      /* Product: '<S3>/Product6' incorporates:
       *  Constant: '<Root>/Constant4'
       *  Product: '<Root>/Product2'
       */
      tmp_5[i_1 + (i << 2)] = 0.0;
      for (i_0 = 0; i_0 < 24; i_0++) {
        tmp_5[i_1 + (i << 2)] += autopilot_hinf_ConstP.Constant4_Value[(i_0 << 2)
          + i_1] * tmp_4[24 * i + i_0];
      }

      /* Sum: '<Root>/Sum2' incorporates:
       *  Integrator: '<Root>/Integrator'
       *  Product: '<Root>/Product2'
       *  Product: '<Root>/Product9'
       */
      rtb_TrigonometricFunction_o1 += tmp_5[(i << 2) + i_1] *
        autopilot_hinf_X.Integrator_CSTATE[i];
    }

    /* Sum: '<Root>/Sum2' incorporates:
     *  Product: '<Root>/Product2'
     *  Product: '<Root>/Product9'
     */
    tmp_8[i_1] = (tmp_9[i_1] - rtb_VectorConcatenate_l[i_1]) +
      rtb_TrigonometricFunction_o1;
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Inport: '<Root>/X'
   */
  rtb_TrigonometricFunction_o1 = autopilot_hinf_U.X[10] * autopilot_hinf_U.X[10];

  /* Concatenate: '<S2>/Matrix Concatenate' incorporates:
   *  Constant: '<S2>/Constant1'
   *  Constant: '<S2>/Constant2'
   *  Constant: '<S2>/Constant3'
   *  Constant: '<S2>/Constant4'
   *  Inport: '<Root>/X'
   *  Product: '<S2>/Product1'
   *  Product: '<S2>/Product2'
   *  Product: '<S2>/Product3'
   *  Product: '<S2>/Product4'
   *  Product: '<S2>/Product5'
   *  Product: '<S2>/Product6'
   */
  for (i_1 = 0; i_1 < 8; i_1++) {
    for (i = 0; i < 8; i++) {
      tmp_a[i + 48 * i_1] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i];
      tmp_a[(i + 48 * i_1) + 8] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i] *
        autopilot_hinf_U.X[9];
      tmp_a[(i + 48 * i_1) + 16] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i]
        * autopilot_hinf_U.X[10];
      tmp_a[(i + 48 * i_1) + 24] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[10];
      tmp_a[(i + 48 * i_1) + 32] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i]
        * autopilot_hinf_U.X[9] * autopilot_hinf_U.X[9];
      tmp_a[(i + 48 * i_1) + 40] = autopilot_hinf_ConstP.pooled3[(i_1 << 3) + i]
        * rtb_TrigonometricFunction_o1;
    }
  }

  /* End of Concatenate: '<S2>/Matrix Concatenate' */

  /* SignalConversion: '<Root>/TmpSignal ConversionAtProduct4Inport2' incorporates:
   *  Inport: '<Root>/X'
   */
  tmp_c[0] = autopilot_hinf_U.X[2];
  tmp_c[1] = autopilot_hinf_U.X[8];
  tmp_c[2] = autopilot_hinf_U.X[3];
  tmp_c[3] = autopilot_hinf_U.X[9];
  tmp_c[4] = autopilot_hinf_U.X[4];
  tmp_c[5] = autopilot_hinf_U.X[10];
  tmp_c[6] = autopilot_hinf_U.X[5];
  tmp_c[7] = autopilot_hinf_U.X[11];
  for (i_1 = 0; i_1 < 4; i_1++) {
    /* Product: '<S2>/Product6' incorporates:
     *  Constant: '<Root>/Constant6'
     *  Product: '<Root>/Product4'
     */
    for (i = 0; i < 8; i++) {
      tmp_b[i_1 + (i << 2)] = 0.0;
      for (i_0 = 0; i_0 < 48; i_0++) {
        tmp_b[i_1 + (i << 2)] += autopilot_hinf_ConstP.Constant6_Value[(i_0 << 2)
          + i_1] * tmp_a[48 * i + i_0];
      }
    }

    /* Product: '<Root>/Product9' incorporates:
     *  Sum: '<Root>/Sum3'
     */
    rtb_VectorConcatenate_l[i_1] = 0.0;
    rtb_VectorConcatenate_l[i_1] += rtb_VectorConcatenate_k[i_1] * tmp_8[0];
    rtb_VectorConcatenate_l[i_1] += rtb_VectorConcatenate_k[i_1 + 4] * tmp_8[1];
    rtb_VectorConcatenate_l[i_1] += rtb_VectorConcatenate_k[i_1 + 8] * tmp_8[2];
    rtb_VectorConcatenate_l[i_1] += rtb_VectorConcatenate_k[i_1 + 12] * tmp_8[3];

    /* Product: '<Root>/Product4' incorporates:
     *  Sum: '<Root>/Sum3'
     */
    tmp_9[i_1] = 0.0;
    for (i = 0; i < 8; i++) {
      tmp_9[i_1] += tmp_b[(i << 2) + i_1] * tmp_c[i];
    }

    /* Sum: '<Root>/Sum3' incorporates:
     *  Gain: '<Root>/Gain5'
     */
    rtb_VectorConcatenate_l_0[i_1] = rtb_VectorConcatenate_l[i_1] - tmp_9[i_1];
  }

  for (i_1 = 0; i_1 < 4; i_1++) {
    /* Gain: '<Root>/Gain5' */
    rtb_TrigonometricFunction_o1 = autopilot_hinf_ConstP.Gain5_Gain[i_1 + 12] *
      rtb_VectorConcatenate_l_0[3] + (autopilot_hinf_ConstP.Gain5_Gain[i_1 + 8] *
      rtb_VectorConcatenate_l_0[2] + (autopilot_hinf_ConstP.Gain5_Gain[i_1 + 4] *
      rtb_VectorConcatenate_l_0[1] + autopilot_hinf_ConstP.Gain5_Gain[i_1] *
      rtb_VectorConcatenate_l_0[0]));

    /* Outport: '<Root>/Va' */
    autopilot_hinf_Y.Va[i_1] = rtb_TrigonometricFunction_o1;

    /* Outport: '<Root>/omega' incorporates:
     *  Constant: '<S6>/Constant'
     *  Constant: '<S6>/Constant1'
     *  Gain: '<S6>/Gain'
     *  Gain: '<S6>/Gain1'
     *  Sqrt: '<S6>/Sqrt'
     *  Sum: '<S6>/Add'
     *  Sum: '<S6>/Add1'
     */
    autopilot_hinf_Y.omega[i_1] = (std::sqrt(9.546539140811456E-6 *
      rtb_TrigonometricFunction_o1 + 1.7556100000000003E-5) + -0.00419) *
      209500.00523750013;

    /* Saturate: '<Root>/Saturation' */
    if (rtb_TrigonometricFunction_o1 > 5.7041741760143205) {
      rtb_TrigonometricFunction_o2 = 5.7041741760143205;
    } else if (rtb_TrigonometricFunction_o1 < 0.0) {
      rtb_TrigonometricFunction_o2 = 0.0;
    } else {
      rtb_TrigonometricFunction_o2 = rtb_TrigonometricFunction_o1;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Outport: '<Root>/Va_s' */
    autopilot_hinf_Y.Va_s[i_1] = rtb_TrigonometricFunction_o2;

    /* Sqrt: '<S5>/Sqrt' incorporates:
     *  Constant: '<S5>/Constant'
     *  Gain: '<S5>/Gain'
     *  Sum: '<S5>/Add'
     */
    rtb_TrigonometricFunction_o2 = std::sqrt(9.546539140811456E-6 *
      rtb_TrigonometricFunction_o2 + 1.7556100000000003E-5);

    /* Outport: '<Root>/omega_s' incorporates:
     *  Constant: '<S5>/Constant1'
     *  Gain: '<S5>/Gain1'
     *  Sum: '<S5>/Add1'
     */
    autopilot_hinf_Y.omega_s[i_1] = (rtb_TrigonometricFunction_o2 + -0.00419) *
      209500.00523750013;

    /* Outport: '<Root>/omega_d' */
    autopilot_hinf_Y.omega_d[i_1] = rtb_TrigonometricFunction_o1;

    /* Saturate: '<Root>/Saturation1' */
    if (rtb_TrigonometricFunction_o1 > 900.0) {
      /* Outport: '<Root>/omega_ds' */
      autopilot_hinf_Y.omega_ds[i_1] = 900.0;
    } else if (rtb_TrigonometricFunction_o1 < 113.0) {
      /* Outport: '<Root>/omega_ds' */
      autopilot_hinf_Y.omega_ds[i_1] = 113.0;
    } else {
      /* Outport: '<Root>/omega_ds' */
      autopilot_hinf_Y.omega_ds[i_1] = rtb_TrigonometricFunction_o1;
    }

    /* End of Saturate: '<Root>/Saturation1' */
  }

  if (rtmIsMajorTimeStep((&autopilot_hinf_M))) {
    rt_ertODEUpdateContinuousStates(&(&autopilot_hinf_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&autopilot_hinf_M)->Timing.clockTick0)) {
      ++(&autopilot_hinf_M)->Timing.clockTickH0;
    }

    (&autopilot_hinf_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&autopilot_hinf_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&autopilot_hinf_M)->Timing.clockTick1++;
      if (!(&autopilot_hinf_M)->Timing.clockTick1) {
        (&autopilot_hinf_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void autopilot_hinfModelClass::autopilot_hinf_derivatives()
{
  XDot_autopilot_hinf_T *_rtXdot;
  _rtXdot = ((XDot_autopilot_hinf_T *) (&autopilot_hinf_M)->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = autopilot_hinf_B.e[0];
  _rtXdot->Integrator_CSTATE[1] = autopilot_hinf_B.e[1];
  _rtXdot->Integrator_CSTATE[2] = autopilot_hinf_B.e[2];
  _rtXdot->Integrator_CSTATE[3] = autopilot_hinf_B.e[3];
}

/* Model initialize function */
void autopilot_hinfModelClass::initialize()
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)(&autopilot_hinf_M), 0,
                sizeof(RT_MODEL_autopilot_hinf_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&autopilot_hinf_M)->solverInfo, &(&autopilot_hinf_M
                          )->Timing.simTimeStep);
    rtsiSetTPtr(&(&autopilot_hinf_M)->solverInfo, &rtmGetTPtr((&autopilot_hinf_M)));
    rtsiSetStepSizePtr(&(&autopilot_hinf_M)->solverInfo, &(&autopilot_hinf_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&autopilot_hinf_M)->solverInfo, &(&autopilot_hinf_M)->derivs);
    rtsiSetContStatesPtr(&(&autopilot_hinf_M)->solverInfo, (real_T **)
                         &(&autopilot_hinf_M)->contStates);
    rtsiSetNumContStatesPtr(&(&autopilot_hinf_M)->solverInfo,
      &(&autopilot_hinf_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&autopilot_hinf_M)->solverInfo,
      &(&autopilot_hinf_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&autopilot_hinf_M)->solverInfo,
      &(&autopilot_hinf_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&autopilot_hinf_M)->solverInfo,
      &(&autopilot_hinf_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&autopilot_hinf_M)->solverInfo, (&rtmGetErrorStatus
      ((&autopilot_hinf_M))));
    rtsiSetRTModelPtr(&(&autopilot_hinf_M)->solverInfo, (&autopilot_hinf_M));
  }

  rtsiSetSimTimeStep(&(&autopilot_hinf_M)->solverInfo, MAJOR_TIME_STEP);
  (&autopilot_hinf_M)->intgData.y = (&autopilot_hinf_M)->odeY;
  (&autopilot_hinf_M)->intgData.f[0] = (&autopilot_hinf_M)->odeF[0];
  (&autopilot_hinf_M)->intgData.f[1] = (&autopilot_hinf_M)->odeF[1];
  (&autopilot_hinf_M)->intgData.f[2] = (&autopilot_hinf_M)->odeF[2];
  (&autopilot_hinf_M)->intgData.f[3] = (&autopilot_hinf_M)->odeF[3];
  (&autopilot_hinf_M)->intgData.f[4] = (&autopilot_hinf_M)->odeF[4];
  (&autopilot_hinf_M)->intgData.f[5] = (&autopilot_hinf_M)->odeF[5];
  (&autopilot_hinf_M)->contStates = ((X_autopilot_hinf_T *) &autopilot_hinf_X);
  rtsiSetSolverData(&(&autopilot_hinf_M)->solverInfo, (void *)
                    &(&autopilot_hinf_M)->intgData);
  rtsiSetSolverName(&(&autopilot_hinf_M)->solverInfo,"ode5");
  rtmSetTPtr((&autopilot_hinf_M), &(&autopilot_hinf_M)->Timing.tArray[0]);
  (&autopilot_hinf_M)->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) &autopilot_hinf_B), 0,
                sizeof(B_autopilot_hinf_T));

  /* states (continuous) */
  {
    (void) memset((void *)&autopilot_hinf_X, 0,
                  sizeof(X_autopilot_hinf_T));
  }

  /* external inputs */
  (void)memset((void *)&autopilot_hinf_U, 0, sizeof(ExtU_autopilot_hinf_T));

  /* external outputs */
  (void) memset((void *)&autopilot_hinf_Y, 0,
                sizeof(ExtY_autopilot_hinf_T));

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  autopilot_hinf_X.Integrator_CSTATE[0] = 0.0;
  autopilot_hinf_X.Integrator_CSTATE[1] = -0.0;
  autopilot_hinf_X.Integrator_CSTATE[2] = 18.3086815399726;
  autopilot_hinf_X.Integrator_CSTATE[3] = -0.0;
}

/* Model terminate function */
void autopilot_hinfModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
autopilot_hinfModelClass::autopilot_hinfModelClass()
{
}

/* Destructor */
autopilot_hinfModelClass::~autopilot_hinfModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_autopilot_hinf_T * autopilot_hinfModelClass::getRTM()
{
  return (&autopilot_hinf_M);
}
