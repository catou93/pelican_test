/*
 * autopilot_hinf_private.h
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

#ifndef RTW_HEADER_autopilot_hinf_private_h_
#define RTW_HEADER_autopilot_hinf_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

/* private model entry point functions */
extern void autopilot_hinf_derivatives();

#endif                                 /* RTW_HEADER_autopilot_hinf_private_h_ */
