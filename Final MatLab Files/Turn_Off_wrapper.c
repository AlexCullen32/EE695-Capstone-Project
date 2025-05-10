
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void Turn_Off_Outputs_wrapper(const uint16_T *sensorReading,
			const real_T *MotorIn,
			real_T *MotorOut)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
if(*sensorReading < (794 - 100) || *sensorReading > (794 + 100)){
        *IN1 = 0;
        *IN2 = 0;
        *PWM = 0;
}

else{
    *MotorOut = *MotorIn;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


