
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
#define y_width 1
#define y_1_width 1
#define y_2_width 1

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
void Set_PWM_Outputs_wrapper(const real_T *motor,
			real_T *IN1,
			real_T *IN2,
			real_T *PWM)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
if(*motor < 0){
    *IN1 = 1;
    *IN2 = 0;
    *PWM = -1 * (*motor) + 10;
}
else{
    *IN1 = 0;
    *IN2 = 1;
    *PWM = *motor + 10;
}
if(*motor == 0){
    *IN1 = 0;
    *IN2 = 0;
    *PWM = 0;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


