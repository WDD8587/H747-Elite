#ifndef FOC_PARAMS_H
#define FOC_PARAMS_H

#include <math.h>

/* Current loop PI gains */
#define Kp_id      0.5f
#define Ki_id      10.0f
#define Kp_iq      0.5f
#define Ki_iq      10.0f

/* Speed loop PI gains */
#define Kp_speed   0.1f
#define Ki_speed   2.0f

/* References */
#define Id_ref     0.0f
#define MAX_CURRENT_A 3.0f

/* Bus voltage */
#define VBUS_NOMINAL    12.0f
#define VBUS_NOMINAL_V  24.0f

/* Math constants not in all math.h */
#ifndef M_SQRT3
#define M_SQRT3         1.7320508075688772f
#endif
#define INV_SQRT3       0.5773502691896257f  /* 1/sqrt(3) for Clarke transform */

#endif
