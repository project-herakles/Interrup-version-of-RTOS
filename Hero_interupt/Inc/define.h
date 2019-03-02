
#ifndef __DEFINE_H
#define __DEFINE_H

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	80.0f,\
	3.0f,\
	5.0f,\
	0,\
	0,\
	0,\
	5000,\
	0,\
	800,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define YAW_ENCODER_DEFAULT {0,0,0,0,0,0,6369,0,0,0,0,0,0,0}

#define PITCH_ENCODER_DEFAULT {0,0,0,0,0,0,1645,0,0,0,0,0,0,0}


#endif
