#pragma once


typedef struct pid_struct//记录偏差的结构体
{
	float p = 0;
	float i = 0;
	float d = 0;
	float err = 0;
	float last_err = 0;
	float err_d = 0;
	float err_i = 0;
	int index_enable = 0;//变积分功能开关 0为关闭，1为开启
	float index_max = 0;//变积分上限
	float index_min = 0;//变积分下限（这两项均为非负数）
}PID_S;



float PID_control_float(float target, float real, PID_S* PID);		//PID控制器，参数依次是目标值，实际值，PID结构体
float cascadePID_control_float(float target, float real, PID_S* PID,  float index_max, float index_min, //外环参数
											 float inner_real, PID_S* inner_PID);//内环参数
void param_init(void);															//参数初始化
