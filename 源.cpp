#include "stdio.h"
#include "pid.h"
#include "math.h"

const float Fre = 1000.0f;//采样率（/hz）
PID_S pid_test;
PID_S pid_inner_test;
void delay(int t)
{
	while (t--);
}




float PID_control_float(float target, float real, PID_S* PID)
{
	float index=1;
	float control_value = 0.0f; //控制量
	PID->err = target - real;  //误差量
	PID->err_d = Fre*(PID->err - PID->last_err);//误差变化速度
	PID->err_i += (PID->err)/Fre;		//累加偏差
	if(PID->index_enable==1)//引入变积分
	{
		if (fabs(PID->err) > PID->index_max)        //变积分过程
		{
			index = 0;
		}
		else if (fabs(PID->err) < PID->index_min)
		{
			index = 1;
			PID->err_i += PID->err;    	//累加偏差
		}
		else
		{
			index = (PID->index_max - fabs(PID->err)) / (PID->index_max - (PID->index_min));
			PID->err_i += PID->err;    	//累加偏差
		} 
	}
	else index = 1;
	control_value += PID->p * PID->err;	//p控制
	control_value += PID->d * PID->err_d;	//d控制
	control_value += index * PID->i * PID->err_i;	//i控制

	PID->last_err = PID->err;	//记录上次数据

	return control_value;
}

float cascadePID_control_float(float target, float real, PID_S* PID,  //外环参数
							   float inner_real, PID_S* inner_PID)//内环参数
{
	return PID_control_float(PID_control_float(target, real, PID),//嵌套外环计算结果后输出
							inner_real, inner_PID);
}

void param_init(void)//
{
	pid_test.p = 16;//16
	pid_test.i = 0;
	pid_test.d = 0;
	pid_test.err = 0;
	pid_test.last_err = 0;
	pid_test.err_d = 0;
	pid_test.err_i = 0;
	pid_test.index_enable = 0;
	pid_test.index_max = 100;
	pid_test.index_min = 0;

	pid_inner_test.p = 32;//32
	pid_inner_test.i = 0;//0
	pid_inner_test.d = 0;//0，该参数有一个上限阈值，系统加速度==a*控制量时，该值（kd）超过1/a会不稳定(本系统为不超过1)
	pid_inner_test.err = 0;
	pid_inner_test.last_err = 0;
	pid_inner_test.err_d = 0;
	pid_inner_test.err_i = 0;
	pid_test.index_enable = 1;
	pid_inner_test.index_max = 10;
	pid_inner_test.index_min = 5;//这里取第errv最终变化缓慢的值
	//调参技巧关注指标，1、x对于目标值响应速度和稳定性，2、errv对于0的响应速度和稳定性
	//个人推荐顺序：
	//1、内外环P顺便置入一个值。如1使得系统能够勉强稳定
	//2、将外环P调整到你满意的响应速度。（指第一次到达目标位置）
	//3、如果发生长期不能稳定，加大内环P使系统速度跟随能够更好地跟随目标速度（此时振动频率可能更大，但是振动幅度更小）
	//将p调整到振动幅度让自己满意
	//4、如果系统表现良好则调参完成，若调完发现系统有小幅高频振动，将内环d调整到最高0.99（不超过1）。
	//结束（其他未提到的参数都不用调）
}


float PID_run(float _x, float _v)
{
	float a = 0;

	return(a = cascadePID_control_float(80, _x, &pid_test,
		_v, &pid_inner_test));//7行1秒)


}
int main()//测试
{
	//模拟二阶系统
	float a = 0;
	float x = 0;
	float v = 0;
	param_init();
	while (1)
	{
		static int count = 0;
		a = PID_run(x, v);
		v += a/Fre ;
		x += v/Fre ;
		if (count%10 == 0)
		printf("x=%03.0f errv=%03.0f\t", x, pid_inner_test.err);//窗口调试

		if (count++ == 60)
		{
			printf("\n");
			count = 0;
		}
		for (int i = 0; i < 10; i++)
			delay(500000);
	}
}

