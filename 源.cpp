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
	pid_test.p = 16;//16，该参数可以提高相应速度，增大到你觉得合适为止。
	pid_test.i = 0;
	pid_test.d = 0;
	pid_test.err = 0;
	pid_test.last_err = 0;
	pid_test.err_d = 0;
	pid_test.err_i = 0;
	pid_test.index_enable = 0;
	pid_test.index_max = 100;
	pid_test.index_min = 0;

	pid_inner_test.p = 32;//32，该参数增大时震荡频率也会增大，但是震荡幅度减小
	pid_inner_test.i = 0;//0，该参数可以抵抗静态差，太大会造成震荡
	pid_inner_test.d = 0;//0，该参数可以抵抗外部干扰，但是有一个上限阈值，此阈值跟系统采样率，系统函数，系统延时均有关，如果干扰不是很明显建议置0
	pid_inner_test.err = 0;
	pid_inner_test.last_err = 0;
	pid_inner_test.err_d = 0;
	pid_inner_test.err_i = 0;
	pid_test.index_enable = 1;
	pid_inner_test.index_max = 10;
	pid_inner_test.index_min = 5;//这里取第errv（目标速度-当前速度）最终变化缓慢的值
	//调参技巧关注指标，1、x对于目标值响应速度和稳定性，2、errv对于0的响应速度和稳定性
	//个人推荐顺序：
	//1、内外环P顺便置入一个值。如1使得系统能够勉强稳定在任何一点
	//2、此时系统可能做明显的正弦运动但随着阻尼逐渐稳定，如果发现振动的中心与目标值有明显的偏差，一点一点的提高内环i，使系统能在目标值附近振动
	//3、此时将外环P调整到你满意的响应速度。（指第一次到达目标位置）
	//4、如果系统稳定速度慢，加大内环P使系统速度跟随能够更好地跟随目标速度（随着p加大，振动频率可能更大，但是振动幅度更小）
	//将p调整到振动幅度让自己满意（有临界值，不能太高）
	//5、如果系统表现良好则调参完成，若调完发现系统还有静态差，微调内环i，若调节过程中系统振动次数多且频率快，略微提高内环d进一步减小振动幅度和振动次数。
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
		if (count%10 == 0)//每十次变化显示一次数据，即显示速率为物理上的Fre/10 hz
		printf("x=%03.0f errv=%03.0f\t", x, pid_inner_test.err);//窗口调试x：当前位置，errv:(目标速度-当前速度)

		if (count++ == 60)//每六个换一次行
		{
			printf("\n");
			count = 0;
		}
		for (int i = 0; i < 10; i++)
			delay(500000);
	}
}

