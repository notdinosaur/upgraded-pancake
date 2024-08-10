#include "control.h"
#include "arm_math.h"


void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo)//Wc调节带宽，一般为0.628f(2*pi*0.1)
{
    float temp = 0;

    pr->Ts=Ts;
    pr->Kp=Kp;
    pr->Kr=Kr;
    pr->wc=wc;
    pr->wo=wo;

    temp = 4 / pr->Ts / pr->Ts + 4 * pr->wc / pr->Ts + pr->wo * pr->wo;

    pr->B0 = (4 * pr->Kp / pr->Ts / pr->Ts + 4 * pr->wc * (pr->Kp + pr->Kr) / pr->Ts+ pr->Kp * pr->wo * pr->wo) / temp;
    pr->B1 = (-8 * pr->Kp / pr->Ts / pr->Ts >Ts / pr->Ts + 2 * pr->Kp * pr->wo * pr->wo) / temp;
    pr->B2 = (4 * pr->Kp / - 4 * pr->wc / pr->Ts * (pr->Kp + pr->Kr)+ pr->Kp * pr->wo * pr->wo) / temp;
    pr->A1 = (-8 / pr->Ts / pr->Ts + 2 * pr->wo * pr->wo) / temp;
    pr->A2 = (4 / pr->Ts / pr->Ts - 4 * pr->wc / pr->Ts + pr->wo * pr->wo) / temp;/*Bo,B1，B2，A1，A2为Z域下的相关系数*/


}

float PR_calc(PR *pr,float target,float actual)
{
    float error=0;
    pr->target=target;
    pr->actual=actual;
    
    error = pr->target - pr->actual;
    pr->vi = error;

    /*y[n]+A1y[n-1]+A2y[n-2]=B0x[n]+B1x[n-1]+B2x[n-2]由Z域传函离散化得到差分方程*/
    pr->vo = -pr->A1 * pr->vo_1 - pr->A2 * pr->vo_2 + pr->B0 * pr->vi + pr->B1 * pr->vi_1+ pr->B2 * pr->vi_2;

    /*误差传递*/
    pr->vo_2 = pr->vo_1;
    pr->vo_1 = pr->vo;
    pr->vi_2 = pr->vi_1;
    pr->vi_1 = pr->vi;


    if(pr->vo>20)  pr->vo = 20;
    if(pr->vo<-1*20)  pr->vo = -1*100;


    return pr->vo;/*返回控制器的输出值*/
}


void SOGI_init(SOGI *sg,float k,float w,float Ts) // 基于广义二重积分的正交变换
{
	
		 float x = 0;
     float y = 0;
	
	   sg->k = k;
	   sg->w = w;
	   sg->Ts = Ts;
     

     x = 2.0f * sg->k * sg->w *sg->Ts;       
     y = sg->w * sg->w * sg->Ts * sg->Ts;  


     sg->B0 = x/(x+y+4.0f);
     sg->B2 = -1.0f*(sg->B0);
     sg->A1 = (2.0f*(4.0f-y))/(x+y+4.0f);
     sg->A2 = (x-y-4.0f)/(x+y+4.0f);
     sg->QB0 = (sg->k*y)/(x+y+4.0f);
     sg->QB1 = 2.0f*sg->QB0;
     sg->QB2 = sg->QB0;
}

void SOGI_Transfer(SOGI *sg,float v)
{

     sg->vi = v;
	
     /*离散化所得差分方程*/
     sg->vo = sg->A1 * sg->vo_1 + sg->A2 * sg->vo_2 + sg->B0 * sg->vi + sg->B2 * sg->vi_2;
     sg->qvo = sg->A1 * sg->qvo_1 + sg->A2 * sg->qvo_2 + sg->QB0 * sg->vi + sg->QB1 * sg->vi_1 + sg->QB2 * sg->vi_2;


     /*误差更迭*/
     sg->vo_2 = sg->vo_1;
     sg->vo_1 = sg->vo;
     sg->vi_2 = sg->vi;
	
		 sg->vo = 2.0f*sg->vo;//补偿
	
     /*误差更迭*/
     sg->qvo_2 = sg->qvo_1;
     sg->qvo_1 = sg->qvo;
     sg->qvi_2 = sg->qvi_1;
     sg->qvi_1 = sg->qvi;

		 sg->qvo = 2*sg->qvo;

}

void Park_transfer(Transfer *tran,float Ualpha,float Ubeta,float sin,float cos)
{
		//参数传递
		tran->Ualpha = Ualpha;
		tran->Ubeta = Ubeta;
		tran->Sin = sin;
		tran->Cos = cos;
	
	
	  tran->Ud = tran->Cos*tran->Ualpha + tran->Sin*tran->Ubeta;
		tran->Uq = tran->Cos*tran->Ubeta - tran->Sin*tran->Ualpha;
	
	
}

void Inverse_Park_Transfer(Transfer *tran,float Ud,float Uq,float sin,float cos)
{
		//参数传递
		tran->Ud = Ud;
		tran->Uq = Uq;
		tran->Sin = sin;
		tran->Cos = cos;
	
		tran->Ualpha = tran->Ud*tran->Cos - tran->Uq*tran->Sin;
		tran->Ubeta = tran->Ud*tran->Sin + tran->Uq*tran->Cos;
	
	
}

void PI_init(PID *pi,float Kp,float Ki,float Kd,float Ts)
{
    float temp = 0;

    //参数传递
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->Kd = Kd;
    pi->Ts = Ts;

    temp = 2.0f/Ts;
    pi->A_0 = (2.0f*(pi->Kp/pi->Ts)+pi->Ki)/temp;
    pi->A_1 = (2.0f*(pi->Kp/pi->Ts)-pi->Ki)/temp;

    pi->vo = 0;
    pi->vo_1 = 0;
    pi->vi = 0;
    pi->vo_1 = 0;

}

void CurrentPIControl(PID *pi,float target,float actual)
{
    float err=0;

    pi->target =  target;
    pi->actual = actual;
  
    err = pi->target - pi->actual;
    pi->vi = err;


    pi->vo = pi->A_0*pi->vi - pi->A_1*pi->vi_1 + pi->vo_1;

    pi->vi_1 = pi->vi;
    pi->vo_1 = pi->vo; 


    if(pi->vo>50)  pi->vo=50;
    if(pi->vo<-50)  pi->vo=-50;

}

void pi_init(PID *pi)
{
    pi->pre=0;
    pi->tar=0;
    pi->bias=0;
    pi->Integral=0;
    pi->out=0;


}

float CurrentPIControl_1(PID *pi,float Kp,float Ki,float tar,float pre)
{
    //参数传递
    pi->Kp = Kp;
    pi->Ki = Ki;

    pi->pre = pre;
    pi->tar = tar;

    //误差计算
    pi->bias = pi->tar - pi->pre;
	
		pi->Integral += pi->bias;
	
    //计算PI控制器的输出值
    pi->out = pi->Kp * pi->bias + pi->Ki * pi->Integral;

	
		if(pi->out>15) pi->out = 15;
		if(pi->out<-10) pi->out = -10;

    return pi->out;

}
float low_pass_filtering(float Ui)
{
  float value,last_value;
  float a = 0.5;
  last_value = value;
  value = Ui;

  return a*value+(1-a)*last_value;
}

float high_pass_filtering(float Ui)
{
  float value,last_value;
  float output,last_output;
  float a = 0.5;

  output = a*last_output+a*(value-last_value);

  last_output = output;
  last_value = value;
  value = Ui;

  return output;
}

	
	













