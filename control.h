#ifndef control_h
#define control_h

typedef struct SOGI
{
    float k;
    float w;
    float Ts;
    float A1,A2,B0,B2,QB0,QB1,QB2;
    float vo,vo_1,vo_2;
    float vi,vi_1,vi_2;
    float qvo,qvo_1,qvo_2;
	float qvi,qvi_1,qvi_2;

} SOGI;


typedef struct PR
{
    float Kp;
    float Kr;
    float wo;
    float wc;
    float Ts;
    float A0, A1, A2, B0, B1, B2;
    float vo, vo_1, vo_2;
    float vi, vi_1, vi_2;
    float target,actual;
	
} PR;

typedef struct Transfer
{
		float Ualpha,Ubeta;
		float Ud,Uq;
		float Sin,Cos;

} Transfer;

typedef struct 
{
		float Kp,Ki,Kd;
    float target,actual;
    float Ts;
    float A_0,A_1;
    float vo,vo_1,vo_2,vi,vi_1,vi_2;
    float pre,tar;
    float bias,lastBias;
		float Integral;
    float out;
	
} PID;



void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo);
float PR_calc(PR *pr,float target,float actual);
void SOGI_init(SOGI *sg,float k,float w,float Ts);
void SOGI_Transfer(SOGI *sg,float v);
void Park_transfer(Transfer *tran,float U¦Á,float U¦Â,float sin,float cos);
void Inverse_Park_Transfer(Transfer *tran,float Ud,float Uq,float sin,float cos);
void CurrentPIControl(PID *pi,float target,float actual);
float CurrentPIControl_1(PID *pi,float Kp,float Ki,float tar,float pre);
void PI_init(PID *pi,float Kp,float Ki,float Kd,float Ts);
void pi_init(PID *pi);
float low_pass_filtering(float Ui);
float high_pass_filtering(float Ui);





#endif
