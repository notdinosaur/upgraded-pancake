#ifndef single_sv_h
#define single_sv_h

typedef struct SV
{
	float Udc;
	float U¦Á,U¦Â;
	float Ts;
	float U1,U2;
	float T0,T1,T2;
	int sector;
	float CCR1,CCR2;



} SV;

void single_sv(SV *sv,float U¦Á,float Udc);



#endif
