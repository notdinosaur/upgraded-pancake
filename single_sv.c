#include "single_sv.h"
#include "math.h"

void single_sv(SV *sv,float Uα,float Udc)
{
	float L,cos;
	//参数传递
	sv->Uα = Uα;
	//sv->Uβ = Uβ;
	sv->Udc = Udc;
	sv->Ts = 1;
	
	//L = sqrt(sv->Uα*sv->Uα+sv->Uβ*sv->Uβ);
	
	//扇区判断
	if(Uα>0)
	{
		sv->sector = 1;
	}
	else
	{
		sv->sector = 2;
	}
	
	//限幅
	if(sv->Uα>sv->Udc)
	{
		sv->Uα = sv->Udc;
	}

	if(sv->Uα<-1*sv->Udc)
	{
		sv->Uα = -1*sv->Udc;
	}
	
	//时间计算
	switch(sv->sector)
	{
		case 1:
			sv->T1 = (sv->Uα/sv->Udc)/sv->Ts;
			sv->T0 = sv->Ts - sv->T1;
			break;
		case 2:
			sv->T2 = -(sv->Uα/sv->Udc)/sv->Ts;
			sv->T0 = sv->Ts - sv->T2;
			break;	
	}
	
	// //五段式分配
	switch(sv->sector)
	{
		case 1:
			sv->CCR1 = ((sv->T1 + sv->T0/2.0f)/sv->Ts) * 4666;
			sv->CCR2 = (sv->T0/2.0f)/sv->Ts * 4666;
			break;
		case 2:
			sv->CCR1 = (sv->T0/2.0f)/sv->Ts * 4666;
			sv->CCR2 = ((sv->T2 + sv->T0/2.0f)/sv->Ts) * 4666;
			break;			
	}


	// if(sv->CCR1 > 4666) sv->CCR1 = 4666;
	// if(sv->CCR1 < 0) sv->CCR1 = 0;
	
	// if(sv->CCR2 > 4666) sv->CCR2 = 4666;
	// if(sv->CCR2 < 0) sv->CCR2 = 0;


}
