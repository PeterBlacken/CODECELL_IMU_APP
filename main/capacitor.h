/*
 *El metodo de eurler es
 * y_n+1=y_n+Delta*f'(x)
 * */


#include <ArduinoBLE.h>
//#include <Timer_header.h>
//#include <cstdint>
const float Vs= 40;
const float R = 8e4;
const float C = 4e-6;

const float Vc_0=0;
float Vc_n;
float Vc_n1;

const float delta_time=0.0001;
const int end_step=200;

const float tau= R*C;

typedef struct 
{
	int64_t timeStamp;
	float	  time_x;
	float	  solve_y;
}solve_element_t;

typedef struct{solve_element_t matrix[end_step];} solve_matrix_t;

//float fx(float x){return (x*x)-2;}

float Dot_fx(float Vc){return (Vs-Vc)/(R*C);}

void solve()
{
	solve_matrix_t m;
	Vc_n=Vc_0;	
	
	for (int step=0;step<end_step;step++)
	{
		m.matrix[step].solve_y=Vc_n;
		m.matrix[step].time_x=delta_time*step;
		
		Vc_n1=Vc_n+step*delta_time*Dot_fx(Vc_n);
		
		Vc_n=Vc_n1;
		m.matrix[step].timeStamp=get_time_us();
		
	}
	
	Serial.print("END");	
	for (int step=0; step<end_step ; step++)
	{
		Serial.printf("TimeStamp = %i uS, Time = %.5f, VC= %.5f step= %i \n",m.matrix[step].timeStamp, m.matrix[step].time_x,m.matrix[step].solve_y,step);
		//delay(200);
	}

}


