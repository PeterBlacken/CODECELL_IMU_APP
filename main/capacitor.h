/*
 *El metodo de eurler es
 * y_n+1=y_n+Delta*f'(x)
 * */


#include <ArduinoBLE.h>
//#include <Timer_header.h>
//#include <cstdint>
const float Vs= 40;
const float R = 8e3;
const float C = 4e-6;

#define	size_of_struct 8
const float Vc_0=0;
float Vc_n;
float Vc_n1;

const float delta_time=0.0001;
const int end_step=size_of_struct;

const float tau= R*C;

typedef struct 
{
	int64_t timeStamp; // 8bytes
	float	  time_x;	 // 4Bytes
	float	  solve_y;   // 4Bytes
}solve_element_t; //16 bytes per matrix 

typedef struct{solve_element_t matrix[end_step];} solve_matrix_t;
static solve_matrix_t m0; // asigno el struct a la memoria static dynamica porque o si no la stack se overflow
static solve_matrix_t m1; 

//typedef struct{solve_element_t matrix[1];} size_matrix_t;
//static size_matrix_t m1;


// intentando enviar la matrix entera 
const char* matrixUUID= "180C";
BLEService sensorService(matrixUUID);
BLEStringCharacteristic sensorCharacteristic(matrixUUID,BLERead | BLENotify,256);


void Init_BLE()
{

   if (!BLE.begin()) {
      Serial.println("starting Bluetooth® Low Energy module failed!");
      while (1);}
	Serial.printf("BLE Inicializado");

    /////////////////////////BLE INIT///////////// 

    BLE.setLocalName("IMU");
    BLE.setAdvertisedService(IMUService);
	 /////////////////////
	 BLE.setAdvertisedService(sensorService); //se agrega el servicio para enviar matrix
	 sensorService.addCharacteristic(sensorCharacteristic);
	 BLE.addService(sensorService);
	 ///////////////////////////
    IMUService.addCharacteristic(AccXChar);
    IMUService.addCharacteristic(AccYChar);
    IMUService.addCharacteristic(AccZChar);
    IMUService.addCharacteristic(GyroXChar);
    IMUService.addCharacteristic(GyroYChar);
    IMUService.addCharacteristic(GyroZChar);
    BLE.addService(IMUService);
    BLE.advertise();

    Serial.println("IMU Peripheral (Sending Data)");
    //////////////////////////////////////////////////////////    
}



float Dot_fx(float Vc){return (Vs-Vc)/(R*C);}

void solve()
{
	Vc_n=Vc_0;	
	
	for (int step=0;step<end_step;step++)
	{
		m0.matrix[step].solve_y=Vc_n;
		m0.matrix[step].time_x=delta_time*step;
		
		Vc_n1=Vc_n+step*delta_time*Dot_fx(Vc_n);
	
		Vc_n=Vc_n1;
		m0.matrix[step].timeStamp=get_time_us();
		
	}	
	for (int step=0;step<end_step;step++)
	{
		m1.matrix[step].solve_y=Vc_n;
		m1.matrix[step].time_x=delta_time*step;
		
		Vc_n1=Vc_n+step*delta_time*Dot_fx(Vc_n);
	
		Vc_n=Vc_n1;
		m1.matrix[step].timeStamp=get_time_us();
		
	}
	
	Serial.println("END");	
	for (int step=0; step<end_step ; step++)
	{
		Serial.printf("m0: TimeStamp = %i uS, Time = %.5f, VC= %.5f step= %i \n",m0.matrix[step].timeStamp, m0.matrix[step].time_x,m0.matrix[step].solve_y,step);
		delay(20);
	}for (int step=0; step<end_step ; step++)
	{
		Serial.printf("m1: TimeStamp = %i uS, Time = %.5f, VC= %.5f step= %i \n",m1.matrix[step].timeStamp, m1.matrix[step].time_x,m1.matrix[step].solve_y,step);
		delay(20);
	}

}


