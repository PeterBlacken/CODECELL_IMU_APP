#pragma once
#include "imu_header.h"
#include <Arduino.h>
#include <cstddef>
#include <cstdint>
#include <util.h>
//#include <imu_header.h>
// Handlers
TaskHandle_t LED_Task_Handle = NULL; 
TaskHandle_t Task_IMURead_Handle = NULL;
TaskHandle_t Task_SerialShow_Handle = NULL;
TaskHandle_t Task_BLE_Handle = NULL;
// Queues
QueueHandle_t IMU_fifo;



// defines
#define FIFO_SIZE 125

// usefull funtions
//
// LED(uint8_t r, uint8_t g, uint8_t b)

//xTaskCreate(
//  tareaFuncion,      // Función de la tarea
//  "NombreTarea",     // Nombre identificador
//  1000,              // Tamaño de la pila
//  NULL,              // Parámetro (puedes pasar datos)
//  1,                 // Prioridad
//  NULL               // Handle (opcional, para referenciar luego)
//);

//xQueueCreate()	Crea una cola con un tamaño y tipo de dato fijo
//xQueueSend()	Envia un dato a la cola (desde una tarea o ISR)
//xQueueReceive()	Recibe un dato de la cola (bloquea hasta que haya datos si querés)
//xQueuePeek()	Mira el dato sin sacarlo de la cola
void TaskLEDTest(void *pvParameters);
void TaskReadIMUData(void*pvParameters);
void TaskSerialShow(void *pvParameters);
void TaskBLE(void *pvParameters);

////////////////////////////////Init Task///////////////////////////////////

void init_freertos_tasks()
{
	IMU_fifo = xQueueCreate(FIFO_SIZE,sizeof(IMU_data_t));
	if(IMU_fifo==NULL)
	{Serial.print("error creating queue"); while(1);}
	



	//xTaskCreate(TaskLEDTest,"TaskGlowLed",1000,NULL,1,&LED_Task_Handle);
	//vTaskSuspend(LED_Task_Handle); // pause the Task

	xTaskCreate(TaskReadIMUData,"TaskIMURead",2048,NULL,5,&Task_IMURead_Handle);
	//xTaskCreate(TaskSerialShow,"TaskSerialShow",2048,NULL,1,&Task_SerialShow_Handle);	
	//vTaskSuspend(Task_SerialShow_Handle); // pause the Task

	xTaskCreate(TaskBLE,"TaskBLE",5096,NULL,23,&Task_BLE_Handle);
}



//////////////////////////////Task declaration/////////////////////////////

void TaskLEDTest(void *pvParameters)
{
	while (true) 
	{
		for (int i = 0; i<255;i++) {LED(i,0,0);vTaskDelay(pdMS_TO_TICKS(10));}
		for (int i = 0; i<255;i++) {LED(0,i,0);vTaskDelay(pdMS_TO_TICKS(10));}
		for (int i = 0; i<255;i++) {LED(0,0,i);vTaskDelay(pdMS_TO_TICKS(10));}

	}
}

/// This task create an array for holding the data from the IMU 
/// and a var for the TimeStamp of this data. 
/// Then add the data to a FIFO for been used in other Task
/// data sampling = 125hz
void TaskReadIMUData(void *pvParameters)
{
	IMU_data_t dato;
	float IMU_snapshot[23];
	int64_t time_stamp_var;


	while(true)
	{
		IMU_read(IMU_snapshot, &time_stamp_var);
	
		//MUY OPTIMIZABLE REHACER PARAR PASAR POR REFERENCIA.
		dato.time_stamp=time_stamp_var;
		dato.acc_x = IMU_snapshot[8];
		dato.acc_y = IMU_snapshot[9];
		dato.acc_z = IMU_snapshot[10];
		dato.gyr_x = IMU_snapshot[11];
		dato.gyr_y = IMU_snapshot[12];
		dato.gyr_z = IMU_snapshot[13];
		dato.mag_x = IMU_snapshot[14];
		dato.mag_y = IMU_snapshot[15];
		dato.mag_z = IMU_snapshot[16];

		if(xQueueSend(IMU_fifo,&dato,portMAX_DELAY)==pdPASS)
		{//Serial.println("data added to FIFO ");
		 }
		else{Serial.println("FIFO FULL");}
		vTaskDelay(pdMS_TO_TICKS(IMU_MS_UPDATE));
	
	}
}
///////////////////// TaskBLE//////////////////////
// Es recomendable que el stack ocupado por el BLE este inicializado antes de
// su uso, por ende, ojala iniciarlo en el setup o antes del bucle principal
// de la TASK
void TaskBLE(void *pvParameters)
{
   IMU_data_t d;
	vTaskSuspend(Task_IMURead_Handle);
	char buffer[sizeof(IMU_data_t)+10];

	while (true) 
	{
	
  	 	BLEDevice central = BLE.central();
  		rainbow(); // rainbow led till the Bluetooth is connected

  		if (central) 
  		{
  		  Serial.println("Connected to central device");
  		  Serial.print("Device MAC address: ");
  		  Serial.println(central.address());	    
  		  LED(100u,100u,100u);// device connected and sending data 
  		  while(central.connected()) 
  		  {
			  
			 vTaskResume(Task_IMURead_Handle);
			 
			 if(xQueueReceive(IMU_fifo,&d,portMAX_DELAY)==pdPASS)
			 {
				 imuDataToCSV_char(&d,buffer,sizeof(buffer));
				 sensorCharacteristic.writeValue(buffer);	
				 
			 }
			 
  		
  		  }
  		  LED(0,0,0);
  		
  		}

	}
}





////////////////////////////////////
/// 

void TaskSerialShow(void *pvParameters)
{
	IMU_data_t d;
	int64_t time_n=0;
	int64_t time_n1=0;;


	while(true)
	{
		time_n=d.time_stamp;
		//cambiar por xQueuePeek(cola,&var,timeout) Esta función lee el dato sin sacarlo del buffer 
		if(xQueueReceive(IMU_fifo,&d,portMAX_DELAY)==pdPASS)
		{
			//Serial.printf("time= %lld, ax = %f, ay= %f, az=%f, gx=%f, gy=%f, gz=%f, mx=%f, my=%f, mz=%f ",d.time_stamp,d.acc_x,d.acc_y,d.acc_z,d.gyr_x,d.gyr_y,d.gyr_z,d.mag_x,d.mag_y,d.mag_z);
			time_n1=d.time_stamp;
		}
		Serial.printf("tiempo entre muestra= %lld \n ", time_n1-time_n);

	}


}










