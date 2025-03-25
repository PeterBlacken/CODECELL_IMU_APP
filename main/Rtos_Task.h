#pragma once
#include "imu_header.h"
#include <Arduino.h>
#include <cstddef>
//#include <imu_header.h>
// Handlers
TaskHandle_t LED_Task_Handle = NULL; 
TaskHandle_t Task_IMURead_Handle = NULL;
TaskHandle_t Task_SerialShow_Handle = NULL;
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

////////////////////////////////Init Task///////////////////////////////////

void init_freertos_tasks()
{
	IMU_fifo = xQueueCreate(FIFO_SIZE,sizeof(IMU_data_t));
	if(IMU_fifo==NULL)
	{Serial.print("error creating queue"); while(1);}
	



	xTaskCreate(TaskLEDTest,"TaskGlowLed",1000,NULL,1,&LED_Task_Handle);
	xTaskCreate(TaskReadIMUData,"TaskIMURead",2048,NULL,1,&Task_IMURead_Handle);
	xTaskCreate(TaskSerialShow,"TaskSerialShow",2048,NULL,1,&Task_SerialShow_Handle);

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
		{Serial.println("data added to FIFO ");}
		else{Serial.println("FIFO FULL");}
		vTaskDelay(pdMS_TO_TICKS(IMU_MS_UPDATE));
	
	}
}

////////////////////////////////////
/// 

void TaskSerialShow(void *pvParameters)
{
	IMU_data_t d;

	while(true)
	{
		if(xQueueReceive(IMU_fifo,&d,portMAX_DELAY)==pdPASS)
		{
			Serial.printf("time= %lld, ax = %f, ay= %f, az=%f, gx=%f, gy=%f, gz=%f, mx=%f, my=%f, mz=%f ",d.time_stamp,d.acc_x,d.acc_y,d.acc_z,d.gyr_x,d.gyr_y,d.gyr_z,d.mag_x,d.mag_y,d.mag_z);
		}
	}


}









