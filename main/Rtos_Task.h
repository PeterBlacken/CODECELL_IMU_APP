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
TaskHandle_t Task_Monitor_Handle		 = NULL;
TaskHandle_t Task_QueueMonitor_Handle= NULL;

// Queues
QueueHandle_t IMU_fifo;



// defines
#define FIFO_SIZE 1250

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
void TaskMonitor(void *pvParameters);
void TaskQueueMonitor(void *pvParameters);
////////////////////////////////Init Task///////////////////////////////////

void init_freertos_tasks()
{
	IMU_fifo = xQueueCreate(FIFO_SIZE,sizeof(IMU_data_t));
	if(IMU_fifo==NULL)
	{Serial.print("error creating queue"); while(1);}
	



	//xTaskCreate(TaskLEDTest,"TaskGlowLed",1000,NULL,1,&LED_Task_Handle);
	//vTaskSuspend(LED_Task_Handle); // pause the Task

	xTaskCreate(TaskReadIMUData,"TaskIMURead",4096,NULL,5,&Task_IMURead_Handle);
	//xTaskCreate(TaskSerialShow,"TaskSerialShow",2048,NULL,1,&Task_SerialShow_Handle);	
	//vTaskSuspend(Task_SerialShow_Handle); // pause the Task

	xTaskCreate(TaskBLE,"TaskBLE",5096,NULL,23,&Task_BLE_Handle);

		//task monitor
	xTaskCreate(TaskMonitor, "TaskMonitor", 2048, NULL, 1, NULL);
	xTaskCreate(TaskQueueMonitor, "QueueMonitor", 2048, NULL, 1, NULL);
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
	char buffer[sizeof(IMU_data_t)+10];

	vTaskSuspend(Task_IMURead_Handle);
	while (true) 
	{
  	 	//BLEDevice central = BLE.central();
  		rainbow(); // rainbow led till the Bluetooth is connected

  		if(NimBLEDevice::getServer()->getConnectedCount())
  		{
				
  		  Serial.println("Connected to central device");
  		  Serial.print("Device MAC address: ");
  		  //Serial.println(central.address());	    
		  vTaskResume(Task_IMURead_Handle);
		  LED(1u,1u,1u);// device connected and sending data 
  			while (NimBLEDevice::getServer()->getConnectedCount()) // Mientras siga conectado
      	{
        		if (xQueueReceive(IMU_fifo, &d, portMAX_DELAY) == pdPASS)
        		{
          		imuDataToCSV_char(&d, buffer, sizeof(buffer));

          		// Enviar por BLE
          		pSensorCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
          		pSensorCharacteristic->notify(); // Notifica el valor actual
        		}
      	}
  		  LED(0,0,0);
		  // Re advertising if get disconected
		  Serial.print("Client disconected");
		  NimBLEDevice::startAdvertising();
  		
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


//Task Monitor
void TaskMonitor(void *pvParameters)
{
	while (true)
	{
		printf("\n====== MONITOR DE SISTEMA ======\n");

		if (LED_Task_Handle)
			printf("[LED Task]      Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(LED_Task_Handle),
			       uxTaskGetStackHighWaterMark(LED_Task_Handle) * sizeof(StackType_t));
		/*
		if (Task_TaskManager_Handle)
			printf("[TaskManager]   Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(Task_TaskManager_Handle),
			       uxTaskGetStackHighWaterMark(Task_TaskManager_Handle) * sizeof(StackType_t));
		
		if (Task_ReadTime_Handle)
			printf("[ReadTime]      Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(Task_ReadTime_Handle),
			       uxTaskGetStackHighWaterMark(Task_ReadTime_Handle) * sizeof(StackType_t));
		*/
		if (Task_IMURead_Handle)
			printf("[IMUReadData]   Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(Task_IMURead_Handle),
			       uxTaskGetStackHighWaterMark(Task_IMURead_Handle) * sizeof(StackType_t));

		if (Task_BLE_Handle)
			printf("[BLE]           Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(Task_BLE_Handle),
			       uxTaskGetStackHighWaterMark(Task_BLE_Handle) * sizeof(StackType_t));

		if (LED_Task_Handle)
			printf("[LED]           Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(LED_Task_Handle),
			       uxTaskGetStackHighWaterMark(LED_Task_Handle) * sizeof(StackType_t));

		if (Task_SerialShow_Handle)
			printf("[SerialShow]    Prio: %2u | Stack libre min: %4u bytes\n",
			       uxTaskPriorityGet(Task_SerialShow_Handle),
			       uxTaskGetStackHighWaterMark(Task_SerialShow_Handle) * sizeof(StackType_t));

		// HEAP
		printf("\nHeap actual disponible: %u bytes\n", (unsigned int)esp_get_free_heap_size());
		printf("Heap mínimo histórico:  %u bytes\n", (unsigned int)esp_get_minimum_free_heap_size());

		printf("====================================\n");

		vTaskDelay(pdMS_TO_TICKS(10000)); // cada 10 segundos
	}
}


void TaskQueueMonitor(void *pvParameters)
{
	while (true)
	{
		printf("\n====== 📦 MONITOR DE COLAS FIFO ======\n");

		if (IMU_fifo) {
			UBaseType_t used = uxQueueMessagesWaiting(IMU_fifo);
			UBaseType_t free = uxQueueSpacesAvailable(IMU_fifo);
			printf("[IMU_fifo]         Ocupado: %2u | Libre: %2u | Total: %u\n", used, free, used + free);
		}
		/*
		if (IMU_reduced_fifo) {
			UBaseType_t used = uxQueueMessagesWaiting(IMU_reduced_fifo);
			UBaseType_t free = uxQueueSpacesAvailable(IMU_reduced_fifo);
			printf("[IMU_reduced_fifo] Ocupado: %2u | Libre: %2u | Total: %u\n", used, free, used + free);
		}
		*/
		printf("=======================================\n");

		vTaskDelay(pdMS_TO_TICKS(5000)); // cada 5 segundos
	}
}








