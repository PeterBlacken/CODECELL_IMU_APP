#pragma once
#include <Arduino.h>

TaskHandle_t LED_Task_Handle = NULL; 

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

////////////////////////////////Init Task///////////////////////////////////

void init_freertos_tasks()
{
	xTaskCreate(TaskLEDTest,"TaskGlowLed",1000,NULL,1,&LED_Task_Handle);
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

