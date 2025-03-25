// FreeRtos implementation
// V1
//  -Implementer Rtos task for led control
//  Created a new header for IMU funtions and variable
//  implemented while for initialized IMU, try until works (needs debugging) 
//  modification to function IMU_read(float,*uint64_t). Now you pass var by reference
//  this modification was made to work with a Task();
//  Created TaskReadIMUData()
//  Created TaskSerialShow() 
// IT WORKSSSSSSS taking  from the IMU in one task and showing the data in other different Task
//
//
#include <CodeCell.h>
#include"BLE_config.h"
#include <BNO085.h>
#include <imu_header.h>
#include <led.h>
#include <Timer_header.h>
//#include <capacitor.h>
#include <Rtos_Task.h>

CodeCell myCodeCell;

String receivedData = "";  // Almacenar el mensaje recibido
bool sendVariable[6] = {false, false, false, false, false, false}; // Estados de las variables
//const int size_of_struct= 200;

void setup() {
    Serial.begin(115200);
    while(!Serial); 
    led_init();
    IMU_Init(IMU_config,10,0xFFFFFFFF);
    
    while(err==1)
    {
      Serial.print("intento IMU");
      delay(100);
      err=0;
      IMU_Init(IMU_config,10,0xFFFFFFFF);
      //while(1){LED(100,0,0);delay(100);}
    }

    Init_BLE();
    init_freertos_tasks();
    /*
    if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);}

    Serial.print("BLE iniciado")
    
    BLE.setLocalName("CodeCell- IMU Test");
    BLE.setAdvertisedService(IMUService);
    */

    //delay(3000);

}

void loop() 
{
   


  /*
  BLEDevice central = BLE.central();
  rainbow(); // rainbow led till the Bluetooth is connected

  if (central) 
  {
    Serial.println("Connected to central device");
    Serial.print("Device MAC address: ");
    Serial.println(central.address());
      
    while(central.connected()) 
    {
      solve(); 

      LED(100u,100u,100u);// device connected and sending data 

    }
    LED(0,0,0);

  }
  */


}

void Show_IMU_data_UART(float *Data_IMU)
{
  // Leer datos del monitor serial
    while (Serial.available()) 
    {
      char c = Serial.read();
      if (c == '\n') 
      {
        // Si se presiona Enter
        int command = receivedData.toInt(); // Convertir a número
        if (command >= 1 && command <= 6) 
        {
          sendVariable[command - 1] = !sendVariable[command - 1]; // Cambiar estado
          Serial.print("Variable ");
          Serial.print(command);
          Serial.println(sendVariable[command - 1] ? " ACTIVADA" : " DESACTIVADA");
        }
        receivedData = ""; // Resetear buffer
      } else {receivedData += c; }
    }
    // Enviar solo las variables activadas
    if (sendVariable[0]) Serial.print(Data_IMU[0]);
    if (sendVariable[1]) Serial.print(","), Serial.print(Data_IMU[1]);
    if (sendVariable[2]) Serial.print(","), Serial.print(Data_IMU[2]);
    if (sendVariable[3]) Serial.print(","), Serial.print(Data_IMU[3]);
    if (sendVariable[4]) Serial.print(","), Serial.print(Data_IMU[4]);
    if (sendVariable[5]) Serial.print(","), Serial.println(Data_IMU[5]); // Última variable con println
    if (!sendVariable[5])Serial.println();
    return;                                                                  //
}


