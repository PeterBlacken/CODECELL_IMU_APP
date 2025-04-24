//#include <esp32-hal-bt.c>
#include <NimBLEDevice.h>
#define	size_of_struct 12000
#define  chunk_size 128


const char* matrixUUID= "180C";
//BLEService sensorService(matrixUUID);
//BLEStringCharacteristic sensorCharacteristic(matrixUUID,BLERead | BLENotify,128);

// Creation of the server, service and characteristic for sending data
static NimBLEServer* pServer = nullptr;
static NimBLEService* pSensorService = nullptr;
static NimBLECharacteristic* pSensorCharacteristic = nullptr;

void Init_BLE()
{

	

  NimBLEDevice::init("CodeCell-IMU-Test");

  NimBLEDevice::setMTU(256); //setMTU after init the BLEDEVICE XDD else crash 
	if (!NimBLEDevice::setDeviceName("CodeCell-IMU Test")) 
	{Serial.print("Name not changed");}
  pServer = NimBLEDevice::createServer();

  // Crear servicio
  pSensorService = pServer->createService(matrixUUID);

  // Crear característica con notificación
  pSensorCharacteristic = pSensorService->createCharacteristic(
    matrixUUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  // Valor inicial
  pSensorCharacteristic->setValue("0");

  // Iniciar el servicio
  pSensorService->start();

  // Configurar publicidad
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pSensorService->getUUID());
  ////pAdvertising->setScanResponse(true);
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("IMU Peripheral (Sending Data) - NimBLE iniciado");
  Serial.printf("MTU = %i",NimBLEDevice::getMTU() );

}



