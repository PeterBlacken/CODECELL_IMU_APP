#include <CodeCell.h>
#include"BLE_config.h"
#include <BNO085.h>
CodeCell myCodeCell;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

float Ax,Ay,Az,Gx,Gy,Gz;
float Data_IMU[6];
String receivedData = "";  // Almacenar el mensaje recibido
bool sendVariable[6] = {false, false, false, false, false, false}; // Estados de las variables
uint32_t IMU_config = 0b100001011010010000;

float RIJK[4];
float aceletation_matrix[3];
float magnetometer_matrix[3];
int acc_rot=0;
int acc_acc=0;

BNO085 IMU;
void IMU_Init(uint32_t config, uint16_t timeBetweenReports = 10, uint32_t activitiesToEnable = 0xFFFFFFFF);


void setup() {
    Serial.begin(115200);
    while(!Serial); 
    IMU_Init(IMU_config,10,0xFFFFFFFF);
    Init_BLE();

    /*
    if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);}

    Serial.print("BLE iniciado")
    
    BLE.setLocalName("CodeCell- IMU Test");
    BLE.setAdvertisedService(IMUService);
    */

    delay(3000);

}

void loop() 
{
   
  BLEDevice central = BLE.central();

  if (central) 
  {
    Serial.println("Connected to central device");
    Serial.print("Device MAC address: ");
    Serial.println(central.address());
      
    while(central.connected()) 
    {
      read_rotation_vector();
      read_acceleration();
      update_acc();
      update_gyr();
    }

  }


      /*
    read_rotation_vector();
    read_acceleration();
    
    if(acc_rot>=2 && acc_acc>=2)
    {
      Rotation_deg(Roll,Pitch,Yaw);
      Serial.printf("X: %.2f m/s^2, Y: %.2f m/s^2, Z: %.2f m/s^2", aceletation_matrix[0], aceletation_matrix[1], aceletation_matrix[2]);
      Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", Roll, Pitch, Yaw);
    }

   
   delay(10);
  */

}
void update_acc()
{

  AccXChar.writeValue(String(aceletation_matrix[0]));
  AccYChar.writeValue(String(aceletation_matrix[1]));
  AccZChar.writeValue(String(aceletation_matrix[2]));
 }

void update_gyr()
{
  GyroXChar.writeValue(String(RIJK[1]));
  GyroYChar.writeValue(String(RIJK[2]));
  GyroZChar.writeValue(String(RIJK[3]));
}

void read_acceleration()
{

  if ( (IMU.getSensorEvent()== true) && IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
    aceletation_matrix[0]=IMU.getAccelX();
    aceletation_matrix[1]=IMU.getAccelY();
    aceletation_matrix[2]=IMU.getAccelZ();
    acc_acc=IMU.getAccelAccuracy();
  }
}

void read_rotation_vector()
{
  
  if((IMU.getSensorEvent()== true) && (IMU.getSensorEventID()== SENSOR_REPORTID_ROTATION_VECTOR) )
  {
    // Leer giroscopio
    RIJK[0]= IMU.getRot_R();
    RIJK[1]= IMU.getRot_I();
    RIJK[2] =IMU.getRot_J();
    RIJK[3]= IMU.getRot_K();

    acc_rot=IMU.getRot_Accuracy();
  }
}

void IMU_Init(uint32_t config, uint16_t timeBetweenReports, uint32_t activitiesToEnable )
{
   Wire.begin(8,9,400000);
    if (!IMU.begin(Wire)) {
        Serial.printf("Error: No se pudo inicializar el BNO085 \n");}
	 else{Serial.printf("BNO085 inicializado correctamente \n");}

    if (config & 0b100000000000000000) { if(IMU.enableRotationVector(timeBetweenReports)) Serial.println("Rotation Vector: true"); else Serial.println("Rotation Vector: error"); }
    if (config & 0b010000000000000000) { if(IMU.enableGeomagneticRotationVector(timeBetweenReports)) Serial.println("Geomagnetic Rotation Vector: true"); else Serial.println("Geomagnetic Rotation Vector: error"); }
    if (config & 0b001000000000000000) { if(IMU.enableGameRotationVector(timeBetweenReports)) Serial.println("Game Rotation Vector: true"); else Serial.println("Game Rotation Vector: error"); }
    if (config & 0b000100000000000000) { if(IMU.enableARVRStabilizedRotationVector(timeBetweenReports)) Serial.println("AR/VR Stabilized Rotation Vector: true"); else Serial.println("AR/VR Stabilized Rotation Vector: error"); }
    if (config & 0b000010000000000000) { if(IMU.enableARVRStabilizedGameRotationVector(timeBetweenReports)) Serial.println("AR/VR Stabilized Game Rotation Vector: true"); else Serial.println("AR/VR Stabilized Game Rotation Vector: error"); }
    if (config & 0b000001000000000000) { if(IMU.enableAccelerometer(timeBetweenReports)) Serial.println("Accelerometer: true"); else Serial.println("Accelerometer: error"); }
    if (config & 0b000000100000000000) { if(IMU.enableLinearAccelerometer(timeBetweenReports)) Serial.println("Linear Accelerometer: true"); else Serial.println("Linear Accelerometer: error"); }
    if (config & 0b000000010000000000) { if(IMU.enableGravity(timeBetweenReports)) Serial.println("Gravity: true"); else Serial.println("Gravity: error"); }
    if (config & 0b000000001000000000) { if(IMU.enableGyro(timeBetweenReports)) Serial.println("Gyro: true"); else Serial.println("Gyro: error"); }
    if (config & 0b000000000100000000) { if(IMU.enableUncalibratedGyro(timeBetweenReports)) Serial.println("Uncalibrated Gyro: true"); else Serial.println("Uncalibrated Gyro: error"); }
    if (config & 0b000000000010000000) { if(IMU.enableMagnetometer(timeBetweenReports)) Serial.println("Magnetometer: true"); else Serial.println("Magnetometer: error"); }
    if (config & 0b000000000001000000) { if(IMU.enableTapDetector(timeBetweenReports)) Serial.println("Tap Detector: true"); else Serial.println("Tap Detector: error"); }
    if (config & 0b000000000000100000) { if(IMU.enableStepCounter(timeBetweenReports)) Serial.println("Step Counter: true"); else Serial.println("Step Counter: error"); }
    if (config & 0b000000000000010000) { if(IMU.enableStabilityClassifier(timeBetweenReports)) Serial.println("Stability Classifier: true"); else Serial.println("Stability Classifier: error"); }
    if (config & 0b000000000000001000) { if(IMU.enableActivityClassifier(timeBetweenReports, activitiesToEnable)) Serial.println("Activity Classifier: true"); else Serial.println("Activity Classifier: error"); }
    if (config & 0b000000000000000100) { if(IMU.enableRawAccelerometer(timeBetweenReports)) Serial.println("Raw Accelerometer: true"); else Serial.println("Raw Accelerometer: error"); }
    if (config & 0b000000000000000010) { if(IMU.enableRawGyro(timeBetweenReports)) Serial.println("Raw Gyro: true"); else Serial.println("Raw Gyro: error"); }
    if (config & 0b000000000000000001) { if(IMU.enableRawMagnetometer(timeBetweenReports)) Serial.println("Raw Magnetometer: true"); else Serial.println("Raw Magnetometer: error"); }
 
}


bool Magnitude_not_relevant(float &x, float &y, float &z)
{
  float temp= sqrt(x*x + y*y + z*z);
  if(temp < 0.1){return false;}
  else{return true;}
}

void Rotation_deg(float &roll, float &pitch, float &yaw)
{
    roll = atan2(2.0 * (RIJK[0] * RIJK[1] + RIJK[2] * RIJK[3]), 1.0 - 2.0 * (RIJK[1] * RIJK[1] + RIJK[2] * RIJK[2]));
    roll = roll * RAD_TO_DEG;

    pitch = atan2(2.0 * (RIJK[0] * RIJK[2] - RIJK[3] * RIJK[1]), 1.0 - 2.0 * (RIJK[2] * RIJK[2] + RIJK[1] * RIJK[1]));
    pitch = pitch * RAD_TO_DEG;

    yaw = atan2(2.0 * (RIJK[0] * RIJK[3] + RIJK[1] * RIJK[2]), 1.0 - 2.0 * (RIJK[2] * RIJK[2] + RIJK[3] * RIJK[3]));
    yaw = yaw * RAD_TO_DEG;

}

void Show_IMU_data_UART()
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


