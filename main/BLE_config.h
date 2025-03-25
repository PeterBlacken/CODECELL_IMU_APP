
#include <ArduinoBLE.h>
#define  chunk_size 128
const char* deviceServiceUuid = "b83b6b32-0d38-45da-9a65-73eecc736b17";
const char* AccXCharUuid = "22128dec-f7bc-4c21-a329-c13449221777";
const char* AccYCharUuid = "677314b1-6d7d-4b38-8e37-0a3d24538c01";
const char* AccZCharUuid = "648ae827-f439-4038-a8ed-88e67b2cbb33";
const char* GyroXCharUuid = "07c120ff-3915-410b-9c45-c84533e674df";
const char* GyroYCharUuid = "17a45678-jj34-5678-1234-56789ab34ef5";
const char* GyroZCharUuid = "9f345678-dd34-5678-1234-56789a67def6";

BLEService IMUService(deviceServiceUuid);
BLEStringCharacteristic AccXChar(AccXCharUuid, BLERead | BLENotify,20);
BLEStringCharacteristic AccYChar(AccYCharUuid, BLERead | BLENotify,20);
BLEStringCharacteristic AccZChar(AccZCharUuid, BLERead | BLENotify,20);
BLEStringCharacteristic GyroXChar(GyroXCharUuid, BLERead | BLENotify,20);
BLEStringCharacteristic GyroYChar(GyroYCharUuid, BLERead | BLENotify,20);
BLEStringCharacteristic GyroZChar(GyroZCharUuid, BLERead | BLENotify,20);


