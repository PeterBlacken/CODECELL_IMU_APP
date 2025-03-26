

String imuDataToString(IMU_data_t data_t) {
  String json = "{";
  json += "\"time_stamp\":" + String(data_t.time_stamp) + ",";
  json += "\"acc_x\":" + String(data_t.acc_x, 4) + ",";
  json += "\"acc_y\":" + String(data_t.acc_y, 4) + ",";
  json += "\"acc_z\":" + String(data_t.acc_z, 4) + ",";
  json += "\"gyr_x\":" + String(data_t.gyr_x, 4) + ",";
  json += "\"gyr_y\":" + String(data_t.gyr_y, 4) + ",";
  json += "\"gyr_z\":" + String(data_t.gyr_z, 4) + ",";
  json += "\"mag_x\":" + String(data_t.mag_x, 4) + ",";
  json += "\"mag_y\":" + String(data_t.mag_y, 4) + ",";
  json += "\"mag_z\":" + String(data_t.mag_z, 4);
  json += "}";
  return json;
}
void imuDataToCSV_char(IMU_data_t *data, char *buffer, size_t bufferSize)
{
  snprintf(buffer, bufferSize,
           "%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
           data->time_stamp,
           data->acc_x, data->acc_y, data->acc_z,
           data->gyr_x, data->gyr_y, data->gyr_z,
           data->mag_x, data->mag_y, data->mag_z);
}


String imuDataToCSV(IMU_data_t *data_t) 
{
  return String(data_t->time_stamp) + "," +
         String(data_t->acc_x, 4) + "," +
         String(data_t->acc_y, 4) + "," +
         String(data_t->acc_z, 4) + "," +
         String(data_t->gyr_x, 4) + "," +
         String(data_t->gyr_y, 4) + "," +
         String(data_t->gyr_z, 4) + "," +
         String(data_t->mag_x, 4) + "," +
         String(data_t->mag_y, 4) + "," +
         String(data_t->mag_z, 4);
}

