#include <Dps310.h>
#include <M5Stack.h>
#include "ros/node_handle.h"
#include "ArduinoHardware.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

Dps310 Dps310PressureSensor = Dps310();
ros::NodeHandle_<ArduinoHardware> nh;

std_msgs::Float32 msg_temperature;
std_msgs::Float32 msg_pressure;
sensor_msgs::Imu msg_imu;

ros::Publisher pub_temperature("~temperature", &msg_temperature);
ros::Publisher pub_pressure("~pressure", &msg_pressure);
ros::Publisher pub_imu("~imu", &msg_imu);

char frame_id[128];

void setup()
{
  M5.begin(true, false, true, true);
  Dps310PressureSensor.begin(Wire);
  M5.IMU.Init();

  nh.initNode();

  nh.advertise(pub_temperature);
  nh.advertise(pub_pressure);
  nh.advertise(pub_imu);

  while (not nh.connected())
  {
    nh.spinOnce();
  }

  if (!nh.getParam("~frame_id", (char**)&frame_id))
  {
    snprintf(frame_id, 128, "sample_elevator_status");
  }
  msg_imu.header.frame_id = frame_id;

  delay(1000);
  nh.loginfo("Initialized.");
}

void loop()
{
  float dsp310_temperature = 0.0F;
  float dsp310_pressure = 0.0F;
  float imu_accX = 0.0F;
  float imu_accY = 0.0F;
  float imu_accZ = 0.0F;
  float imu_gyroX = 0.0F;
  float imu_gyroY = 0.0F;
  float imu_gyroZ = 0.0F;

  Dps310PressureSensor.measureTempOnce(dsp310_temperature, 7);
  Dps310PressureSensor.measurePressureOnce(dsp310_pressure, 7);
  M5.IMU.getGyroData(&imu_gyroX, &imu_gyroY, &imu_gyroZ);
  M5.IMU.getAccelData(&imu_accX, &imu_accY, &imu_accZ);

  msg_temperature.data = dsp310_temperature;
  msg_pressure.data = dsp310_pressure;
  msg_imu.header.stamp = nh.now();
  msg_imu.linear_acceleration.x = imu_accX;
  msg_imu.linear_acceleration.y = imu_accY;
  msg_imu.linear_acceleration.z = imu_accZ;
  msg_imu.angular_velocity.x = imu_gyroX;
  msg_imu.angular_velocity.y = imu_gyroY;
  msg_imu.angular_velocity.z = imu_gyroZ;

  pub_temperature.publish(&msg_temperature);
  pub_pressure.publish(&msg_pressure);
  nh.spinOnce();
  pub_imu.publish(&msg_imu);
}
