#include <Dps310.h>
#include <M5Core2.h>
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
const float grav_accel = 9.80665;

bool enable_temperature = true;
bool enable_pressure = true;
bool enable_imu = true;

float dsp310_temperature = 0.0F;
float dsp310_pressure = 0.0F;
float imu_accX = 0.0F;
float imu_accY = 0.0F;
float imu_accZ = 0.0F;
float imu_gyroX = 0.0F;
float imu_gyroY = 0.0F;
float imu_gyroZ = 0.0F;

void init_ros()
{
  while (not nh.connected())
  {
    nh.spinOnce();
  }

  if (!nh.getParam("~frame_id", (char**)&frame_id))
  {
    snprintf(frame_id, 128, "sample_elevator_status");
  }
  msg_imu.header.frame_id = frame_id;

  nh.getParam("~enable_temperature", &enable_temperature);
  nh.getParam("~enable_pressure", &enable_pressure);
  nh.getParam("~enable_imu", &enable_imu);
}

void setup()
{
  M5.begin(true, false, true, true);
  Dps310PressureSensor.begin(Wire);
  M5.IMU.Init();

  nh.initNode();

  nh.advertise(pub_temperature);
  nh.advertise(pub_pressure);
  nh.advertise(pub_imu);

  init_ros();

  delay(1000);
  nh.loginfo("Initialized.");
}

void loop()
{
  if (not nh.connected())
  {
    init_ros();
  }

  if (enable_temperature)
  {
    if (Dps310PressureSensor.measureTempOnce(dsp310_temperature, 7) != DPS__SUCCEEDED)
    {
      nh.logerror("Dps310 temperature measurement failed.");
    }
    else
    {
      msg_temperature.data = dsp310_temperature;
      pub_temperature.publish(&msg_temperature);
    }
  }

  if (enable_pressure)
  {
    if (Dps310PressureSensor.measurePressureOnce(dsp310_pressure, 7) != DPS__SUCCEEDED)
    {
      nh.logerror("Dps310 pressure measurement failed.");
    }
    else
    {
      msg_pressure.data = dsp310_pressure;
      pub_pressure.publish(&msg_pressure);
    }
  }

  if (enable_imu)
  {
    M5.IMU.getGyroData(&imu_gyroX, &imu_gyroY, &imu_gyroZ);
    M5.IMU.getAccelData(&imu_accX, &imu_accY, &imu_accZ);
    msg_imu.header.stamp = nh.now();
    msg_imu.linear_acceleration.x = grav_accel * imu_accX;  // m/secsec
    msg_imu.linear_acceleration.y = grav_accel * imu_accY;  // m/secsec
    msg_imu.linear_acceleration.z = grav_accel * imu_accZ;  // m/secsec
    msg_imu.angular_velocity.x = imu_gyroX;                 // degrees / sec
    msg_imu.angular_velocity.y = imu_gyroY;                 // degrees / sec
    msg_imu.angular_velocity.z = imu_gyroZ;                 // degrees / sec
    pub_imu.publish(&msg_imu);
  }

  nh.spinOnce();
}
