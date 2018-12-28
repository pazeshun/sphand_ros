#include "vl53l0x_mraa_ros/vl53l0x_mraa.h"
#include <ros/ros.h>

Vl53l0xMraa lox = Vl53l0xMraa();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "single_vl53l0x_ros");
  ros::NodeHandle nh;

  ROS_INFO("VL53L0X test");
  mraa::I2c i2c(0);
  if (!lox.begin(&i2c))
  {
    ROS_FATAL("Failed to boot VL53L0X");
    return 0;
  }
  ROS_INFO("VL53L0X API Simple Ranging example");
  ros::Rate rate(10);
  while (ros::ok())
  {
    VL53L0X_RangingMeasurementData_t measure;
    lox.getSingleRangingMeasurement(&measure, false);  // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4)
    {
      // phase failures have incorrect data
      ROS_INFO("Distance (mm): %d", measure.RangeMilliMeter);
    }
    else
    {
      ROS_INFO("Out of range");
    }
    rate.sleep();
  }

  return 0;
}
