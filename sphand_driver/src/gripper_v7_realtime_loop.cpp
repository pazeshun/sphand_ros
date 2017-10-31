#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "mraa.hpp"

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class PressureSensorDriver
{
private:
  mraa::Spi spi_;
  uint16_t dig_T1_;
  int16_t dig_T2_;
  int16_t dig_T3_;
  uint16_t dig_P1_;
  int16_t dig_P2_;
  int16_t dig_P3_;
  int16_t dig_P4_;
  int16_t dig_P5_;
  int16_t dig_P6_;
  int16_t dig_P7_;
  int16_t dig_P8_;
  int16_t dig_P9_;

public:
  PressureSensorDriver(const int spi_bus = 2, const int spi_cs = 0, const uint32_t max_speed = 8000000)
    : spi_(spi_bus, spi_cs)
  {
    spi_.frequency(max_speed);
  }

  ~PressureSensorDriver()
  {
  }

  void initBME()
  {
    uint8_t tx[2];
    tx[0] = 0xF5 & 0x7F;
    tx[1] = 0x20;
    spi_.write(tx, 2);
    tx[0] = 0xF4 & 0x7F;
    tx[1] = 0x27;
    spi_.write(tx, 2);
  }

  void readTrim()
  {
    uint8_t tx[25] = {};
    uint8_t rx[25];
    tx[0] = 0x88 | 0x80;
    spi_.transfer(tx, rx, 25);

    dig_T1_ = (rx[2] << 8) | rx[1];
    dig_T2_ = (rx[4] << 8) | rx[3];
    dig_T3_ = (rx[6] << 8) | rx[5];
    dig_P1_ = (rx[8] << 8) | rx[7];
    dig_P2_ = (rx[10] << 8) | rx[9];
    dig_P3_ = (rx[12] << 8) | rx[11];
    dig_P4_ = (rx[14] << 8) | rx[13];
    dig_P5_ = (rx[16] << 8) | rx[15];
    dig_P6_ = (rx[18] << 8) | rx[17];
    dig_P7_ = (rx[20] << 8) | rx[19];
    dig_P8_ = (rx[22] << 8) | rx[21];
    dig_P9_ = (rx[24] << 8) | rx[23];
  }

  bool init()
  {
    initBME();
    readTrim();

    return true;
  }

  void readRawPressureAndTemperature(uint32_t* pres_raw, uint32_t* temp_raw)
  {
    uint8_t data[8];
    uint8_t tx[9] = {};
    uint8_t rx[9];
    tx[0] = 0xF7 | 0x80;
    spi_.transfer(tx, rx, 9);
    *pres_raw = rx[1];
    *pres_raw = ((*pres_raw) << 8) | rx[2];
    *pres_raw = ((*pres_raw) << 4) | (rx[3] >> 4);
    *temp_raw = rx[4];
    *temp_raw = ((*temp_raw) << 8) | rx[5];
    *temp_raw = ((*temp_raw) << 4) | (rx[6] >> 4);
  }

  int32_t calibTemperature(int32_t temp_raw)
  {
    int32_t var1, var2, T;
    var1 = ((((temp_raw >> 3) - ((int32_t)dig_T1_ << 1))) * ((int32_t)dig_T2_)) >> 11;
    var2 =
        (((((temp_raw >> 4) - ((int32_t)dig_T1_)) * ((temp_raw >> 4) - ((int32_t)dig_T1_))) >> 12) * ((int32_t)dig_T3_)) >>
        14;

    return (var1 + var2);
  }

  uint32_t calibPressure(int32_t pres_raw, int32_t t_fine)
  {
    int32_t var1, var2;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6_);
    var2 = var2 + ((var1 * ((int32_t)dig_P5_)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4_) << 16);
    var1 = (((dig_P3_ * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2_) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1_)) >> 15);
    if (var1 == 0)
    {
      return 0;
    }
    uint32_t P = (((uint32_t)(((int32_t)1048576) - pres_raw) - (var2 >> 12))) * 3125;
    if (P < 0x80000000)
    {
      P = (P << 1) / ((uint32_t)var1);
    }
    else
    {
      P = (P / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9_) * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(P >> 2)) * ((int32_t)dig_P8_)) >> 13;
    P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7_) >> 4));
    return P;
  }

  double getPressure()
  {
    uint32_t pres_raw, temp_raw;
    readRawPressureAndTemperature(&pres_raw, &temp_raw);
    return ((double)calibPressure(pres_raw, calibTemperature(temp_raw)) / 100.0);
  }
};  // end class PressureSensorDriver

class GripperRealtimeLoop : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;

  PressureSensorDriver pres_sen_;

  // For multi-threaded spinning
  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;

public:
  GripperRealtimeLoop()
  {
    pres_sen_.init();

    // Start spinning
    nh_.setCallbackQueue(&subscriber_queue_);
    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
  }

  void cleanup()
  {
    subscriber_spinner_->stop();
  }

  void read()
  {
    ROS_INFO("%lf", pres_sen_.getPressure());
  }

  void write()
  {
  }
};  // end class GripperRealtimeLoop

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_v7_realtime_loop_node");

  GripperRealtimeLoop gripper;
  controller_manager::ControllerManager cm(&gripper);

  // For non-realtime spinner thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Control loop
  ros::Rate rate(10);
  ros::Time prev_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const ros::Duration elapsed_time = now - prev_time;

    gripper.read();
    cm.update(now, elapsed_time);
    gripper.write();
    prev_time = now;

    rate.sleep();
  }
  spinner.stop();
  gripper.cleanup();

  return 0;
}
