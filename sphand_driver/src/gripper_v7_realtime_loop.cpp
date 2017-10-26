#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class SPICommunication
{
private:
  std::string file_;
  int fd_;
  uint32_t max_speed_;
  uint8_t bits_per_word_;
  uint8_t mode_;
  uint16_t delay_;

public:
  SPICommunication(const std::string& file = "", const uint32_t max_speed = 500000, const uint8_t bits_per_word = 8,
                   const uint8_t mode = 0, const uint16_t delay = 0)
    : file_(file), max_speed_(max_speed), bits_per_word_(bits_per_word), mode_(mode), delay_(delay), fd_(-1)
  {
  }

  ~SPICommunication()
  {
    end();
  }

  bool start(const std::string& file)
  {
    file_ = file;
    return start();
  }

  bool start()
  {
    if (fd_ > 0)
    {
      ROS_WARN("SPI port already opens");
      return true;
    }

    fd_ = open(file_.c_str(), O_RDWR);
    if (fd_ < 0)
    {
      ROS_ERROR("Cannot open SPI port: %s", file_.c_str());
      return false;
    }

    // max speed hz
    int ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot set max speed hz: %s", file_.c_str());
      return false;
    }

    // bits per word
    ret = ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot set bits per word: %s", file_.c_str());
      return false;
    }

    // SPI mode
    ret = ioctl(fd_, SPI_IOC_WR_MODE, &mode_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot set SPI mode: %s", file_.c_str());
      return false;
    }

    return true;
  }

  bool getSPIInfo(std::string* file, uint32_t* max_speed, uint8_t* bits_per_word, uint8_t* mode, uint16_t* delay)
  {
    *file = file_;
    *delay = delay_;

    // max speed hz
    int ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot get max speed hz: %s", file_.c_str());
      return false;
    }
    *max_speed = max_speed_;

    // bits per word
    ret = ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot get bits per word: %s", file_.c_str());
      return false;
    }
    *bits_per_word = bits_per_word_;

    // SPI mode
    ret = ioctl(fd_, SPI_IOC_RD_MODE, &mode_);
    if (ret == -1)
    {
      ROS_ERROR("Cannot get SPI mode: %s", file_.c_str());
      return false;
    }
    *mode = mode_;

    return true;
  }

  uint8_t transfer(const uint8_t data)
  {
    uint8_t tx[1] = {0};
    uint8_t rx[1] = {0};
    struct spi_ioc_transfer tr[1];

    tx[0] = data;
    tr[0].tx_buf = (unsigned long)tx;
    tr[0].rx_buf = (unsigned long)rx;
    tr[0].len = 1;
    tr[0].delay_usecs = delay_;
    tr[0].speed_hz = max_speed_;
    tr[0].bits_per_word = bits_per_word_;

    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
    {
      ROS_ERROR("Cannot send SPI message: %s", file_.c_str());
      return 0;
    }

    return rx[0];
  }

  void end()
  {
    if (fd_ > 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }
};  // end class SPICommunication

class PressureSensorDriver
{
private:
  SPICommunication spi_;
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
  PressureSensorDriver(const std::string& file = "/dev/spidev2.0", const uint32_t max_speed = 8000000)
    : spi_(file, max_speed, 8)
  {
  }

  ~PressureSensorDriver()
  {
  }

  void initBME()
  {
    spi_.transfer((0xF5 & 0x7F));
    spi_.transfer(0x20);
    spi_.transfer((0xF4 & 0x7F));
    spi_.transfer(0x27);
  }

  void readTrim()
  {
    uint8_t data[24];
    spi_.transfer((0x88 | 0x80));
    for (int i = 0; i < 24; i++)
    {
      data[i] = spi_.transfer(0);
    }

    dig_T1_ = (data[1] << 8) | data[0];
    dig_T2_ = (data[3] << 8) | data[2];
    dig_T3_ = (data[5] << 8) | data[4];
    dig_P1_ = (data[7] << 8) | data[6];
    dig_P2_ = (data[9] << 8) | data[8];
    dig_P3_ = (data[11] << 8) | data[10];
    dig_P4_ = (data[13] << 8) | data[12];
    dig_P5_ = (data[15] << 8) | data[14];
    dig_P6_ = (data[17] << 8) | data[16];
    dig_P7_ = (data[19] << 8) | data[18];
    dig_P8_ = (data[21] << 8) | data[20];
    dig_P9_ = (data[23] << 8) | data[22];
  }

  bool init()
  {
    if (!spi_.start())
    {
      return false;
    }
    initBME();
    readTrim();

    return true;
  }

  void readRawPressureAndTemperature(uint32_t* pres_raw, uint32_t* temp_raw)
  {
    uint8_t data[8];
    spi_.transfer((0xF7 | 0x80));
    for (int i = 0; i < 8; i++)
    {
      data[i] = spi_.transfer(0x00);
    }
    *pres_raw = data[0];
    *pres_raw = ((*pres_raw) << 8) | data[1];
    *pres_raw = ((*pres_raw) << 4) | (data[2] >> 4);
    *temp_raw = data[3];
    *temp_raw = ((*temp_raw) << 8) | data[4];
    *temp_raw = ((*temp_raw) << 4) | (data[5] >> 4);
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
