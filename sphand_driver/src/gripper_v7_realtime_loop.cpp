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

class GripperRealtimeLoop : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;

  // For multi-threaded spinning
  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;

public:
  GripperRealtimeLoop()
  {
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
  ros::Rate rate(100);
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
