#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "mraa.hpp"

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <force_proximity_ros/ProximityArray.h>
#include <force_proximity_ros/Proximity.h>
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

  int32_t calibTemperature(const int32_t temp_raw)
  {
    int32_t var1, var2, T;
    var1 = ((((temp_raw >> 3) - ((int32_t)dig_T1_ << 1))) * ((int32_t)dig_T2_)) >> 11;
    var2 =
        (((((temp_raw >> 4) - ((int32_t)dig_T1_)) * ((temp_raw >> 4) - ((int32_t)dig_T1_))) >> 12) * ((int32_t)dig_T3_)) >>
        14;

    return (var1 + var2);
  }

  uint32_t calibPressure(const int32_t pres_raw, const int32_t t_fine)
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

class FlexSensorDriver
{
private:
  mraa::Spi spi_;
  const int sensor_num_;

public:
  FlexSensorDriver(const int sensor_num = 2, const int spi_bus = 2, const int spi_cs = 1,
                   const uint32_t max_speed = 1000000)
    : spi_(spi_bus, spi_cs), sensor_num_(sensor_num)
  {
    spi_.frequency(max_speed);
  }

  ~FlexSensorDriver()
  {
  }

  void getFlex(std::vector<uint16_t>* flex)
  {
    flex->clear();
    uint8_t tx[3] = {};
    uint8_t rx[3];
    for (int sensor_no = 0; sensor_no < sensor_num_; sensor_no++)
    {
      tx[0] = (0x18 | sensor_no) << 3;
      spi_.transfer(tx, rx, 3);
      uint16_t value = (rx[0] & 0x01) << 11;
      value |= rx[1] << 3;
      value |= rx[2] >> 5;
      flex->push_back(value);
    }
  }
};  // end class FlexSensorDriver

class ProximitySensorDriver
{
private:
  // Constants
  enum
  {
    // 7-bit unshifted I2C address of Multiplexer (PCA9547D)
    MUX_ADDR = 0x70,
    // 7-bit unshifted I2C address of VCNL4040
    VCNL4040_ADDR = 0x60,
    // Command Registers
    PS_CONF1 = 0x03,
    PS_CONF3 = 0x04,
    PS_DATA_L = 0x08,
  };
  const int sensor_num_;
  // Sensitivity of touch/release detection
  const int sensitivity_;
  // exponential average weight parameter / cut-off frequency for high-pass filter
  const double ea_;
  // low-pass filtered proximity reading
  std::vector<double> average_value_;
  // FA-II value
  std::vector<double> fa2_;
  mraa::I2c i2c_;

public:
  ProximitySensorDriver(const int sensor_num = 1, const int i2c_bus = 0)
    : sensor_num_(sensor_num), sensitivity_(1000), ea_(0.3), fa2_(sensor_num, 0), i2c_(i2c_bus)
  {
  }

  ~ProximitySensorDriver()
  {
  }

  mraa::Result setMultiplexerCh(const uint8_t mux_addr, const int8_t ch)
  {
    uint8_t tx;

    if (ch == -1)
    {
      // No channel selected
      tx &= 0xF7;
    }
    else if (0 <= ch && ch <= 7)
    {
      // Channel 0~7 selected
      tx = (uint8_t)ch | 0x08;
    }
    else
    {
      ROS_ERROR("I2C Multiplexer has no channel %d", ch);
      return mraa::ERROR_UNSPECIFIED;
    }

    i2c_.address(mux_addr);
    return i2c_.writeByte(tx);
  }

  // Read from two Command Registers of VCNL4040
  uint16_t readCommandRegVCNL4040(const uint8_t command_code)
  {
    i2c_.address(VCNL4040_ADDR);
    uint16_t rx = i2c_.readWordReg(command_code);
    return rx;
  }

  // Write to two Command Registers of VCNL4040
  mraa::Result writeCommandRegVCNL4040(const uint8_t command_code, const uint8_t low_data, const uint8_t high_data)
  {
    uint16_t data = ((uint16_t)high_data << 8) | low_data;

    i2c_.address(VCNL4040_ADDR);
    return i2c_.writeWordReg(command_code, data);
  }

  // Configure VCNL4040
  void initVCNL4040()
  {
    // Set PS_CONF3 and PS_MS
    uint8_t conf3 = 0x00;
    // uint8_t ms = 0x00;  // IR LED current to 50mA
    uint8_t ms = 0x01;  // IR LED current to 75mA
    // uint8_t ms = 0x02;  // IR LED current to 100mA
    // uint8_t ms = 0x06;  // IR LED current to 180mA
    // uint8_t ms = 0x07;  // IR LED current to 200mA
    writeCommandRegVCNL4040(PS_CONF3, conf3, ms);
  }

  void startProxSensor()
  {
    // Clear PS_SD to turn on proximity sensing
    // uint8_t conf1 = 0x00;  // Clear PS_SD bit to begin reading
    uint8_t conf1 = 0x0E;  // Integrate 8T, Clear PS_SD bit to begin reading
    // uint8_t conf2 = 0x00;  // Clear PS to 12-bit
    uint8_t conf2 = 0x08;  // Set PS to 16-bit
    writeCommandRegVCNL4040(PS_CONF1, conf1, conf2);
  }

  void stopProxSensor()
  {
    // Set PS_SD to turn off proximity sensing
    uint8_t conf1 = 0x01;  // Set PS_SD bit to stop reading
    uint8_t conf2 = 0x00;
    writeCommandRegVCNL4040(PS_CONF1, conf1, conf2);
  }

  bool init()
  {
    for (int sensor_no = 0; sensor_no < sensor_num_; sensor_no++)
    {
      setMultiplexerCh(MUX_ADDR, sensor_no);
      initVCNL4040();
      startProxSensor();
    }

    return true;
  }

  void getRawProximities(std::vector<uint16_t>* raw_proximities)
  {
    raw_proximities->clear();
    for (int sensor_no = 0; sensor_no < sensor_num_; sensor_no++)
    {
      setMultiplexerCh(MUX_ADDR, sensor_no);
      raw_proximities->push_back(readCommandRegVCNL4040(PS_DATA_L));
    }
  }

  void getProximityArray(force_proximity_ros::ProximityArray* proximity_array)
  {
    std::vector<uint16_t> raw_proximities;
    getRawProximities(&raw_proximities);
    // Record time of reading sensor
    proximity_array->header.stamp = ros::Time::now();

    // Fill necessary data of proximity
    proximity_array->proximities.clear();
    force_proximity_ros::Proximity proximity;
    for (int sensor_no = 0; sensor_no < sensor_num_; sensor_no++)
    {
      uint16_t raw = raw_proximities[sensor_no];
      proximity.proximity = raw;
      if (average_value_.size() == sensor_no)
      {
        // Init average value
        average_value_.push_back(proximity.proximity);
      }
      proximity.average = average_value_[sensor_no];
      proximity.fa2derivative = average_value_[sensor_no] - raw - fa2_[sensor_no];
      fa2_[sensor_no] = average_value_[sensor_no] - raw;
      proximity.fa2 = fa2_[sensor_no];
      if (fa2_[sensor_no] < -sensitivity_)
      {
          proximity.mode = "T";
      }
      else if (fa2_[sensor_no] > sensitivity_)
      {
          proximity.mode = "R";
      }
      else
      {
          proximity.mode = "0";
      }
      average_value_[sensor_no] = ea_ * raw + (1 - ea_) * average_value_[sensor_no];
      proximity_array->proximities.push_back(proximity);
    }
  }
};  // end class ProximitySensorDriver

class GripperLoop : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;

  PressureSensorDriver pres_sen_;

  FlexSensorDriver flex_sen_;
  std::vector<std::string> flex_names_;

  ProximitySensorDriver prox_sen_;

  // ROS publishers
  ros::Publisher pressure_pub_;
  std::map<std::string, ros::Publisher> flex_pub_;
  ros::Publisher prox_pub_;

  // For multi-threaded spinning
  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;

public:
  GripperLoop(const std::vector<std::string>& flex_names)
    : flex_names_(flex_names), flex_sen_(flex_names.size())
  {
    pres_sen_.init();

    prox_sen_.init();

    // Publisher for pressure
    pressure_pub_ = nh_.advertise<std_msgs::Float64>("pressure", 1);

    // Publisher for flex
    for (int i = 0; i < flex_names_.size(); i++)
    {
      flex_pub_[flex_names_[i]] = nh_.advertise<std_msgs::UInt16>("flex/" + flex_names_[i], 1);
    }

    // Publisher for proximity
    prox_pub_ = nh_.advertise<force_proximity_ros::ProximityArray>("proximity_array", 1);

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
    // Get and publish pressure
    std_msgs::Float64 pressure;
    pressure.data = pres_sen_.getPressure();
    pressure_pub_.publish(pressure);

    // Get and publish flex
    std::vector<uint16_t> flex;
    flex_sen_.getFlex(&flex);
    for (int i = 0; i < flex_names_.size(); i++)
    {
      std_msgs::UInt16 value;
      value.data = flex[i];
      flex_pub_[flex_names_[i]].publish(value);
    }

    // Get and publish proximity
    force_proximity_ros::ProximityArray proximity_array;
    prox_sen_.getProximityArray(&proximity_array);
    prox_pub_.publish(proximity_array);
  }

  void write()
  {
  }
};  // end class GripperLoop

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_v7_loop_node");

  std::vector<std::string> flex_names;

  if (!(ros::param::get("~flex_names", flex_names)))
  {
    ROS_ERROR("Couldn't get necessary parameters");
    return 0;
  }

  GripperLoop gripper(flex_names);
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
