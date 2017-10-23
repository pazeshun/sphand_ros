#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

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
