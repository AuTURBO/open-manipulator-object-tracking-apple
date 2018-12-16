#ifndef OPEN_MANIPULATOR_EXAMPLE_H
#define OPEN_MANIPULATOR_EXAMPLE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <termios.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseArray.h>
//#include <std_msgs/String.h>

#define NUM_OF_JOINT 4

namespace open_manipulator_example
{

class OM_EXAMPLE
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;

  ros::Subscriber chain_joint_states_sub_;
  ros::Subscriber chain_kinematics_pose_sub_;
  ros::Subscriber ar_tracker_alvar_sub_;
  ros::Subscriber centroid_pose_array_sub;

//  ros::Publisher target_object_pub_;

  std::vector<double> present_joint_angle;
  std::vector<double> present_kinematic_position;
  std::vector<uint32_t> ar_id_;
  std::vector<geometry_msgs::PoseStamped> ar_pose_;



 public:
bool flag;
std::vector<double> goalPose;
  OM_EXAMPLE();
  ~OM_EXAMPLE();

  void initPublisher();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void centroidPoseArrayMsgCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  void setGoal();

//  bool setToolControl(std::vector<double> joint_angle);
  void on_btn_gripper_open_clicked(void);
  void on_btn_gripper_close_clicked(void);
  
//  void objectPublisher();

};
}

#endif //OPEN_MANIPULATOR_EXAMPLE_H
