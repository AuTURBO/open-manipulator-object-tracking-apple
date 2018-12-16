
#include "open_manipulator_example/open_manipulator_example.h"
#include <math.h>
#include <stdlib.h>

using namespace open_manipulator_example;

OM_EXAMPLE::OM_EXAMPLE()
    :node_handle_("")
{
  present_joint_angle.resize(NUM_OF_JOINT);
  present_kinematic_position.resize(3);

  initPublisher();
  initSubscriber();

  ROS_INFO("OpenManipulator example initialization");
  flag = false;

}

OM_EXAMPLE::~OM_EXAMPLE()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void OM_EXAMPLE::initPublisher()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");

}
void OM_EXAMPLE::initSubscriber()
{
  chain_joint_states_sub_ = node_handle_.subscribe("/open_manipulator/joint_states", 10, &OM_EXAMPLE::jointStatesCallback, this);
  chain_kinematics_pose_sub_ = node_handle_.subscribe("/open_manipulator/kinematics_pose", 10, &OM_EXAMPLE::kinematicsPoseCallback, this);
  ar_tracker_alvar_sub_ = node_handle_.subscribe("/ar_pose_marker", 1, &OM_EXAMPLE::arPoseMarkerCallback, this);

}

void OM_EXAMPLE::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;
}

void OM_EXAMPLE::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position = temp_position;
}

void OM_EXAMPLE::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  std::vector<uint32_t> ar_id;
  ar_id.resize(msg->markers.size());
  std::vector<geometry_msgs::PoseStamped> ar_pose;
  ar_pose.resize(msg->markers.size());

  for(int i = 0; i < msg->markers.size(); i ++)
  {
    ar_id.at(i) = msg->markers.at(i).id;
    ar_pose.at(i) = msg->markers.at(i).pose;
    if(ar_id.at(i) == 0)
    {
      //ROS_INFO("-- %.3lf, %.3lf, %.3lf\n", ar_pose.at(i).pose.position.x, ar_pose.at(i).pose.position.x, ar_pose.at(i).pose.position.x);
      flag = true;
    }

  }
  ar_id_ = ar_id;
  ar_pose_ = ar_pose;

  setGoal();
}

std::vector<double> OM_EXAMPLE::getPresentJointAngle()
{
  return present_joint_angle;
}
std::vector<double> OM_EXAMPLE::getPresentKinematicsPose()
{
  return present_kinematic_position;
}

bool OM_EXAMPLE::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool OM_EXAMPLE::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}

bool OM_EXAMPLE::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  //if((std::isnan(kinematics_pose.at(0))||(std::isnan(kinematics_pose.at(0))||(std::isnan(kinematics_pose.at(0)){
  if( std::isnan(kinematics_pose.at(0)) || std::isnan(kinematics_pose.at(1)) || std::isnan(kinematics_pose.at(2)) ) {

      std::cerr << "Warning Error: NaN value operation.";
      return false ;
    }


  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);//1);// 0.05 ~ 0.25 //
  if( srv.request.kinematics_pose.pose.position.x > 0.22){
      srv.request.kinematics_pose.pose.position.x  = 0.22 ;
 }else if( srv.request.kinematics_pose.pose.position.x < -0.00){
     srv.request.kinematics_pose.pose.position.x  = -0.00 ;
}
srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);//2);// -0.2 ~ 0.2 //

if( srv.request.kinematics_pose.pose.position.y > 0.25){
    srv.request.kinematics_pose.pose.position.y  = 0.25 ;
}else if( srv.request.kinematics_pose.pose.position.y < -0.25){
     srv.request.kinematics_pose.pose.position.y  = -0.25 ;
}

srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);//0);
if ( srv.request.kinematics_pose.pose.position.z  < 0.07 ){
  srv.request.kinematics_pose.pose.position.z  = 0.07 ;
}
else if ( srv.request.kinematics_pose.pose.position.z > 0.25 ){
  srv.request.kinematics_pose.pose.position.z  = 0.25 ;
}
ROS_INFO("-- x( %.3lf, %.3lf) y( %.3lf, %.3lf) z( %.3lf, %.3lf) \n", kinematics_pose.at(0), srv.request.kinematics_pose.pose.position.x,
         kinematics_pose.at(1), srv.request.kinematics_pose.pose.position.y, kinematics_pose.at(2), srv.request.kinematics_pose.pose.position.z );
srv.request.path_time = path_time;

if(goal_task_space_path_client_.call(srv))
{
 return srv.response.isPlanned;
}
return false;
}

void OM_EXAMPLE::setGoal()
{

 for(int i = 0; i < ar_id_.size(); i ++)
 {
   if(ar_id_.at(i) == 0)
   {

      //goalPose.at(0) += (ar_pose_.at(i).pose.position.z * 1.0 - 0.270);
      //goalPose.at(1) -= (ar_pose_.at(i).pose.position.x * 1.0);
      //goalPose.at(2) -= (ar_pose_.at(i).pose.position.y * 1.0);
      //goalPose.at(0) = (ar_pose_.at(i).pose.position.z * 1.0 - 0.270);
      
	goalPose.at(0) = (ar_pose_.at(i).pose.position.z * 1.0 -0.470);
      goalPose.at(1) = -(ar_pose_.at(i).pose.position.x * 1.0);
      goalPose.at(2) = -(ar_pose_.at(i).pose.position.y * 1.0-0.15);
      //goalPose.at(1) = (ar_pose_.at(i).pose.position.z * 1.0);
      //goalPose.at(2) = (ar_pose_.at(i).pose.position.x*-1.0);
      //goalPose.at(0) = (ar_pose_.at(i).pose.position.y)+0.4;
      //ROS_INFO("-- %.3lf, %.3lf, %.3lf\n", goalPose.at(0), goalPose.at(1), goalPose.at(2));

      return;
    }
  }
}
/*
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_example");

  OM_EXAMPLE om_example_;
  om_example_.goalPose = om_example_.getPresentKinematicsPose();

  sleep(2);

  ros::spinOnce();
  om_example_.setGoal();
  om_example_.setTaskSpacePath(om_example_.goalPose, 3.0);

  return 0;

}*/

/*bool OM_EXAMPLE::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.isPlanned;
  }
  return false;
}*/

void OM_EXAMPLE::on_btn_gripper_open_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(-1.0);

  if(!OM_EXAMPLE::setToolControl(joint_angle))
  {
    //writeLog("[ERR!!] Failed to send service");
    return;
  }

  //writeLog("Send gripper open");
}

void OM_EXAMPLE::on_btn_gripper_close_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(0.5);
  if(!OM_EXAMPLE::setToolControl(joint_angle))
  {
    //writeLog("[ERR!!] Failed to send service");
    return;
  }

  //writeLog("Send gripper close");
}


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_example");

  OM_EXAMPLE om_example_;

  om_example_.goalPose = om_example_.getPresentKinematicsPose();


  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 2.0;
  joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  joint_name.push_back("joint2"); joint_angle.push_back(0.0);
  joint_name.push_back("joint3"); joint_angle.push_back(0.0);
  joint_name.push_back("joint4"); joint_angle.push_back(0.0);
  om_example_.setJointSpacePath(joint_name, joint_angle, path_time);
  
  sleep(2);
  om_example_.on_btn_gripper_open_clicked();
  


  joint_name.clear();
  joint_angle.clear();
  joint_name.push_back("joint1"); joint_angle.push_back(-0.342);
  joint_name.push_back("joint2"); joint_angle.push_back(-0.305);
  joint_name.push_back("joint3"); joint_angle.push_back(0.779);
  joint_name.push_back("joint4"); joint_angle.push_back(-0.518);
  om_example_.setJointSpacePath(joint_name, joint_angle, path_time);
  sleep(2);

  joint_name.clear();
  joint_angle.clear();
  joint_name.push_back("joint1"); joint_angle.push_back(-0.379);
  joint_name.push_back("joint2"); joint_angle.push_back(0.301);
  joint_name.push_back("joint3"); joint_angle.push_back(0.055);
  joint_name.push_back("joint4"); joint_angle.push_back(-0.333);
  om_example_.setJointSpacePath(joint_name, joint_angle, path_time);
  
  sleep(2);
  om_example_.on_btn_gripper_close_clicked();
  sleep(2);
  
  joint_name.clear();
  joint_angle.clear();
  joint_name.push_back("joint1"); joint_angle.push_back(-0.0);
  joint_name.push_back("joint2"); joint_angle.push_back(0.0);
  joint_name.push_back("joint3"); joint_angle.push_back(0.0);
  joint_name.push_back("joint4"); joint_angle.push_back(-0.0);
  om_example_.setJointSpacePath(joint_name, joint_angle, path_time);
  sleep(2);  



  while(ros::ok()){

      ros::spinOnce();
      om_example_.setGoal();
      om_example_.setTaskSpacePath(om_example_.goalPose, 0.5);
      //sleep(0.2);

  }

  return 0;

}
