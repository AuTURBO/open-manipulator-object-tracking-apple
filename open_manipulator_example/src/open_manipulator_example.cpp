
#include "open_manipulator_example/open_manipulator_example.h"
#include <math.h>
#include <stdlib.h>

using namespace open_manipulator_example;
geometry_msgs::Pose centroid_pose[10];
int centroid_pose_size = 0;
std::string target_object;
uint8_t task = 0, pre_task = 0;
float object_x = 0;
float object_y = 0;
float object_z = 0;

enum
{
  INIT_POSITION,
  CHECK_OBJECT_POSE,
  MOVE_ARM_TOPICK,
  MOVE_ARM_TOPLACE,
  END_POSITION,
  FINISH,
};

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
  // service client
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");
  //set_actuator_state_client_ = n.serviceClient<open_manipulator_msgs::SetActuatorState>("/open_manipulator/set_actuator_state");
  //goal_drawing_trajectory_client_ = n.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("/open_manipulator/goal_drawing_trajectory");

  target_object_pub_  = node_handle_.advertise<std_msgs::String>("/targetObjectSubscriber", 10);
}
void OM_EXAMPLE::initSubscriber()
{
  //open_manipulator_states_sub_       = n.subscribe("/open_manipulator/states", 10, &QNode::statesCallback, this);
  chain_joint_states_sub_ = node_handle_.subscribe("/open_manipulator/joint_states", 10, &OM_EXAMPLE::jointStatesCallback, this);
  chain_kinematics_pose_sub_ = node_handle_.subscribe("/open_manipulator/kinematics_pose", 10, &OM_EXAMPLE::kinematicsPoseCallback, this);

  ar_tracker_alvar_sub_ = node_handle_.subscribe("/ar_pose_marker", 1, &OM_EXAMPLE::arPoseMarkerCallback, this);
  centroid_pose_array_sub = node_handle_.subscribe("/cluster_decomposer/centroid_pose_array", 10, &OM_EXAMPLE::centroidPoseArrayMsgCallback,this);
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
  kinematics_pose_.pose = msg->pose;
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

void OM_EXAMPLE::centroidPoseArrayMsgCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{  
  //ROS_INFO("SAVE POSE OF centroidPoseArray");

  if( task == CHECK_OBJECT_POSE || task == MOVE_ARM_TOPICK ) {
  //if( 1 == 1 ) {

	  /*float x_offset = 0.07;
	  float y_offset = -0.015;
	  float z_offset = 0.017;*/
	  float x_offset = 0.0;
	  float y_offset = -0.02;
	  float z_offset = -0.1;

	  if (msg->poses.size() == 0){
	    centroid_pose_size = 0;
	    return;
	  }
	    
	  for(int i = 0 ; i < msg->poses.size() ; i++){
		if( !( ( msg->poses[i].position.x == 0 ) && ( msg->poses[i].position.y == 0 ) && ( msg->poses[i].position.z == 0 ) ) ) {
			centroid_pose[i] = msg->poses[i];		

			object_x = centroid_pose[i].position.z + x_offset;
			object_y = -(centroid_pose[i].position.x + y_offset);
			object_z = -(centroid_pose[i].position.y + z_offset);

			ROS_INFO("coord %d _  %.3f, %.3f, %.3f _  %.3f, %.3f, %.3f  ",i, object_x, object_y, object_z, centroid_pose[i].position.x, centroid_pose[i].position.y, centroid_pose[i].position.z );

			centroid_pose_size = centroid_pose_size + 1;	
                        task = MOVE_ARM_TOPICK;
		}
		if( centroid_pose_size >= 10){
			break ;
		}
	  }  
  }else{
      
  }
  
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
    return srv.response.is_planned;
  }
  return false;
}

bool OM_EXAMPLE::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OM_EXAMPLE::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
   
  srv.request.end_effector_name = "gripper";

  if( std::isnan(kinematics_pose.at(0)) || std::isnan(kinematics_pose.at(1)) || std::isnan(kinematics_pose.at(2)) ) {

      std::cerr << "Warning Error: NaN value operation.";
      return false ;
    }


  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose_.pose.orientation.w;
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose_.pose.orientation.x;
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose_.pose.orientation.y;
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose_.pose.orientation.z;

  ROS_INFO("-- x( %.3lf, %.3lf) y( %.3lf, %.3lf) z( %.3lf, %.3lf) \n", kinematics_pose.at(0), srv.request.kinematics_pose.pose.position.x,
         kinematics_pose.at(1), srv.request.kinematics_pose.pose.position.y, kinematics_pose.at(2), srv.request.kinematics_pose.pose.position.z );
  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OM_EXAMPLE::setGoal()
{
 return ;

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


void OM_EXAMPLE::on_btn_gripper_open_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(-0.01);

  if(!OM_EXAMPLE::setToolControl(joint_angle))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }

  ROS_INFO("Send gripper open");
}

void OM_EXAMPLE::on_btn_gripper_close_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(0.01);
  if(!OM_EXAMPLE::setToolControl(joint_angle))
  {
    //writeLog("[ERR!!] Failed to send service");
    return;
  }

  //writeLog("Send gripper close");
}

void OM_EXAMPLE::objectPublisher(void)
{
  std_msgs::String msg;
  msg.data = target_object.c_str();
  target_object_pub_.publish(msg);
  //ROS_INFO("send objectPublisher %s ", target_object.c_str());
}

void OM_EXAMPLE::init_pose(void)
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 2.0;

  joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
  joint_name.push_back("joint3"); joint_angle.push_back(0.35);
  joint_name.push_back("joint4"); joint_angle.push_back(0.70);
  if(!setJointSpacePath(joint_name, joint_angle, path_time))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }
  ROS_INFO("Send joint angle to home pose");

	/*goalPose.at(0) = 0.138; //x
	goalPose.at(1) = 0.0; //y
	goalPose.at(2) = 0.238; //z
	setTaskSpacePath(goalPose, 2);
	on_btn_gripper_open_clicked();
	task = CHECK_OBJECT_POSE ; */
}


void OM_EXAMPLE::process(void)
{
  objectPublisher();
  switch (task)
  {

    case INIT_POSITION:
	init_pose();
        task = CHECK_OBJECT_POSE ;
	break;

    case CHECK_OBJECT_POSE:
	break;

    case MOVE_ARM_TOPICK:
        //object_x = object_x - 0.16 ;
        ROS_INFO(" move %.3f, %.3f, %.3f ", object_x, object_y, object_z );
	goalPose.at(0) = object_x ;//x
	goalPose.at(1) = object_y; //y
	goalPose.at(2) = object_z ; //z
	setTaskSpacePath(goalPose, 2);
        sleep(2.2);
	//on_btn_gripper_close_clicked();
        //sleep(1.0);
	//task = MOVE_ARM_TOPLACE ;
        //task = FINISH ;
	break;

    case MOVE_ARM_TOPLACE:
	goalPose.at(0) = 0.138; //x
	goalPose.at(1) = 0.0; //y
	goalPose.at(2) = 0.238; //z
	setTaskSpacePath(goalPose, 2);
        sleep(2.2);
	goalPose.at(0) = -0.1; //x
	goalPose.at(1) = 0.1; //y
	goalPose.at(2) = 0.200; //z
	setTaskSpacePath(goalPose, 2);
        sleep(2.2);
	on_btn_gripper_open_clicked();
        sleep(1.0);
	task = END_POSITION ;
	break;

    case END_POSITION:
	goalPose.at(0) = 0.138; //x
	goalPose.at(1) = 0.0; //y
	goalPose.at(2) = 0.238; //z
	setTaskSpacePath(goalPose, 2);
        sleep(2.5);
	task = FINISH ;
	break;

    case FINISH:
	break;

    default:
	break;
  }

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_example start");
  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("target_object", target_object);

  OM_EXAMPLE om_example_;
  om_example_.goalPose = om_example_.getPresentKinematicsPose(); 
  ros::Rate loop_rate(25);
  task = INIT_POSITION ;

  while(ros::ok()){
      ros::spinOnce();
      om_example_.process();      
      loop_rate.sleep();
      if( task == FINISH ){
	   break;
	}
  }
  ROS_INFO("open_manipulator_example finish");
  return 0;

}
