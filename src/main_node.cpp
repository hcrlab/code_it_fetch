#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "code_it_fetch/robot_api.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"

using code_it_fetch::RobotApi;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  //updating info used from rostopic /joint_states

  for (unsigned int i = 0; i < msg->name.size(); i++) {
    if (i >= msg->position.size()) {
      continue;
    }
    //joint_states_publisher will return 0 as a default value if there is no new data. we don't update our maps unless there is new (non-zero) data.  
    if (msg->position[i] != 0) {
       code_it_fetch::positions[msg->name[i]] =  msg->position[i];
    }

    if (msg->velocity[i] != 0) {
       code_it_fetch::velocities[msg->name[i]] = msg->velocity[i];
    }
  }
  
  //updating whether the gripper has slipped
  if (code_it_fetch::checkingIfSlipped) { 
    float l_gripper_pos_new = code_it_fetch::positions["l_gripper_finger_joint"];
    float l_gripper_vel_new = code_it_fetch::velocities["l_gripper_finger_joint"];

    float r_gripper_pos_new = code_it_fetch::positions["r_gripper_finger_joint"];
    float r_gripper_vel_new = code_it_fetch::velocities["r_gripper_finger_joint"];
    
    //wasGrasping is true if the velocities of the gripper fingers are between GRIPPER_VEL_TOLERANCE and -1 * GRIPPER_VEL_TOLERANCE 
    bool wasGrasping = (code_it_fetch::l_gripper_vel_old <= code_it_fetch::GRIPPER_VEL_TOLERANCE) && (code_it_fetch::l_gripper_vel_old >= -1 * code_it_fetch::GRIPPER_VEL_TOLERANCE);
       // && (code_it_fetch::r_gripper_vel_old <= code_it_fetch::GRIPPER_VEL_TOLERANCE) && (code_it_fetch::r_gripper_vel_old >= -1 * code_it_fetch::GRIPPER_VEL_TOLERANCE);
  
    //true if the gripper was open in the last callback
    bool wasOpen = code_it_fetch::l_gripper_pos_old >= 0.05; 
     
    //true if gripper is closed in current callback
    bool isClosed = l_gripper_pos_new <= 0.001;

    //closing is positive velocity
    bool nowClosing = (l_gripper_vel_new >= (code_it_fetch::GRIPPER_VEL_TOLERANCE)); // && (r_gripper_vel_new < (-1 * code_it_fetch::GRIPPER_VEL_TOLERANCE));
     
     if (wasGrasping && nowClosing && !wasOpen && !isClosed) {
     	code_it_fetch::gripperSlipped = true; //this value will be true until the gripper sent another goal (to open or close)
     }
     //update old velocity and position values
     code_it_fetch::l_gripper_vel_old = l_gripper_vel_new;
     code_it_fetch::r_gripper_vel_old = r_gripper_vel_new;
     
     code_it_fetch::l_gripper_pos_old = l_gripper_pos_new;
     code_it_fetch::r_gripper_pos_old = r_gripper_pos_new;
     //put in a check for position so that when it closes, we dont get true.
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_fetch");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointCallback);
  spinner.start();

  rapid::fetch::Fetch* robot = rapid::fetch::BuildReal();

  actionlib::SimpleActionClient<blinky::FaceAction> blinky_client("blinky",
                                                                  true);
  if (!blinky_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Blinky server not available!");
  }

  actionlib::SimpleActionClient<map_annotator::GoToLocationAction> nav_client(
      "/map_annotator/go_to_location", true);
  if (!nav_client.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Map annotator server not available!");
  }

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      head_client("head_controller/follow_joint_trajectory", true);
  while (ros::ok() && !head_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for head server...");
  }

  actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>
      pbd_client("/rapid_pbd/execute_program_action", true);
  if (!pbd_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Rapid PbD server not available!");
  }

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client("gripper_controller/gripper_action", true);
  while (ros::ok() && !gripper_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for gripper server...");
  }

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      torso_client("torso_controller/follow_joint_trajectory", true);
  while (ros::ok() && !torso_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for torso server...");
  }


  RobotApi api(robot, &blinky_client, &nav_client, &head_client, &pbd_client,
               &gripper_client, &torso_client);

  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  ROS_INFO("CodeIt! for the Fetch is ready.");
  ros::waitForShutdown();
  delete robot;
  return 0;
}

