#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "code_it_fetch/robot_api.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"

using code_it_fetch::RobotApi;

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_fetch");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
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

  actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>
      pbd_client("/rapid_pbd/execute_program_action", true);
  if (!pbd_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Rapid PbD server not available!");
  }

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      torso_client("torso_controller/follow_joint_trajectory", true);
  while (!torso_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for torso server...");
  }

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client("gripper_controller/gripper_action", true);
  while (!gripper_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for gripper server...");
  }

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      head_client("head_controller/follow_joint_trajectory", true);
  while (!head_client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for head server...");
  }

  RobotApi api(robot, &blinky_client, &nav_client, &pbd_client, &torso_client,
               &gripper_client, &head_client);

  ros::ServiceServer ask_mc_srv = nh.advertiseService(
      "code_it/api/ask_multiple_choice", &RobotApi::AskMultipleChoice, &api);
  ros::ServiceServer disp_msg_srv = nh.advertiseService(
      "code_it/api/display_message", &RobotApi::DisplayMessage, &api);
  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  ROS_INFO("CodeIt! for the Fetch is ready.");
  ros::waitForShutdown();
  delete robot;
  return 0;
}
