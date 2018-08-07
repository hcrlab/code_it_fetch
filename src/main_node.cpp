#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "code_it_fetch/robot_api.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"

using code_it_fetch::RobotApi;

void locationCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  code_it_fetch::curr_pose = msg->pose.pose;
}

void posesCallback(const map_annotator::PoseNames::ConstPtr& msg) {
  for (unsigned int i = 0; i < msg->names.size(); i++) {
    code_it_fetch::pose_names.push_back(msg->names[i]);
  }
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // updating info used from rostopic /joint_states

  for (unsigned int i = 0; i < msg->name.size(); i++) {
    if (i >= msg->position.size()) {
      continue;
    }
    // joint_states_publisher will return 0 as a default value if there is no
    // new data. we don't update our maps unless there is new (non-zero) data.
    if (msg->position[i] != 0) {
      code_it_fetch::positions[msg->name[i]] = msg->position[i];
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "code_it_fetch");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, jointCallback);
  ros::Subscriber location_sub =
      nh.subscribe("/amcl_pose", 10, locationCallback);
  ros::Subscriber poses_sub =
      nh.subscribe("/map_annotator/pose_names", 10, posesCallback);
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

  actionlib::SimpleActionClient<map_annotator::GetPoseAction> pose_client(
      "/map_annotator/get_pose", true);
  if (!pose_client.waitForServer(ros::Duration(5.0))) {
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

  RobotApi api(robot, &blinky_client, &nav_client, &pose_client, &head_client,
               &pbd_client, &gripper_client, &torso_client);

  ros::ServiceServer say_srv =
      nh.advertiseService("code_it/api/say", &RobotApi::Say, &api);
  ros::Subscriber stop_sub = nh.subscribe(
      "code_it/is_program_running", 10, &RobotApi::HandleProgramStopped, &api);
  ROS_INFO("CodeIt! for the Fetch is ready.");
  ros::waitForShutdown();
  delete robot;
  return 0;
}
