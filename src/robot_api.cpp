#include "code_it_fetch/robot_api.h"

#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/GoTo.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using std::string;

typedef actionlib::SimpleActionClient<map_annotator::GoToLocationAction>
    NavClient;
typedef actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>
    PbdClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TorsoClient;

namespace code_it_fetch {
RobotApi::RobotApi(rapid::fetch::Fetch *robot, NavClient *nav_client,
                   PbdClient *pbd_client, TorsoClient *torso_client)
    : robot_(robot),
      nav_client_(nav_client),
      pbd_client_(pbd_client),
      torso_client_(torso_client) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest &req,
                                 code_it_msgs::AskMultipleChoiceResponse &res) {
  string choice;
  bool success =
      robot_->display->AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  if (!success) {
    res.error = errors::ASK_MC_QUESTION;
  }
  return true;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest &req,
                              code_it_msgs::DisplayMessageResponse &res) {
  bool success = robot_->display->ShowMessage(req.h1_text, req.h2_text);
  if (!success) {
    res.error = errors::DISPLAY_MESSAGE;
  }
  return true;
}

bool RobotApi::RunPbdProgram(code_it_msgs::RunPbdActionRequest &req,
                             code_it_msgs::RunPbdActionResponse &res) {
  rapid_pbd_msgs::ExecuteProgramGoal goal;
  goal.name = req.name;
  pbd_client_->sendGoal(goal);
  bool success = pbd_client_->waitForResult(ros::Duration(10 * 60));
  if (!success) {
    res.error = "Rapid PbD action did not finish within 10 minutes.";
    return true;
  }
  rapid_pbd_msgs::ExecuteProgramResultConstPtr result =
      pbd_client_->getResult();
  if (!result) {
    res.error = "Error with Rapid PbD server.";
    return true;
  }
  res.error = result->error;
  return true;
}

bool RobotApi::Say(code_it_msgs::SayRequest &req,
                   code_it_msgs::SayResponse &res) {
  robot_->Say(req.text);
  return true;
}

bool RobotApi::GoTo(code_it_msgs::GoToRequest &req,
                    code_it_msgs::GoToResponse &res) {
  map_annotator::GoToLocationGoal goal;
  goal.name = req.location;
  nav_client_->sendGoal(goal);
  bool success = nav_client_->waitForResult(ros::Duration(60 * 60));
  if (!success) {
    res.error = "Navigation did not finish within an hour.";
    return true;
  }
  map_annotator::GoToLocationResultConstPtr result = nav_client_->getResult();
  if (!result) {
    res.error = "Error with navigation server";
    return true;
  }
  res.error = result->error;
  return true;
}

bool RobotApi::SetGripper(code_it_msgs::SetGripperRequest &req,
                          code_it_msgs::SetGripperResponse &res) {
  if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
    robot_->gripper->Open(req.max_effort);
  } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
    robot_->gripper->Close(req.max_effort);
  }
  return true;
}

bool RobotApi::SetTorso(code_it_msgs::SetTorsoRequest &req,
                        code_it_msgs::SetTorsoResponse &res) {
  float height = req.height;
  float maxHeight = 0.4;
  float minHeight = 0.0;
  int TIME_FROM_START = 5;  // Time in seconds
  string JOINT_NAME = "torso_lift_joint";

  height = (height < minHeight) ? minHeight
                                : height;  // Makes sure height is within bounds
  height = (height > maxHeight) ? maxHeight : height;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(height);
  point.time_from_start = ros::Duration(TIME_FROM_START, 0);
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names.push_back(JOINT_NAME);
  trajectory.points.push_back(point);
  goal.trajectory = trajectory;
  torso_client_->sendGoal(goal);
  bool success = torso_client_->waitForResult(ros::Duration(10));
  if (!success) {
    res.error = "Torso action did not finish within 10 seconds.";
  }
  return true;
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool &msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  robot_->display->ShowDefault();
}
}  // namespace code_it_fetch
