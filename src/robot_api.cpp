#include "code_it_fetch/robot_api.h"

#include <algorithm>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
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

typedef actionlib::SimpleActionClient<blinky::FaceAction> BlinkyClient;
typedef actionlib::SimpleActionClient<map_annotator::GoToLocationAction>
    NavClient;
typedef actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>
    PbdClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TorsoClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HeadClient;

namespace code_it_fetch {
RobotApi::RobotApi(rapid::fetch::Fetch *robot, BlinkyClient *blinky_client,
                   NavClient *nav_client, PbdClient *pbd_client,
                   TorsoClient *torso_client, HeadClient *head_client)
    : robot_(robot),
      blinky_client_(blinky_client),
      nav_client_(nav_client),
      pbd_client_(pbd_client),
      torso_client_(torso_client),
      head_client_(head_client),
      set_torso_server_("/code_it/api/set_torso",
                        boost::bind(&RobotApi::SetTorso, this, _1), false) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest &req,
                                 code_it_msgs::AskMultipleChoiceResponse &res) {
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::ASK_CHOICE;
  goal.question = req.question;
  goal.choices = req.choices;
  if (!blinky_client_->waitForServer(ros::Duration(5.0))) {
    res.error = errors::BLINKY_NOT_AVAILABLE;
    return true;
  }
  blinky_client_->sendGoal(goal);
  blinky_client_->waitForResult(ros::Duration(0));
  blinky::FaceResultConstPtr result = blinky_client_->getResult();
  res.choice = result->choice;
  return true;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest &req,
                              code_it_msgs::DisplayMessageResponse &res) {
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::DISPLAY_MESSAGE;
  goal.h1_text = req.h1_text;
  goal.h2_text = req.h2_text;
  blinky_client_->sendGoal(goal);
  if (!blinky_client_->waitForResult(ros::Duration(5.0))) {
    res.error = errors::BLINKY_NOT_AVAILABLE;
    return true;
  }

  return true;
}

bool RobotApi::MoveHead(code_it_msgs::MoveHeadRequest &req,
                        code_it_msgs::MoveHeadResponse &res) {
  float pan = std::min(std::max(req.pan_degrees, -90.0), 90.0);
  float tilt = std::min(std::max(req.tilt_degrees, -90.0), 45.0);

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(pan * M_PI / 180.0);
  point.positions.push_back(tilt * M_PI / 180.0);
  point.time_from_start = ros::Duration(2.5);
  control_msgs::FollowJointTrajectoryGoal goal;

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names.push_back("head_pan_joint");
  trajectory.joint_names.push_back("head_tilt_joint");
  trajectory.points.push_back(point);
  goal.trajectory = trajectory;
  head_client_->sendGoal(goal);

  bool success = head_client_->waitForResult(ros::Duration(10));
  if (!success) {
    res.error = "Head action did not finish within 10 seconds.";
  }
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

void RobotApi::SetTorso(const code_it_msgs::SetTorsoGoalConstPtr &goal) {
  float height = goal->height;
  height = fmin(height, 0.4);
  height = fmax(height, 0.0);
  int TIME_FROM_START = 5;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(height);
  point.time_from_start = ros::Duration(TIME_FROM_START);
  control_msgs::FollowJointTrajectoryGoal torso_goal;
  torso_goal.trajectory.joint_names.push_back("torso_lift_joint");
  torso_goal.trajectory.points.push_back(point);

  torso_client_->sendGoal(torso_goal);
  while (!torso_client_->getState().isDone()) {
    if (set_torso_server_.isPreemptRequest() || !ros::ok()) {
      torso_client_->cancelAllGoals();
      set_torso_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (torso_client_->getState() ==
      actionlib::SimpleClientGoalState::PREEMPTED) {
    torso_client_->cancelAllGoals();
    set_torso_server_.setPreempted();
    return;
  } else if (torso_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    torso_client_->cancelAllGoals();
    set_torso_server_.setAborted();
    return;
  }

  control_msgs::FollowJointTrajectoryResult::ConstPtr torso_result =
      torso_client_->getResult();
  code_it_msgs::SetTorsoResult result;
  result.error = torso_result->error_string;
  set_torso_server_.setSucceeded(result);
}

void RobotApi::HandleProgramStopped(const std_msgs::Bool &msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::DEFAULT;
  blinky_client_->sendGoal(goal);
}
}  // namespace code_it_fetch
