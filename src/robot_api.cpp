#include "code_it_fetch/robot_api.h"

#include <algorithm>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "blinky/FaceAction.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/GoTo.h"
#include "code_it_msgs/RunPbdAction.h"
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
      go_to_server_("/code_it/api/go_to",
                    boost::bind(&RobotApi::GoTo, this, _1), false),
      move_head_server_("/code_it/api/move_head",
                        boost::bind(&RobotApi::MoveHead, this, _1), false),
      rapid_pbd_server_("/code_it/api/run_pbd_action",
                        boost::bind(&RobotApi::RunPbdProgram, this, _1), false),
      set_torso_server_("/code_it/api/set_torso",
                        boost::bind(&RobotApi::SetTorso, this, _1), false) {
  go_to_server_.start();
  move_head_server_.start();
  rapid_pbd_server_.start();
  set_torso_server_.start();
}

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

void RobotApi::MoveHead(const code_it_msgs::MoveHeadGoalConstPtr &goal) {
  float pan = goal->pan_degrees;
  float tilt = goal->tilt_degrees;
  pan = fmin(fmax(pan, -90.0), 90.0);
  tilt = fmin(fmax(tilt, -90.0), 45.0);
  int TIME_FROM_START = 5;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(pan * M_PI / 180.0);
  point.positions.push_back(tilt * M_PI / 180.0);
  point.time_from_start = ros::Duration(TIME_FROM_START);
  control_msgs::FollowJointTrajectoryGoal head_goal;
  head_goal.trajectory.joint_names.push_back("head_pan_joint");
  head_goal.trajectory.joint_names.push_back("head_tilt_joint");
  head_goal.trajectory.points.push_back(point);

  head_client_->sendGoal(head_goal);
  while (!head_client_->getState().isDone()) {
    if (move_head_server_.isPreemptRequested() || !ros::ok()) {
      head_client_->cancelAllGoals();
      move_head_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (head_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    head_client_->cancelAllGoals();
    move_head_server_.setPreempted();
    return;
  } else if (head_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    head_client_->cancelAllGoals();
    move_head_server_.setAborted();
    return;
  }

  control_msgs::FollowJointTrajectoryResult::ConstPtr head_result =
      head_client_->getResult();
  code_it_msgs::MoveHeadResult result;
  result.error = head_result->error_string;
  move_head_server_.setSucceeded(result);
}

void RobotApi::RunPbdProgram(
    const code_it_msgs::RunPbdActionGoalConstPtr &goal) {
  rapid_pbd_msgs::ExecuteProgramGoal rapid_pbd_goal;
  rapid_pbd_goal.name = goal->name;

  pbd_client_->sendGoal(rapid_pbd_goal);
  while (!pbd_client_->getState().isDone()) {
    if (rapid_pbd_server_.isPreemptRequested() || !ros::ok()) {
      pbd_client_->cancelAllGoals();
      rapid_pbd_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (pbd_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    pbd_client_->cancelAllGoals();
    rapid_pbd_server_.setPreempted();
    return;
  } else if (pbd_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    torso_client_->cancelAllGoals();
    rapid_pbd_server_.setAborted();
    return;
  }

  rapid_pbd_msgs::ExecuteProgramResult::ConstPtr rapid_pbd_result =
      pbd_client_->getResult();
  code_it_msgs::RunPbdActionResult result;
  result.error = rapid_pbd_result->error;
  rapid_pbd_server_.setSucceeded(result);
}

//  bool success = pbd_client_->waitForResult(ros::Duration(10 * 60));
//    res.error = "Rapid PbD action did not finish within 10 minutes.";
//    res.error = "Error with Rapid PbD server.";

bool RobotApi::Say(code_it_msgs::SayRequest &req,
                   code_it_msgs::SayResponse &res) {
  robot_->Say(req.text);
  return true;
}

void RobotApi::GoTo(const code_it_msgs::GoToGoalConstPtr &goal) {
  map_annotator::GoToLocationGoal location_goal;
  location_goal.name = goal->location;
  nav_client_->sendGoal(location_goal);
  while (!nav_client_->getState().isDone()) {
    if (go_to_server_.isPreemptRequested() || !ros::ok()) {
      nav_client_->cancelAllGoals();
      go_to_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (nav_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    nav_client_->cancelAllGoals();
    go_to_server_.setPreempted();
    return;
  } else if (nav_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    nav_client_->cancelAllGoals();
    go_to_server_.setAborted();
    return;
  }

  map_annotator::GoToLocationResult::ConstPtr location_result =
      nav_client_->getResult();
  code_it_msgs::GoToResult result;
  result.error = location_result->error;
  go_to_server_.setSucceeded(result);
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
    if (set_torso_server_.isPreemptRequested() || !ros::ok()) {
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

bool RobotApi::TorsoIsDone() const {
  return torso_client_->getState().isDone();
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
