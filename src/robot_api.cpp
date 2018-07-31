#include "code_it_fetch/robot_api.h"

#include <algorithm>
#include <cmath>
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
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using std::abs;
using std::string;

typedef actionlib::SimpleActionClient<blinky::FaceAction> BlinkyClient;
typedef actionlib::SimpleActionClient<map_annotator::GoToLocationAction>
    NavClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    HeadClient;
typedef actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>
    PbdClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
    GripperClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TorsoClient;

namespace code_it_fetch {
RobotApi::RobotApi(rapid::fetch::Fetch *robot, ros::ServiceClient *loc_client,
	       	   BlinkyClient *blinky_client, NavClient *nav_client,
		   HeadClient *head_client, PbdClient *pbd_client,
		   GripperClient *gripper_client, TorsoClient *torso_client)
    : robot_(robot),
      loc_client_(loc_client),
      blinky_client_(blinky_client),
      nav_client_(nav_client),
      head_client_(head_client),
      pbd_client_(pbd_client),
      gripper_client_(gripper_client),
      torso_client_(torso_client),
      ask_mc_server_("/code_it/api/ask_multiple_choice",
                     boost::bind(&RobotApi::AskMC, this, _1), false),
      display_message_server_("/code_it/api/display_message",
                              boost::bind(&RobotApi::DisplayMessage, this, _1),
                              false),
      get_loc_server_("/code_it/api/get_location",
                      boost::bind(&RobotApi::GetLocation, this, _1), false),
      get_pos_server_("/code_it/api/get_position",
                      boost::bind(&RobotApi::GetPosition, this, _1), false),
      go_to_server_("/code_it/api/go_to",
                    boost::bind(&RobotApi::GoTo, this, _1), false),
      move_head_server_("/code_it/api/move_head",
                        boost::bind(&RobotApi::MoveHead, this, _1), false),
      rapid_pbd_server_("/code_it/api/run_pbd_action",
                        boost::bind(&RobotApi::RunPbdProgram, this, _1), false),
      set_gripper_server_("/code_it/api/set_gripper",
                          boost::bind(&RobotApi::SetGripper, this, _1), false),
      set_torso_server_("/code_it/api/set_torso",
                        boost::bind(&RobotApi::SetTorso, this, _1), false) {
  ask_mc_server_.start();
  display_message_server_.start();
  get_loc_server_.start();
  get_pos_server_.start();
  go_to_server_.start();
  move_head_server_.start();
  rapid_pbd_server_.start();
  set_gripper_server_.start();
  set_torso_server_.start();
}

void RobotApi::AskMC(const code_it_msgs::AskMultipleChoiceGoalConstPtr &goal) {
  blinky::FaceGoal face_goal;
  face_goal.display_type = blinky::FaceGoal::ASK_CHOICE;
  face_goal.question = goal->question;
  face_goal.choices = goal->choices;

  blinky_client_->sendGoal(face_goal);
  while (!blinky_client_->getState().isDone()) {
    if (ask_mc_server_.isPreemptRequested() || !ros::ok()) {
      blinky_client_->cancelAllGoals();
      ask_mc_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (blinky_client_->getState() ==
      actionlib::SimpleClientGoalState::PREEMPTED) {
    blinky_client_->cancelAllGoals();
    ask_mc_server_.setPreempted();
    return;
  } else if (blinky_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    blinky_client_->cancelAllGoals();
    ask_mc_server_.setAborted();
    return;
  }

  blinky::FaceResult::ConstPtr face_result = blinky_client_->getResult();
  code_it_msgs::AskMultipleChoiceResult result;
  result.choice = face_result->choice;
  ask_mc_server_.setSucceeded(result);
}

void RobotApi::DisplayMessage(
    const code_it_msgs::DisplayMessageGoalConstPtr &goal) {
  blinky::FaceGoal face_goal;
  face_goal.display_type = blinky::FaceGoal::DISPLAY_MESSAGE;
  face_goal.h1_text = goal->h1_text;
  face_goal.h2_text = goal->h2_text;

  blinky_client_->sendGoal(face_goal);
  while (!blinky_client_->getState().isDone()) {
    if (display_message_server_.isPreemptRequested() || !ros::ok()) {
      blinky_client_->cancelAllGoals();
      display_message_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (blinky_client_->getState() ==
      actionlib::SimpleClientGoalState::PREEMPTED) {
    blinky_client_->cancelAllGoals();
    display_message_server_.setPreempted();
    return;
  } else if (blinky_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    blinky_client_->cancelAllGoals();
    display_message_server_.setAborted();
    return;
  }

  blinky::FaceResult::ConstPtr face_result = blinky_client_->getResult();
  code_it_msgs::DisplayMessageResult result;
  display_message_server_.setSucceeded(result);
}

void RobotApi::GetLocation(const code_it_msgs::GetLocationGoalConstPtr &goal) { 
  location_server::GetPoseByName srv;
  geometry_msgs::Pose poses[1024] = {};
  for (unsigned int i = 0; i < 1024; i++) {
    srv.request.name = pose_names[i];
    loc_client_->call(srv);
    poses[i] = srv.response.pose_stamped.pose;
  }
  
  code_it_msgs::GetLocationResult::ConstPtr result;

  for (unsigned int i = 0; i < 1024; i++) {
    geometry_msgs::Pose nextpose = poses[i];
    if (nextpose.position.x == curr_pose.position.x &&
        nextpose.position.y == curr_pose.position.y && 
        nextpose.position.z == curr_pose.position.z &&
	nextpose.orientation.x == curr_pose.orientation.x &&
	nextpose.orientation.y == curr_pose.orientation.y &&
	nextpose.orientation.z == curr_pose.orientation.z &&
	nextpose.orientation.w == curr_pose.orientation.w) {
      result.name = pose_names[i];
      get_loc_server_.setSucceeded(result);
      return;
    }
  }
  result.name = "Not a named location";
  get_loc_server_.setSucceeded(result);
}  


void RobotApi::GetPosition(const code_it_msgs::GetPositionGoalConstPtr &goal) {
  string resource = goal->name;
  code_it_msgs::GetPositionResult result;
  if (resource.compare("TORSO") == 0) {
    result.position = floor(GetCurrentPos("torso_lift_joint") * 1000) / 1000;
  } else if (resource.compare("HEADPAN") == 0) {
    float pan = GetCurrentPos("head_pan_joint");	  
    result.position = floor(((pan * 180.0) / M_PI) * 1000) / 1000; 
  } else if (resource.compare("HEADTILT") == 0) {
    float tilt = GetCurrentPos("head_tilt_joint");	  
    result.position = floor(((tilt * 180.0) / M_PI) * 1000) / 1000; 
  } else if (resource.compare("GRIPPER") == 0) {
    float gap = GetCurrentPos("l_gripper_finger_joint") +
                GetCurrentPos("r_gripper_finger_joint");
    result.position = floor(gap * 1000) / 1000;
  }
  get_pos_server_.setSucceeded(result);
}

void RobotApi::GoTo(const code_it_msgs::GoToGoalConstPtr &goal) {
  map_annotator::GoToLocationGoal location_goal;
  location_goal.name = goal->location;
  nav_client_->sendGoal(location_goal);
  while (!nav_client_->getState().isDone()) {
    if (go_to_server_.isPreemptRequested() || !ros::ok()) {
      nav_client_->cancelAllGoals();
    }
    ros::spinOnce();
  }
  map_annotator::GoToLocationResult::ConstPtr location_result =
      nav_client_->getResult();
  code_it_msgs::GoToResult result;
  result.error = location_result->error;
  if (nav_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED ||
      go_to_server_.isPreemptRequested()) {
    nav_client_->cancelAllGoals();
    go_to_server_.setPreempted(result);
    return;
  } else if (nav_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    nav_client_->cancelAllGoals();
    go_to_server_.setAborted(result);
    return;
  }
  go_to_server_.setSucceeded(result);
}

void RobotApi::MoveHead(const code_it_msgs::MoveHeadGoalConstPtr &goal) {
  float pan = goal->pan_degrees;
  float tilt = goal->tilt_degrees;
  pan = fmin(fmax(pan, -90.0), 90.0);
  float diffPan = abs(pan - GetCurrentPos("head_pan_joint"));
  tilt = fmin(fmax(tilt, -90.0), 45.0);
  float diffTilt = abs(tilt - GetCurrentPos("head_tilt_joint"));
  float distance = sqrt((diffPan * diffPan) + (diffTilt * diffTilt));
  float TIME_FROM_START = (5.0 / 225) * distance;

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
    }
    ros::spinOnce();
  }
  rapid_pbd_msgs::ExecuteProgramResult::ConstPtr rapid_pbd_result =
      pbd_client_->getResult();
  code_it_msgs::RunPbdActionResult result;
  result.error = rapid_pbd_result->error;
  if (pbd_client_->getState() == actionlib::SimpleClientGoalState::PREEMPTED ||
      rapid_pbd_server_.isPreemptRequested()) {
    pbd_client_->cancelAllGoals();
    rapid_pbd_server_.setPreempted(result);
    return;
  } else if (pbd_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    pbd_client_->cancelAllGoals();
    rapid_pbd_server_.setAborted(result);
    return;
  }
  rapid_pbd_server_.setSucceeded(result);
}

bool RobotApi::Say(code_it_msgs::SayRequest &req,
                   code_it_msgs::SayResponse &res) {
  robot_->Say(req.text);
  return true;
}

void RobotApi::SetGripper(const code_it_msgs::SetGripperGoalConstPtr &goal) {
  float CLOSED_POS = 0.0;
  float OPENED_POS = 0.10;
  int MIN_EFFORT = 35;
  int MAX_EFFORT = 100;

  int action = goal->action;

  control_msgs::GripperCommandGoal gripper_goal;
  if (action == 1) {
    gripper_goal.command.position = OPENED_POS;
  } else if (action == 2) {
    int max_effort = goal->max_effort;
    max_effort = fmin(max_effort, MAX_EFFORT);
    max_effort = fmax(max_effort, MIN_EFFORT);

    gripper_goal.command.position = CLOSED_POS;
    gripper_goal.command.max_effort = max_effort;
  }
  gripper_client_->sendGoal(gripper_goal);
  while (!gripper_client_->getState().isDone()) {
    if (set_gripper_server_.isPreemptRequested() || !ros::ok()) {
      gripper_client_->cancelAllGoals();
      set_gripper_server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (gripper_client_->getState() ==
      actionlib::SimpleClientGoalState::PREEMPTED) {
    gripper_client_->cancelAllGoals();
    set_gripper_server_.setPreempted();
    return;
  } else if (gripper_client_->getState() ==
             actionlib::SimpleClientGoalState::ABORTED) {
    gripper_client_->cancelAllGoals();
    set_gripper_server_.setAborted();
    return;
  }

  control_msgs::GripperCommandResult::ConstPtr gripper_result =
      gripper_client_->getResult();
  code_it_msgs::SetGripperResult result;
  set_gripper_server_.setSucceeded(result);
}

void RobotApi::SetTorso(const code_it_msgs::SetTorsoGoalConstPtr &goal) {
  float height = goal->height;
  height = fmin(height, 0.4);
  height = fmax(height, 0.0);
  float diffHeight = abs(height - GetCurrentPos("torso_lift_joint"));
  int TIME_FROM_START = (5 / 0.4) * diffHeight;

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

void RobotApi::HandleProgramStopped(const std_msgs::Bool &msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  blinky::FaceGoal goal;
  goal.display_type = blinky::FaceGoal::DEFAULT;
  blinky_client_->sendGoal(goal);
}

float RobotApi::GetCurrentPos(const string joint_name) {
  float pos = 0;
  for (unsigned int i = 0; i < 30; i++) {  // initialized to 30 in .h file
    if (joint_name.compare(joint_states_names[i]) == 0) {
      pos = joint_states_pos[i];
    }
  }
  return pos;
}

}  // namespace code_it_fetch
