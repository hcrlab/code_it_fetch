#include "code_it_fetch/robot_api.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <vector>

#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "rapid_fetch/fetch.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

using std::string;

namespace code_it_fetch {
RobotApi::RobotApi(rapid::fetch::Fetch* robot) : robot_(robot) {}

bool RobotApi::AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                                 code_it_msgs::AskMultipleChoiceResponse& res) {
  string choice;
  bool success =
      robot_->display->AskMultipleChoice(req.question, req.choices, &choice);
  res.choice = choice;
  if (!success) {
    res.error = errors::ASK_MC_QUESTION;
  }
  return true;
}

bool RobotApi::DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                              code_it_msgs::DisplayMessageResponse& res) {
  bool success = robot_->display->ShowMessage(req.h1_text, req.h2_text);
  if (!success) {
    res.error = errors::DISPLAY_MESSAGE;
  }
  return true;
}

bool RobotApi::Say(code_it_msgs::SayRequest& req,
                   code_it_msgs::SayResponse& res) {
  robot_->Say(req.text);
  return true;
}

bool RobotApi::SetGripper(code_it_msgs::SetGripperRequest& req,
                          code_it_msgs::SetGripperResponse& res) {
  if (req.action == code_it_msgs::SetGripperRequest::OPEN) {
    robot_->gripper->Open(req.max_effort);
  } else if (req.action == code_it_msgs::SetGripperRequest::CLOSE) {
    robot_->gripper->Close(req.max_effort);
  }
  return true;
}

bool RobotApi::SetTorso(code_it_msgs::SetTorsoRequest& req, code_it_msgs::SetTorsoResponse& res){
    float height = req.height;
    float maxHeight = 0.4;
    float minHeight = 0.0;
    int TIME_FROM_START = 5; //Time in seconds
    string JOINT_NAME = "torso_lift_joint";

    height = (height < minHeight) ? minHeight: height; //Makes sure height is within bounds
    height = (height > maxHeight) ? maxHeight: height;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("torso_controller/follow_joint_trajectory", true);
    bool serverConnected = client.waitForServer(ros::Duration(5));
    if(serverConnected) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(height);
        point.time_from_start = ros::Duration(TIME_FROM_START, 0);
        control_msgs::FollowJointTrajectoryGoal goal;
        trajectory_msgs::JointTrajectory trajectory;
        trajectory.joint_names.push_back(JOINT_NAME);
        trajectory.points.push_back(point);
        goal.trajectory = trajectory;
        client.sendGoal(goal);
        std::cout << "Waiting for result" << std::endl;
        bool success = client.waitForResult();
        std::cout << "Result received" << std::endl;
        return success;
    }
    else{
        std::cerr << "The torso server did not connect" << std::endl;
        return  false;
    }

}

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  robot_->display->ShowDefault();
}
}  // namespace code_it_fetch
