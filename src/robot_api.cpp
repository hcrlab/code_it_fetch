#include "code_it_fetch/robot_api.h"

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

void RobotApi::HandleProgramStopped(const std_msgs::Bool& msg) {
  if (msg.data) {
    return;  // Program is running, nothing to do.
  }
  robot_->display->ShowDefault();
}
}  // namespace code_it_fetch
