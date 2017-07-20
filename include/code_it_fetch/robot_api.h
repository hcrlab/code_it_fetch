#ifndef _CODE_IT_FETCH_ROBOT_API_H_
#define _CODE_IT_FETCH_ROBOT_API_H_

#include <string>

#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/GoTo.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/SetTorso.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace code_it_fetch {
namespace errors {
static const char ASK_MC_QUESTION[] = "Failed to ask multiple choice question.";
static const char DISPLAY_MESSAGE[] = "Failed to display message.";
static const char GO_TO[] = "Failed to navigate to specified location.";
static const char CLOSE_GRIPPER[] = "Failed to close gripper.";
static const char OPEN_GRIPPER[] = "Failed to open gripper.";
}  // namespace errors

class RobotApi {
 public:
  // Does not take ownership of the Fetch pointer.
  RobotApi(
      rapid::fetch::Fetch* robot,
      actionlib::SimpleActionClient<map_annotator::GoToLocationAction>*
          nav_client,
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
          torso_client);
  bool AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                         code_it_msgs::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                      code_it_msgs::DisplayMessageResponse& res);
  bool GoTo(code_it_msgs::GoToRequest& req, code_it_msgs::GoToResponse& res);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  bool SetGripper(code_it_msgs::SetGripperRequest& req,
                  code_it_msgs::SetGripperResponse& res);
  bool SetTorso(code_it_msgs::SetTorsoRequest& req,
                code_it_msgs::SetTorsoResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

 private:
  rapid::fetch::Fetch* const robot_;
  actionlib::SimpleActionClient<map_annotator::GoToLocationAction>* nav_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
      torso_client_;
};
}  // namespace code_it_fetch
#endif  // _CODE_IT_FETCH_ROBOT_API_H_
