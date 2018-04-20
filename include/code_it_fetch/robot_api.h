#ifndef _CODE_IT_FETCH_ROBOT_API_H_
#define _CODE_IT_FETCH_ROBOT_API_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "code_it_msgs/AskMultipleChoice.h"
#include "code_it_msgs/DisplayMessage.h"
#include "code_it_msgs/GoTo.h"
#include "code_it_msgs/MoveHead.h"
#include "code_it_msgs/RunPbdAction.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripper.h"
#include "code_it_msgs/SetTorsoAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "map_annotator/GoToLocationAction.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace code_it_fetch {
namespace errors {
static const char BLINKY_NOT_AVAILABLE[] =
    "Failed to connect to a Blinky face.";
static const char GO_TO[] = "Failed to navigate to specified location.";
static const char CLOSE_GRIPPER[] = "Failed to close gripper.";
static const char OPEN_GRIPPER[] = "Failed to open gripper.";
}  // namespace errors

class RobotApi {
 public:
  // Does not take ownership of the Fetch pointer.
  RobotApi(
      rapid::fetch::Fetch* robot,
      actionlib::SimpleActionClient<blinky::FaceAction>* blinky_client,
      actionlib::SimpleActionClient<map_annotator::GoToLocationAction>*
          nav_client,
      actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>*
          pbd_client,
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
          torso_client,
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
          head_client);
  bool AskMultipleChoice(code_it_msgs::AskMultipleChoiceRequest& req,
                         code_it_msgs::AskMultipleChoiceResponse& res);
  bool DisplayMessage(code_it_msgs::DisplayMessageRequest& req,
                      code_it_msgs::DisplayMessageResponse& res);
  bool GoTo(code_it_msgs::GoToRequest& req, code_it_msgs::GoToResponse& res);
  bool MoveHead(code_it_msgs::MoveHeadRequest& req,
                code_it_msgs::MoveHeadResponse& res);
  bool RunPbdProgram(code_it_msgs::RunPbdActionRequest& req,
                     code_it_msgs::RunPbdActionResponse& res);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  bool SetGripper(code_it_msgs::SetGripperRequest& req,
                  code_it_msgs::SetGripperResponse& res);
  void HandleProgramStopped(const std_msgs::Bool& msg);

  /* commenting to signify new things we're not sure where to place */

  void SetTorso(const code_it_msgs::SetTorsoGoalConstPtr& goal);

  /* end of commenting*/

 private:
  rapid::fetch::Fetch* const robot_;
  actionlib::SimpleActionClient<blinky::FaceAction>* blinky_client_;
  actionlib::SimpleActionClient<map_annotator::GoToLocationAction>* nav_client_;
  actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>*
      pbd_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
      torso_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
      head_client_;
  actionlib::SimpleActionServer<code_it_msgs::SetTorsoAction> set_torso_server_;
};
}  // namespace code_it_fetch
#endif  // _CODE_IT_FETCH_ROBOT_API_H_
