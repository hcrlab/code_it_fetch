#ifndef _CODE_IT_FETCH_ROBOT_API_H_
#define _CODE_IT_FETCH_ROBOT_API_H_

#include <map>
#include <string>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "blinky/FaceAction.h"
#include "code_it_msgs/AskMultipleChoiceAction.h"
#include "code_it_msgs/DisplayMessageAction.h"
#include "code_it_msgs/GetLocationAction.h"
#include "code_it_msgs/GetPositionAction.h"
#include "code_it_msgs/GoToAction.h"
#include "code_it_msgs/MoveHeadAction.h"
#include "code_it_msgs/RunPbdActionAction.h"
#include "code_it_msgs/Say.h"
#include "code_it_msgs/SetGripperAction.h"
#include "code_it_msgs/SetTorsoAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "geometry_msgs/Pose.h"
#include "location_server/GetPoseByName.h"
#include "map_annotator/GetPoseAction.h"
#include "map_annotator/GoToLocationAction.h"
#include "map_annotator/PoseNames.h"
#include "rapid_fetch/fetch.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
using std::map;
using std::string;
using std::vector;

namespace code_it_fetch {
namespace errors {
static const char BLINKY_NOT_AVAILABLE[] =
    "Failed to connect to a Blinky face.";
static const char GO_TO[] = "Failed to navigate to specified location.";
static const char CLOSE_GRIPPER[] = "Failed to close gripper.";
static const char OPEN_GRIPPER[] = "Failed to open gripper.";
}  // namespace errors

vector<string> pose_names;
geometry_msgs::Pose curr_pose;
map<string, float> positions;

class RobotApi {
 public:
  // Does not take ownership of the Fetch pointer.
  RobotApi(
      rapid::fetch::Fetch* robot,
      actionlib::SimpleActionClient<blinky::FaceAction>* blinky_client,
      actionlib::SimpleActionClient<map_annotator::GoToLocationAction>*
          nav_client,
      actionlib::SimpleActionClient<map_annotator::GetPoseAction>* pose_client,
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
          head_client,
      actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>*
          pbd_client,
      actionlib::SimpleActionClient<control_msgs::GripperCommandAction>*
          gripper_client,
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
          torso_client);
  void AskMC(const code_it_msgs::AskMultipleChoiceGoalConstPtr& goal);
  void DisplayMessage(const code_it_msgs::DisplayMessageGoalConstPtr& goal);
  void GoTo(const code_it_msgs::GoToGoalConstPtr& goal);
  void MoveHead(const code_it_msgs::MoveHeadGoalConstPtr& goal);
  void RunPbdProgram(const code_it_msgs::RunPbdActionGoalConstPtr& goal);
  bool Say(code_it_msgs::SayRequest& req, code_it_msgs::SayResponse& res);
  void SetGripper(const code_it_msgs::SetGripperGoalConstPtr& goal);
  void SetTorso(const code_it_msgs::SetTorsoGoalConstPtr& goal);
  void HandleProgramStopped(const std_msgs::Bool& msg);
  void GetPosition(const code_it_msgs::GetPositionGoalConstPtr& goal);
  bool CompareLocation(const geometry_msgs::Pose& pose1,
                       const geometry_msgs::Pose& pose2);
  void GetLocation(const code_it_msgs::GetLocationGoalConstPtr& goal);
  float GetCurrentPos(const string joint_name);

 private:
  rapid::fetch::Fetch* const robot_;
  actionlib::SimpleActionClient<blinky::FaceAction>* blinky_client_;
  actionlib::SimpleActionClient<map_annotator::GoToLocationAction>* nav_client_;
  actionlib::SimpleActionClient<map_annotator::GetPoseAction>* pose_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
      head_client_;
  actionlib::SimpleActionClient<rapid_pbd_msgs::ExecuteProgramAction>*
      pbd_client_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>*
      gripper_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*
      torso_client_;
  actionlib::SimpleActionServer<code_it_msgs::AskMultipleChoiceAction>
      ask_mc_server_;
  actionlib::SimpleActionServer<code_it_msgs::DisplayMessageAction>
      display_message_server_;
  actionlib::SimpleActionServer<code_it_msgs::GetLocationAction>
      get_loc_server_;
  actionlib::SimpleActionServer<code_it_msgs::GetPositionAction>
      get_pos_server_;
  actionlib::SimpleActionServer<code_it_msgs::GoToAction> go_to_server_;
  actionlib::SimpleActionServer<code_it_msgs::MoveHeadAction> move_head_server_;
  actionlib::SimpleActionServer<code_it_msgs::RunPbdActionAction>
      rapid_pbd_server_;
  actionlib::SimpleActionServer<code_it_msgs::SetGripperAction>
      set_gripper_server_;
  actionlib::SimpleActionServer<code_it_msgs::SetTorsoAction> set_torso_server_;
};
}  // namespace code_it_fetch
#endif  // _CODE_IT_FETCH_ROBOT_API_H_
