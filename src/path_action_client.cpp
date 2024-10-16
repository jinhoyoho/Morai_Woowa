#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <morai_woowa/Planning_Tracking_ActAction.h> // 실제 패키지명으로 변경

// 콜백 함수: 서버가 완료되었을 때 호출됩니다.
void doneCb(const actionlib::SimpleClientGoalState& state,
            const morai_woowa::Planning_Tracking_ActResultConstPtr& result)
{
  ROS_INFO("Action finished: %s", state.toString().c_str());
  // 결과 처리
  ROS_INFO("Result: %d", result->success);  // result_value는 예시로, 실제 정의에 맞게 수정
}

// 콜백 함수: 목표가 활성화되었을 때 호출됩니다.
void activeCb()
{
  ROS_INFO("Goal is now active");
}

// 콜백 함수: 피드백을 받을 때마다 호출됩니다.
void feedbackCb(const morai_woowa::Planning_Tracking_ActFeedbackConstPtr& feedback)
{
  ROS_INFO("Received feedback: %lf", feedback->progress_percentage);  // 예시로 피드백 값 출력
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_tracking_action_client");

  // 액션 클라이언트 선언 (액션 이름은 서버와 동일해야 함)
  actionlib::SimpleActionClient<morai_woowa::Planning_Tracking_ActAction> ac("planning_tracking", true);

  ROS_INFO("Waiting for action server to start...");
  ac.waitForServer();  // 서버가 시작될 때까지 대기

  ROS_INFO("Action server started, sending goal.");

  // 목표(goal) 생성
  morai_woowa::Planning_Tracking_ActGoal goal;
  goal.path = "indoor_0_3.csv";  // 목표 위치 설정 (예시, 실제 목표에 맞게 수정)

  // 액션 클라이언트에 목표를 보내고, 콜백 함수 등록
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  // 목표가 완료될 때까지 기다림
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    ROS_INFO("Action finished successfully.");
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}
