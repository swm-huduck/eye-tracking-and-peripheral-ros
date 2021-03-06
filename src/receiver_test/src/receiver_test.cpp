#include <ros/ros.h>
#include "peripheral/speed.h"
#include "peripheral/navigation_turn_event.h"

#include "peripheral/setting.h"
#include "peripheral/call.h"
#include "peripheral/sms.h"
#include "peripheral/kakao_talk.h"

#include "eye_tracking/eye_position.h"

void SpeedCallback(const peripheral::speed& data) {
    ROS_INFO("[SpeedCallback] value: %f, time: %s",  data.value, data.time.c_str());
}

void NavigationTurnEventCallback(const peripheral::navigation_turn_event& data) {
    ROS_INFO("[NavigationTurnEventCallback] nextTurnType: %d, nextLeftDistanceM: %f, nextRelationalPosition: (%f, %f), nextNextTurnType: %d, nextNextLeftDistanceM: %f",
        data.next_turn_type, data.next_left_distance, data.next_relational_position_x, data.next_relational_position_y, data.next_next_turn_type, data.next_next_left_distance
    );
}

void SettingCallback(const peripheral::setting& data) {
    ROS_INFO("[SettingCallback] item: %s, value: %s", data.item.c_str(), data.value.c_str());
}

void CallCallback(const peripheral::call& data) {
    ROS_INFO("[CallCallback] name: %s, callType: %d", data.name.c_str(), data.call_type);
}

void SmsCallback(const peripheral::sms& data) {
    ROS_INFO("[SmsCallback] name: %s, content: %s", data.name.c_str(), data.content.c_str());
}

void KakaoTalkCallback(const peripheral::kakao_talk& data) {
    ROS_INFO("[KakaoTalkCallback] name: %s, content: %s", data.name.c_str(), data.content.c_str());
}

void EyePositionCallback(const eye_tracking::eye_position& data) {
    // ROS_INFO("[EyePositionCallback] position: (%d, %d)", data.x, data.y);
}

int main(int argc, char **argv) {
    setlocale( LC_ALL, "" );

    ros::init(argc, argv, "receiver_test");  // node

    ros::NodeHandle nh;

    // ros::Subscriber subscriber = nh.subscribe("data", 1, Callback);

    ros::Subscriber subscriber1 = nh.subscribe("speed", 10, SpeedCallback);
    ros::Subscriber subscriber2 = nh.subscribe("navigation_turn_event", 10, NavigationTurnEventCallback);
    
    ros::Subscriber subscriber3 = nh.subscribe("setting", 10, SettingCallback);
    
    ros::Subscriber subscriber4 = nh.subscribe("call", 10, CallCallback);
    ros::Subscriber subscriber5 = nh.subscribe("sms", 10, SmsCallback);
    ros::Subscriber subscriber6 = nh.subscribe("kakao_talk", 10, KakaoTalkCallback);

    ros::Subscriber subscriber7 = nh.subscribe("eye_position", 10, EyePositionCallback);

    ros::spin();
}