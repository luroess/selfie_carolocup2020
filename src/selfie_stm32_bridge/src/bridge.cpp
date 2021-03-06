#include <selfie_stm32_bridge/bridge.h>

Time::Time():begin(ros::Time::now())
{

}

uint32_t Time::get_ms_time()
{
    ros::Time now = ros::Time::now();
    return (now.sec - begin.sec) * 1000 + (now.nsec / 1000000);

}

Sub_messages::Sub_messages()
{
    indicator.left = 0;
    indicator.right = 0;
    ackerman.steering_angle = 0;
    ackerman.steering_angle_velocity = 0;
    ackerman.speed = 0;
    ackerman.acceleration = 0;
    ackerman.jerk = 0;
}