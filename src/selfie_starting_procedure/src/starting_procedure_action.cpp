/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(std::string name) :
    as_(nh_, name, boost::bind(&StartingProcedureAction::executeCB, this, _1), false), action_name_(name)
{
    as_.start();
    button_sub_ = nh_.subscribe("start_button", 1000, &StartingProcedureAction::buttonCB, this);
    distance_sub_ = nh_.subscribe("distance", 10, &StartingProcedureAction::distanceCB, this);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);

    distance_base_ = 0.0;

    ROS_INFO("Starting procedure object created");
}
StartingProcedureAction::~StartingProcedureAction(void)
{
}

void StartingProcedureAction::executeCB(const selfie_msgs::startingGoalConstPtr &goal)
{
    ROS_INFO("received goal %f", goal->distance);
    publishFeedback(SELFIE_READY);

    while (1)
    {
        if (button_status_ == BUTTON_FREE_DRIVE_PRESSED)
        {
            publishFeedback(BUTTON_FREE_DRIVE_PRESSED);
            result_.drive_mode = true;
            break;
        }
        else if (button_status_ == BUTTON_OBSTACLE_DRIVE_PRESSED)
        {
            publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
            result_.drive_mode = false;
            break;
        }
    }

    // check if car started to move
    while (covered_distance_ < 0.1)
    {
        driveBoxOut(1.5);
    }

    ROS_INFO("CAR START MOVE");
    publishFeedback(START_DRIVE);

    while (covered_distance_ < goal->distance)
    {
        driveBoxOut(1.5);
    }
    ROS_INFO("DISTANCE COVERED");
    driveBoxOut(0.0);
    publishFeedback(END_DRIVE);

    // publish result
    as_.setSucceeded(result_);
}
void StartingProcedureAction::buttonCB(const std_msgs::BoolConstPtr &msg)
{
    ROS_INFO("Button pressed %d", msg->data);
    if (msg->data == false)
        button_status_ = BUTTON_FREE_DRIVE_PRESSED;
    else if (msg->data == true)
        button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
}

void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{
    if (distance_base_ == 0.0)
    {
        distance_base_ = msg->data;
    }
    covered_distance_ = msg->data - distance_base_;
}

void StartingProcedureAction::publishFeedback(feedback_variable program_state)
{
    feedback_.action_status = program_state;
    as_.publishFeedback(feedback_);
}

void StartingProcedureAction::driveBoxOut(float speed)
{
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.drive.speed = speed;
    cmd.drive.steering_angle = 0;
    cmd.drive.steering_angle_velocity = 15;
    drive_pub_.publish(cmd);
}