#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(std::string name) :
    pnh_("~"), as_(nh_, name, boost::bind(&StartingProcedureAction::executeCB, this, _1), false),
  action_name_(name),starting_speed_(2.0),button_status_(0),minSecondPressTime_(ros::Time(0)),debounceDuration_(ros::Duration(2)),
  distanceGoal_(0),distanceRead_(0.0)
{
    as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));
    pnh_.getParam("starting_speed",starting_speed_);

    button_sub_ = nh_.subscribe("start_button", 1000, &StartingProcedureAction::buttonCB, this);
    distance_sub_ = nh_.subscribe("distance",10,&StartingProcedureAction::distanceCB, this);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    as_.start();

    ROS_INFO("Starting procedure object created");
}
StartingProcedureAction::~StartingProcedureAction(void)
{
    pnh_.deleteParam("starting_speed");
}

void StartingProcedureAction::executeCB(const selfie_msgs::startingGoalConstPtr &goal)
{
    ROS_INFO("received goal %f",goal->distance);
    publishFeedback(SELFIE_READY);
    state_ = State::WAIT_BUTTON;
}
void StartingProcedureAction::buttonCB(const std_msgs::BoolConstPtr &msg)
{
    ROS_INFO("Button pressed %d",msg->data);
    if(state_ == State::WAIT_BUTTON)
    {
        minSecondPressTime_ = ros::Time::now() + debounceDuration_;
        if(msg->data == false)
            button_status_ = BUTTON_FREE_DRIVE_PRESSED;
        else if (msg->data == true)
            button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
        state_ = State::WAIT_START;
    }
    else if(state_ == State::WAIT_START)
    {
        if(ros::Time::now() > minSecondPressTime_)
        {
            if(button_status_ == BUTTON_FREE_DRIVE_PRESSED) publishFeedback(BUTTON_FREE_DRIVE_PRESSED);
            else if(button_status_ == BUTTON_OBSTACLE_DRIVE_PRESSED) publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
            state_ = State::START_MOVE;
            distanceGoal_ = distanceRead_ + distanceGoal_;
            startingDistance_ = distanceRead_;
            publishFeedback(START_DRIVE);
        }
    }
}
void StartingProcedureAction::preemptCB()
{
    as_.setAborted();
    state_ = State::IDLE;
}
void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{  
    distanceRead_ = msg->data;
    if(state_ == State::START_MOVE )
    {
        driveBoxOut(starting_speed_);
        if(distanceRead_ > distanceGoal_)
        {
          state_ = State::IDLE;
          publishFeedback(END_DRIVE);
          as_.setSucceeded(result_);
        }
    }
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
    drive_pub_.publish(cmd);
}
