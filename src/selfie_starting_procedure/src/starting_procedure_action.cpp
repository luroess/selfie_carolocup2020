#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),pnh_(pnh), as_(nh_, "starting_procedure", boost::bind(&StartingProcedureAction::executeCB, this, _1), false),
  minSecondPressTime_(ros::Time(0)),debounceDuration_(ros::Duration(2)),
  distanceGoal_(0.f),distanceRead_(0.f)
{
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));

  parking_button_sub_ = nh_.subscribe("start_button", 10, &StartingProcedureAction::parking_buttonCB, this);
  obstacle_button_sub_ = nh_.subscribe("start_button", 10, &StartingProcedureAction::obstacle_buttonCB, this);
  distance_sub_ = nh_.subscribe("distance",10,&StartingProcedureAction::distanceCB, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
  pnh_.param<float>("starting_speed", starting_speed_,2.f);
  pnh_.param("use_scan",use_scan_,false);
  pnh_.param("use_qr",use_qr_,true);
  ROS_INFO("starting_speed: %.3f", starting_speed_);

  as_.start();
  ROS_INFO("Starting procedure object created");
}
StartingProcedureAction::~StartingProcedureAction(void)
{ 
  obstacle_button_sub_.shutdown();
  parking_button_sub_.shutdown();
  distance_sub_.shutdown();
}

void StartingProcedureAction::executeCB(const selfie_msgs::startingGoalConstPtr &goal)
{
  ROS_INFO("received goal %f",goal->distance);
  publishFeedback(SELFIE_READY);
  state_ = State::WAIT_BUTTON;
}
void StartingProcedureAction::parking_buttonCB(const std_msgs::Empty &msg)
{
  if(state_ == State::WAIT_BUTTON)
  {
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    button_status_ = BUTTON_PARKING_DRIVE_PRESSED;
    state_ = State::WAIT_START;
  }
  else if(state_ == State::WAIT_START)
  {
    if(ros::Time::now() > minSecondPressTime_)
    {
        publishFeedback(BUTTON_PARKING_DRIVE_PRESSED);
        state_ = State::START_MOVE;
        distanceGoal_ = distanceRead_ + distanceGoal_;
        startingDistance_ = distanceRead_;
        publishFeedback(START_DRIVE);
    }
  }
}

void StartingProcedureAction::obstacle_buttonCB(const std_msgs::Empty &msg)
{
  if(state_ == State::WAIT_BUTTON)
  {
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
    state_ = State::WAIT_START;
  }
  else if(state_ == State::WAIT_START)
  {
    if(ros::Time::now() > minSecondPressTime_)
    {
        publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
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
  ROS_INFO("STARTING PROCEDURE ABORTED");
  parking_button_sub_.shutdown();
  obstacle_button_sub_.shutdown();
  distance_sub_.shutdown();
  as_.setAborted();
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
