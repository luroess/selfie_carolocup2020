#include <selfie_scheduler/search_action_client.h>

SearchClient::SearchClient(std::string name):
    ac_(name, true)
{
    next_action_ = PARK;
    result_flag_ = false;
    action_state_ = SELFIE_IDLE;
}
SearchClient::~SearchClient()
{
}
void SearchClient::setGoal(boost::any goal)
{
    float parking_spot;

    try
    {
        parking_spot = boost::any_cast<float>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s", e.what());
        return;
    }
    goal_.min_spot_lenght = parking_spot;
    ac_.sendGoal(goal_, boost::bind(&SearchClient::doneCb, this, _1, _2),
                boost::bind(&SearchClient::activeCb, this),
                boost::bind(&SearchClient::feedbackCb, this, _1));
}
bool SearchClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool SearchClient::waitForServer(float timeout)
{
    result_flag_ = false;
    ROS_INFO("Wait for search action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void SearchClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::searchResultConstPtr& result)
{
    ROS_INFO("Finished search in state [%s]", state.toString().c_str());

    if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
        ROS_INFO("SEARCH ABORTED!!");
        result_flag_ = 2;
    }
    else
    {
        result_ = result->parking_spot;
        result_flag_ = 1;
    }
}
void SearchClient::activeCb()
{
    ROS_INFO("Search action server active");
}
void SearchClient::feedbackCb(const selfie_msgs::searchFeedbackConstPtr& feedback)
{
  ROS_INFO("Search action feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}
void SearchClient::cancelAction()
{
  ac_.cancelAllGoals();
}
program_state SearchClient::getActionState()
{
    if(action_state_ != SELFIE_IDLE)
        return action_state_;
}
int SearchClient::isActionFinished()
{
    return result_flag_;
}
void SearchClient::getActionResult(boost::any &goal)
{
    goal = result_;
}
action SearchClient::getNextAction()
{
    return next_action_;
}