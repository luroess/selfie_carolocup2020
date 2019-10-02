#include <ros/ros.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/park_action_client.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/scheduler.h>
#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/PolygonArray.h>

class ActionMock
{
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer <selfie_msgs::parkAction> park_action_;
    actionlib::SimpleActionServer <selfie_msgs::drivingAction> drive_action_;
    actionlib::SimpleActionServer <selfie_msgs::startingAction> starting_action_;
    actionlib::SimpleActionServer <selfie_msgs::searchAction> search_action_;
public:
    ActionMock():
        park_action_(nh_,"park",boost::bind(&ActionMock::park_action_goalCB,this,_1),false),
        drive_action_(nh_,"free_drive",boost::bind(&ActionMock::drive_action_goalCB,this,_1),false),
        starting_action_(nh_,"starting_procedure",boost::bind(&ActionMock::starting_action_goalCB,this,_1),false),
        search_action_(nh_,"search",boost::bind(&ActionMock::search_action_goalCB,this,_1),false)
    {
        starting_action_.start();
        drive_action_.start();
        search_action_.start();
        park_action_.start();

    }
    void starting_action_goalCB(const selfie_msgs::startingGoalConstPtr &goal)
    {
        ROS_INFO("Starting procedure goal is %f",goal->distance);
        ros::Duration(10).sleep();
        starting_action_.setSucceeded([](bool result){selfie_msgs::startingResult starting_result; starting_result.drive_mode = result; return starting_result;}(false));
    }

    void drive_action_goalCB(const selfie_msgs::drivingGoalConstPtr &goal)
    {
        ROS_INFO("Drive goal is %d",goal->mode);
        ros::Duration(10).sleep();
        drive_action_.setSucceeded([](bool result){selfie_msgs::drivingResult drive_result; drive_result.parking_area = result; return drive_result;}(true));
    }
    void park_action_goalCB(const selfie_msgs::parkGoalConstPtr &goal)
    {
        ROS_INFO("Park goal is");
        ros::Duration(10).sleep();
        park_action_.setSucceeded([](bool result){selfie_msgs::parkResult park_result; park_result.done = result; return park_result;}(true));
    }
    void search_action_goalCB(const selfie_msgs::searchGoalConstPtr &goal)
    {
        ROS_INFO("Search goal is %f",goal->min_spot_lenght);
        ros::Duration(10).sleep();
        search_action_.setSucceeded(this->getMockObstacle());
    }

    selfie_msgs::searchResult getMockObstacle()
    {

        selfie_msgs::searchResult result;
        geometry_msgs::Point32 p;
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        return result;
    }

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_mock");

    ActionMock action_mock;

    while (ros::ok())
    {
        ros::spinOnce();
    }
}
