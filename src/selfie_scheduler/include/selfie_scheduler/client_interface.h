#ifndef CLIENT_INTERFACE_H
#define CLIENT_INTERFACE_H

#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <boost/any.hpp>

class ClientInterface
{
public:
    virtual ~ClientInterface() = 0;
    virtual bool waitForServer(float timeout) = 0;
    virtual void setGoal(boost::any goal) = 0;
    virtual bool waitForResult(float timeout) = 0;
    virtual void cancelAction() = 0;
    virtual int isActionFinished() = 0;
    virtual void getActionResult(boost::any &result) = 0;
    virtual program_state getActionState() = 0;
    virtual action getNextAction() = 0;

};
#endif // CLIENT_INTERFACE_H