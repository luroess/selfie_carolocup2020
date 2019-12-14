#!/usr/bin/env python
import rospy
from selfie_msgs.msg import parkAction, parkActionGoal, parkGoal
from geometry_msgs.msg import Point
from actionlib import SimpleActionClient

if __name__ == '__main__':
  rospy.init_node('park_mock')
  goal = parkGoal()
  goal.parking_spot.points.append(Point(0,-1,0))
  goal.parking_spot.points.append(Point(1,-1,0))
  goal.parking_spot.points.append(Point(1,0,0))
  goal.parking_spot.points.append(Point(0,0,0))
  
  
  cli = SimpleActionClient('park',parkAction)
  cli.wait_for_server()
  print 'server active'
  cli.send_goal(goal)
  print 'waiting'
  cli.wait_for_result()
