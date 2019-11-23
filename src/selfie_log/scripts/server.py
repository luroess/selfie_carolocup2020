#!/usr/bin/env python

import rospy

from selfie_msgs.msg import Log
from dynamic_reconfigure.server import Server
from selfie_log.cfg import LogConfigConfig

pub = rospy.Publisher('log_msg', Log, queue_size=10)
def pass_log_config(config):
    msg = Log()
    for i in config:
        print i
    for i in dir(msg):
        print i
def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {scheduler_node}, {scheduler_driving}""".format(**config))
    pass_log_config(config)
    msg = Log()
    msg.scheduler_node = config.scheduler_driving
    pub.publish(msg)
    return config
if __name__ == "__main__":
    rospy.init_node("selfie_log", anonymous = False)
    srv = Server(LogConfigConfig, callback)
    rospy.spin()
