#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def update_msg(message):

    rospy.loginfo(rospy.get_caller_id() + " Mensagem: %s", message.data)


def app():

    rospy.init_node('app_node', anonymous=False)

    rospy.Subscriber("sensor_topic", String, update_msg)

    rospy.spin()

if __name__ == '__main__':
    app()