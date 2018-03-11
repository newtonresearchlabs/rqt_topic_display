#!/usr/bin/env python
# demonstrate string display in rqt_topic_display by publishing a variety of strings

import rospy

from std_msgs.msg import String


rospy.init_node("pub_strings")

msgs = ["test",
        "foo",
        "kjsdl kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk",
        "1\n2\n3\n4\n5\n6\n"]


pub = rospy.Publisher("string", String, queue_size=1)

while True:
    done = False
    for msg in msgs:
        if rospy.is_shutdown():
            done = True
            break
        pub.publish(String(msg))
        rospy.sleep(1)
    if done:
        break
