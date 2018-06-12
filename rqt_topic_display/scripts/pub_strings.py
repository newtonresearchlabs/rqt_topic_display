#!/usr/bin/env python
# demonstrate string display in rqt_topic_display by publishing a variety of strings

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32


rospy.init_node("pub_strings")

msgs = ["test",
        "foo",
        "kjsdl kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk",
        "1\n2\n3\n4\n5\n6\n"]


pub = rospy.Publisher("string", String, queue_size=1)
num_pub = rospy.Publisher("number", Float32, queue_size=1)

count = 0
while True:
    done = False
    for msg in msgs:
        if rospy.is_shutdown():
            done = True
            break
        pub.publish(String(msg))
        num_pub.publish(Float32(count))
        rospy.sleep(2)
        count += 1
    if done:
        break
