#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sys

def ask_for_question():
    print("What would you like to ask?")
    question = ""

    try:
        question = str(input())
    except KeyboardInterrupt:
        sys.exit(0)

    return question

def talker():
    rospy.init_node('node_talker', anonymous=True)
    
    # This will make the node run at 10Hz, as to not use full PC strenght
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        question = ask_for_question()
     
            
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass