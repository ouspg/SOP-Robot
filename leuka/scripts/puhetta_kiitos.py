#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool
from tts import onko_kaynnissa

valamis = False

def Onko_puhe_valmis(data):
    global valamis
    rospy.loginfo("On puhetta")
    if data.data and not valamis:
    	pub3.publish((True))
        valamis = True

def talker():
    global valamis
    rospy.init_node('talker', anonymous=True)
    pub2.publish("man")
    hello_str = "ä ö" 
    rospy.loginfo(hello_str)
    pub.publish((hello_str)) #EMME YMMÄRRÄ ÄÄKKÖSIÄ
    rospy.spin()

pub = rospy.Publisher('text', String, queue_size = 100)
sub0 = rospy.Subscriber("tts_ready", Bool, Onko_puhe_valmis)
pub3 = rospy.Publisher("servo_ready", Bool, queue_size = 1)
pub2 = rospy.Publisher("gender", String, queue_size = 5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass