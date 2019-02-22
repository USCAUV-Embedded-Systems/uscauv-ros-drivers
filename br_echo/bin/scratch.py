#simplePingExample.py
from brping import Ping1D
import time
import rospy
from builtins import input
from std_msgs.msg import String

def callback(data):
    print("data: %s" % data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("br_echo", String, callback)

if __name__ == '__main__':
    listener()