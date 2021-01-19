#this was an attempt to print the robots path

#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
#map, = plt.plot([],][])
ox,oy=0,0

def poseCallback(data):
    x,y=data.pose.pose.position.x,data.pose.pose.position.y

    print x,y
    if ox==x and oy==y:
        return
    plt.plot(x,y,'.')
    #plt.plot(1,1,'o')
    #plt.draw()
    plt.show()
    ox,oy=x,y


def listen():
    rospy.init_node('nav_path')
    rospy.Subscriber("/odom",Odometry,poseCallback)
    rospy.spin()

if __name__=='__main__':
    #plt.figure
    listen()
    #plt.show()
