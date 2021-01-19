#!/usr/bin/env python
"""after running this look at the mapdata.txt. it's supposed
to show the probabilites of an object actually being otherwise
"""
"""Purpose of this script is to visualize the data in nav msgs
occupancy grid
I think it's just the pixels
data is a 4000,4000 array with some values 100 and some 0
100 represents somethings definately there, 0 means something
is definately not there. -1 is unkown
"""
from __future__ import print_function
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
def showMap(msg):
    m=msg.data
    """
    #for i in len(msg.data)/2:
    a=np.reshape(m,(4000,4000))
    ex1=np.ones(4000)*-1
    minx=0
    miny=0
    c=0
    for x in a[:,]:
        c+=1
        if not np.array_equal(x,np.transpose(ex1)):
            minx=c
            break
    c=0
    for y in a:
        c+=1
        if not np.array_equal(y,(ex1)):
            miny=c

    print (minx,miny)
    print(a[1982:2300,2006:2300])"""
    f=open('/home/isler/Abhi/create_ws/src/create_autonomy/ca_driver/scripts/mapdata.txt','w')
    #f.write("Hellow???")
    print(f.name)
    for i in range(len(m)):
        if i>7925000 and i<8024000:
            if i%4000==0:
                f.write('\n')
                print("")
            if i%4000>1980 and i%4000<2020:
                f.write(str(m[i])+' ')
            if m[i]!=-1:
                print(str(m[i]),end=' ')
                #print(str(m[i])+","+str(i),end=' ')
    f.close()
    print(len(m))

rospy.Subscriber("/map",OccupancyGrid,showMap)
rospy.init_node("map_data",anonymous=True)
rospy.spin()
