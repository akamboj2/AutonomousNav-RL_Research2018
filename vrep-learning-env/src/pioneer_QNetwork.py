#!/usr/bin/env python
#using the pioneer robot and hokuyo urg sensor to learn obstacle avoidance
#states will be heading to goal and 343 range sensor readings
#output will be leftMotor and right motor speed
#positive reward at goal
#negative reward if obstacle is hit


#NOTE: in the vrep environment the robot starts at 0,0 oriention 0, facing positive x,
#positive y is to the left of it
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf

pose=[0,0] #x,y
orient=0 #orientation is from 0 to 2pi (0 at x going ccw)
ranges=np.ones(343)*6 #need an extra bc 342 length means 341 points and going everyother we end up with an extra point
#max range is 5 so default 6 should be fine
goalpose=[4,4]

def orientCallback(msg):
    #remember orientation is only the z from -pi to pi centered at pos x
    #but we want it from 0 to 2pi
    global orient
    heading=msg.z if msg.z>0 else msg.z+2*np.pi
    #goal_dist=((goal[0]-pose[0])**2+(goal[1]-pose[1])**2)**(.5)
    goal_head=np.arctan2(goal[1]-pose[1],goal[0]-pose[0])
    if goal_head<0: goal_head+=np.pi*2
    orient=heading-goal_heading

def poseCallback(msg):
    global pose
    pose=[msg.x,msg.y]
    #print pose

def scanCallback(msg):
    #1d array of x,y,z
    #NOTE: points are relative to robot position and orientation (pos x is infront pos y is to the left of it)
    xs,ys=[],[]
    count=0
    global ranges
    for i in range(0,len(msg.data),3):
        if count%2==0:
            x=msg.data[i]
            y=msg.data[i+1]
            xs.append(x)
            ys.append(y)
            ang=np.arctan2(y,x)
            ind=int(round(ang/.00612395784838)/2)#after rounding you have correct index            #out of 684 then divide by 2 to find correct index overall
            #out of 684 then divide by 2 to find correct index overall
            ind+=171 #this is bc rn the index goes from -171 to 171, but we want it from 0 to 342
            #print x,y,pose,ind
            ranges[ind]=(x**2+y**2)**(.5)
        count+=1
    """
    max range is about 5 m
    684 scans -2.09439502791(about -120 degrees) to 2.09439214038 (about 120 degrees)
    increments of 0.00612395784838 (about .53 degrees)

    amt=len(ranges)
    print amt
    start,end=np.arctan2(ys[0],xs[0]),np.arctan2(ys[amt-1],xs[amt-1])
    print "from ",start, "to",end, "increments of",(end-start)/amt
    """
    #print ranges
    #plt.plot(xs,ys,'.')
    #plt.show()
def reset():
    #make sure all
    while startPub.getNumSubscribers()==0:
        #wait until vrep subscribes to startsim (meanign it is read to start)
        pass
    while startPub.getNumSubscribers()!=0:
        startPub.publish(True) #wait until it has started (then it will stop subscrbing to startsim)
    #reset the stuff if needed
    #pose=[0,0] #x,y
    #orient=0 #orientation is from 0 to 2pi (0 at x going ccw)
    #ranges=np.ones(343)*6

    while
def learn():
    #output are actions (combos of left,right motor speed from -3 to 3 step .2)
    h=309600 #num of hidden neurons
    inputs=tf.placeholder(shape=[1,344])
    W1=tf.get_variable("W1",shape=[344,h],intializer=tf.contrib.layers.xavier_initializer())
    L1=tf.nn.relu(tf.matmul(inputs,W1))
    W2=tf.get_variable("W2",shape=[h,900],initializer=tf.contrib.layers.xavier_initializer())
    Qout=tf.matmul(L1,W2)
    #output=tf.nn.softmax(900)#I think we want a function approximator
    #this of output as if you were to print 30 rows of 30 (starting at index 0 to 30, then 30 to 60 etc)
    #you would get a gird that goes from -3,3 down to -3,3 (y increases going up, x increases going right)
    predict=tf.argmax(out,1)
    nextQ=tf.placeholder(shape=[1,900])
    loss=tf.reduce_sum(tf.square(nextQ-Qout))
    trainer=tf.train.GradientDescentOptimzer(learning_rate=.01)
    update=trainer.minimize(loss)

    init = tf.global_variables_initializer()

    y=.90
    e=.1
    episodes=2000
    reset()

    with tf.Session() as sess:
        sess.run(init)
        for i in range(episodes):
            while j<200:
                state=np.concatenate(([orient],ranges),0) #to make 1 long 1d array with orient, then list of ranges
                a,allQ=sess.run([predict,Qout],feed_dict={inputs:})


    #initial idea was to make a an ouput layer of 2 with a left and right motor control
    #but then what would the loss function be?? some function of reward? but what is that? supervised training?
    #will it work?--would probs have to use batches instead of incremental learning
    """h=688 #num of hidden neurons
    inputs=tf.placeholder(shape=[1,344])
    W1=tf.get_variable("W1",shape=[344,h],intializer=tf.contrib.layers.xavier_initializer())
    L1=tf.nn.relu(tf.matmul(inputs,W1))
    W2=tf.get_variable("W2",shape=[h,2],initializer=tf.contrib.layers.xavier_initializer())
    outLayer=tf.matmul(L1,W2)
    output=tf.nn.tanh(outLayer)"""

if __name__=='__main__':
    rospy.init_node('pioneer_learn',anonymous=True)
    leftSpeed=rospy.Publisher("/leftMotorSpeed",Float32,queue_size=1)
    rightSpeed=rospy.Publisher("/rightMotorSpeed",Float32,queue_size=1)
    orientSub=rospy.Subscriber("/robotGlobalOrientation",Point,orientCallback)
    posSub=rospy.Subscriber("/robotGlobalPosition",Point,poseCallback)
    scanSub=rospy.Subscriber("/scan",Float32MultiArray,scanCallback)
    startPub=rospy.Publisher("/start_sim",Bool,queue_size=1)
    stopPub=rospy.Publisher("/stop_sim",Bool,queue_size=1)

    learn()
    while True:
        pass
    #while not rospy.is_shutdown():
    #    pass
