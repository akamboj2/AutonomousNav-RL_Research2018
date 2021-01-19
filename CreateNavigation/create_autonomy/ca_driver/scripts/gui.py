#!/usr/bin/env python

#this is the main gui for the create navigation project
#origin is top left corner; pos x to right, pos y downwards

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import Tkinter as tk
from AStarPath import AStarPlanner

#below info is in yaml file #NOTE ignore the next 2 commented out lines, i think that's false
#offset bc origin of graph is at -100,-100 (in yaml file)
#offx,offy=[100,100]
res=.05 # resolution in meters per pixels .05 in example

"""center is where you want the center of the map to be anchored in the gui
 if you started mapping in the top left corner, gmapping would have created
 a pgm file with 0,0 being that spot so you would want to anchor it somewhere in
 the top left corner relative to the size of the map. Changing center would
 pan the map on the gui"""
center=[100,100]
offx,offy=center
size=[600,600] #size is the size of the gui in pixels [width,length]
#NOTE: Make sure size is well larger than the map, because path planner won't be able to
#see some of the map if that is not true^^^
#NOTE: make the size a square bc the path planner assumes its a square
pos_circle=0
square=0 #square is destination
currpose=[0,0]
plan=[]
points=[]
planner=None
dest=[]
def motion(event):
    x, y = event.x, event.y
    x=(x-offx)*res
    y=-(y-offy)*res
    #NOTE: all the y are flipped because tkinter y increases downwards but for gmapping
    #and our map, y is negative below the starting point
    #print('{}, {}'.format(x, y))
    T.delete(1.0,tk.END)
    T.insert(tk.END,str(x)+","+str(y))

def click(event):
    x,y=event.x,event.y
    global square
    if (square!=0):
        canvas.delete(square)

    square=canvas.create_rectangle(x-4,y-4,x+4,y+4,fill='blue')
    x1,y1=x,y
    #offset bc origin of graph is at -100,-100
    #resolution is .05 meters/pixel
    x=(x-offx)*res
    y=-(y-offy)*res
    print "clicked at",x,y
    global dest
    dest=[x,y]
    T.delete(1.0,tk.END)
    T.insert(tk.END,"clicked at "+str(x)+","+str(y))
    global planner
    planner = AStarPlanner(res,3,size[0])
    #works better if Planner plans more area then you see just in case
    root.after(0,plot_path)
    root.update()
    root.after(0,move_path)
    #move_path()
    #sq=canvas.create_rectangle(x1-4,y1-4,x1+4,y1+4,fill='green')
    """problem: for some reason after calling planner.PlanPath
    tkinter canvas.create functions stop working
    soln: Use a call back (p) to call planner.PlanPath after 0milliseconds in the mainloop()
    explanation: I think matplot lib blocks tkinter and when you comment out plt.show()
    in AStarPath, canvas is more cooperative here
    """
def plot_path():
    global plan
    print "in plot_path"
    if plan!=[]:
        for po in points:
            canvas.delete(po)
    plan=planner.PlanPath(currloc=currpose,goal=dest)
    if plan==None:
        print "Path Planning failed."
        return
    #print plan
    global points
    points=[[]]
    c=0
    numpts=20 #how many points you want to print
    nth=len(plan)/numpts
    if nth==0: nth=1
    for pt in range(len(plan)):
        if pt!=len(plan)-1 and pt%nth!=0:
            continue
        a=plan[pt]
        #the plan's cordinates are centered (0,0) at row_len/2,row_len/2 (and we put row_len as size)
        #we want the cordinates centered at center[x,y]. Also we have to flip y cordinate bc in matplotlib (what AStarPath planner map was based off of/visualized with )
        #x,y start in bottom left and y goes postive upwards which is reverse of what we have here
        plan[pt]=[(a[0]-size[1]/2)+center[0],-(a[1]-size[1]/2)+center[1]]
        a=plan[pt]
        points[c]=canvas.create_oval(a[0]-1,a[1]-1,a[0]+1,a[1]+1,fill='black')
        points.append([])
        c+=1

def move_path():#"""NEED TO TEST THIS FUNCTION NOW WITH ACTUAL ROBOT!"""
    for pt in range(len(plan)):
        if pt!=len(plan)-1 and pt%15!=0 or pt==0 :
            continue
        x,y=plan[pt]
        x=(x-offx)*res #translate from pixels to meters
        y=-(y-offy)*res
        print "Robot moving to ",x,y
        msg.x=x
        msg.y=y
        destPub.publish(msg)
        while((currpose[0]-x)**2+(currpose[1]-y)**2)**(.5)>.05:
            pass
            #print currpose
            #print "\its at ",currpose[0],currpose[1]

def setPose(msg):
    #we publish the x,y to currpose in meters not pixels
    print msg
    x,y=msg.pose.pose.position.x,msg.pose.pose.position.y
    xd=x/res+offx#transform from meters to pixels
    yd=-y/res+offy
    global currpose,pos_circle
    currpose=[x,y]
    print "currpose changes to ",x,y,
    if pos_circle!=0: canvas.delete(pos_circle)
    pos_circle=canvas.create_oval(xd-4,yd-4,xd+4,yd+4,fill='red')


if __name__=='__main__':

    #add image onto canvas
    root =tk.Tk()
    img=tk.PhotoImage(file="~/Abhi/create_ws/halfHall.pgm")
    #img=img.subsample(2,2)
    canvas=tk.Canvas(root,width=size[0],height=size[1])
    canvas.pack()
    #canvas.pack(side="bottom",fill="both",expand="yes")
    canvas.create_image(center[0],center[1],anchor='center',image=img)
    #that means the center of the picture is anchored at point center[0],center[1] in the canvas
    #so if you change the centor it pans the image

    #for showing position
    x,y=currpose
    print currpose
    x=x/res+offx#transform from meters to pixels
    y=-y/res+offy
    print x,y
    pos_circle=canvas.create_oval(x-4,y-4,x+4,y+4,fill='red')

    #create text under image
    T=tk.Text(root,height=1)
    T.pack(side="bottom")
    T.insert(tk.END,"words\n")

    #make buttons work on canvas
    root.bind('<Motion>', motion)
    root.bind('<Button-1>',click)

    #for some reason other functions can still use these publishers/subscribers
    #even tho they aren't global... or are they.???
    destPub=rospy.Publisher("/destination",Point,queue_size=1)
    poseSub=rospy.Subscriber("/odom",Odometry,setPose)
    msg=Point(x=0,y=0,z=0)

    rospy.init_node('Nav_gui',anonymous=True)

    #global planner #need to readjust A* algo if this will stay here
    #planner = AStarPlanner() #needs to happen after initiailizing rospy
    while not rospy.is_shutdown():
        root.mainloop()
    root.destroy() #so Tkinter closes when we control c
