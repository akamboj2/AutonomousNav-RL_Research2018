#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid



class mnode:
    def __init__(self,x,y,scale):
        self.g=0 #backptr length
        self.h=0 #dist(x,y,[0,0])
        #print "mnode: ",x,y,self.h
        self.f=self.h+self.g
        self.backptr=None
        self.visited=False
        self.x=x
        self.y=y
        self.scale=scale
    def update_f(self):
        self.f=self.h+self.g
    def update_h(self,goal):
        self.h=self.dist(goal)#this is to update the goal
        self.update_f()
    def dist(self,dest):
        return (((self.x-dest[0])**2+(self.y-dest[1])**2)**(.5))*self.scale

class AStarPlanner:
    def __init__(self,res=1,scale=1,rlen=400):
        self.map=[]#np.array([np.zeros(400)]) #need np arrays to be able to do in operator
        self.row_len=rlen #row length of map (it is square so same as column length)
        #NOTE: map should be centered in middle of map (so point (self.row_len/2,rowlen/2) translates to (0,0)
        # on the actual map created by gmapping)
        self.O=[] #Open set for A*
        self.C=[] #Closed set for A*
        self.nodemap=[]
        self.step=res
        self.scale=3#scale decreases the resolution to make this run faster. if your scale is 5 you skip every fifth pixel when drawing the node map
        #side effects of scale: clicking on an unscaled map near the boarder may be out of bounds in the scaled down map
        rospy.Subscriber("/map",OccupancyGrid,self.drawMap)
        #rospy.init_node("map_drawer",anonymous=True)
        #don't need to init node if already done in gui

        while(len(self.map)<self.row_len):#this gives ros enough time to make the map
        #    print len(self.map)
            print "waiting for map.."
        #print len(self.nodemap),len(self.map)#can check to make sure map is made
        #self.map=np.array(self.map)
        #print self.map
        #print self.nodemap
        #flood fill algo
        #PlanPath(.05,currloc=[300,200],goal=[190,5])

    def drawMap(self,msg):
        off=[0,0]
        m=msg.data
        dim=int((len(m))**(1./2)) #the map is a dimxdim map
        #i believe it's 4000 in our case^^
        h_size=self.row_len/2 #this is half the size of from the middle of the map that you want
        row_start=int(dim*(dim/2)-(h_size-off[1])*dim)
        row_end=int(dim*dim/2+(h_size+off[1])*dim)
        #self.row_len=h_size*2 #row length of map
        #print dim,h_size,row_start,row_end
        count=0
        for row in range(row_start,row_end,dim):#go through rows by adding dim each time
            self.map.append(m[(row+(2000-h_size))+off[0]:(row+(2000+h_size))+off[0]])
            if count%self.scale!=0:
                count+=1
                continue #skip every other nodein y dir
            self.nodemap.append([])
            for i in range(self.row_len):
                if i%self.scale!=0: continue #skip every other node in x dir
                if self.map[count][i]==0:
                    self.nodemap[count/self.scale].append(mnode(i/self.scale,count/self.scale,self.step*self.scale))
                else:
                    self.nodemap[count/self.scale].append(None)
            count+=1 #this is count of rows
    #so the occupancy grid map has pos x going left to right but positive y going top to bottom


    def PlanPath(self,currloc,goal,r_rad=-1):
        #tranform from gmapping map coordinates to mapnode's cordinates (which starts at 0,0 being top left pos x towards right and pos y down)
        #this was done bc gmapping has a 4000x4000 pixel array with 2000,2000 being center and you can't index at negative number
        #that is too large so it was cropped down, while keeping the resolution the same
        currloc=[int(currloc[0]/self.step+self.row_len/2),int(currloc[1]/self.step+self.row_len/2)]
        goal=[int(goal[0]/self.step+self.row_len/2),int(goal[1]/self.step+self.row_len/2)]
        currloc=[currloc[0]/self.scale,currloc[1]/self.scale]
        goal=[goal[0]/self.scale,goal[1]/self.scale]
        print currloc,goal
        self.row_len=self.row_len/self.scale #this is for skipping every other point in map

        #first edit map to account for size of robot
        if r_rad!=-1:#if you want to account for radius of robot note this works but runs signficantly slower
                    #and requires a fairly well defined/precise graph
            #find edges
            xs=[]
            ys=[]
            edges=[]
            for y in range(self.row_len):
                for x in range(self.row_len):
                    #remember that the array goes up-down y first, then side to side x in indices
                    if self.nodemap[y][x]!=None and (y-1==0 or x-1==0 or y+1==self.row_len or x+1==self.row_len or self.nodemap[y-1][x]==None or self.nodemap[y+1][x]==None or self.nodemap[y][x-1]==None or self.nodemap[y][x+1]==None):
                        edges.append(self.nodemap[y][x])
                        xs.append(x)
                        ys.append(y)

            #plt.plot(xs,ys,'y.')
            #plt.show() #shows the edges

            for k in range(self.row_len):#to filter for robot size
                for l in range(self.row_len):
                    if self.nodemap[k][l]!=None:
                        for ed in edges:
                            if ed!=None and (self.nodemap[k][l].dist([ed.x,ed.y])<=r_rad):
                                #print "edge too close"
                                self.nodemap[k][l]=None
                                break
            """#shows the mapped area minus the robot width correction
            xs=[]
            ys=[]
            for y in range(self.row_len):
                for x in range(self.row_len):
                    if self.nodemap[y][x]!=None:
                        xs.append(x)
                        ys.append(y)
            plt.plot(xs,ys,'.')
            plt.show() """
        #print self.row_len
        #Next update node goal distance (the heursitic cost to goal is euclidean distance)
#        print "node map is ",len(self.nodemap),"by",len(self.nodemap[0])
        for y in range(self.row_len):
            for x in range(self.row_len):
                if self.nodemap[y][x]!=None: self.nodemap[y][x].update_h(goal)

        #to print self.nodemap and display path
        """xs=[]
        ys=[]
        for y in range(self.row_len):
            for x in range(self.row_len):
                if self.nodemap[y][x]!=None:
                    xs.append(x)
                    ys.append(y)
        plt.plot(xs,ys,'.')#plot all points, then points visited then path
        print goal
        plt.plot(goal[0],goal[1],'rx')
        plt.show()"""

        if self.nodemap[goal[1]][goal[0]] is None:
            print "INVALID GOAL it's outside of the known mapped area!"
            return None

        #do A*
        start=self.nodemap[currloc[1]][currloc[0]]
        self.O.append(start)#initialize O to start node
        currgoal=None
        #print "starting at "+str(self.O[0])
        while(len(self.O)!=0):
            #find node in O with least estimated cost to goal f
            minf=9999999
            n=None
            for node in range(len(self.O)):#neeed to do range(len()) to modify a node in O (instead of a copy of it)
                if self.O[node].f<minf:
                    minf=self.O[node].f
                    n=self.O[node]
            #remove that node from O add it to C
            self.O.remove(n)
            self.C.append(n)
            #if your at goal then you're done
            if n.x==goal[0] and n.y==goal[1]:
                #print self.O
                currgoal=n
                for node in range(len(self.O)):
                    if self.O[node].f<minf:
                        minf=self.O[node].f
                        n=self.O[node]
                    #print self.O[node].x,self.O[node].y,self.O[node].f,self.O[node].h,self.O[node].g
                #print minf,n.x,n.y,n.f,n.h,n.g
                if minf<currgoal.f:
                    continue
                else:
                    break
                    #print "AT GOAL!!"
                    #break #this might need to be a continue to find optimal path
            #expand n
            adj=[]
            if n.x+1<self.row_len: adj.append(self.nodemap[n.y][n.x+1])
            if n.y-1>=0: adj.append(self.nodemap[n.y-1][n.x])
            if n.x-1>=0: adj.append(self.nodemap[n.y][n.x-1])
            if n.y+1<self.row_len: adj.append(self.nodemap[n.y+1][n.x])

            for a in range(len(adj)):
                if adj[a] in self.C or adj[a]==None: continue #ignore anything already visited or anything that's none
                if adj[a] not in self.O:
                    #print "new neighbor added to O"
                    self.O.append(adj[a])
                    adj[a].backptr=n
                    adj[a].g+=self.step+n.g
                    adj[a].update_f()
                elif n.g+self.step< adj[a].g:
                    #print "shorter path found, shall expore"
                    adj[a].backptr=n
                    adj[a].g+=self.step+n.g
                    adj[a].update_f()
            """if len(self.C)%5000==0: #NOTE FOR DEBUGGING
                xclosed=[]
                yclosed=[]
                for node in self.C:#this shows all the points visited
                    xclosed.append(node.x)
                    yclosed.append(node.y)
                #to print self.nodemap and display path
                xs=[]
                ys=[]
                for y in range(self.row_len):
                    for x in range(self.row_len):
                        if self.nodemap[y][x]!=None:
                            xs.append(x)
                            ys.append(y)
                plt.plot(xs,ys,'.')#plot all points, then points visited then path
                plt.plot(xclosed,yclosed,"g.")
                plt.show()"""

        xpath=[]
        ypath=[]
        path=[]
        n=currgoal
        while(n !=start and n!=None):
            xpath.append(n.x)
            ypath.append(n.y)
            path.append([n.x,n.y])
            n=n.backptr
        if n==None: print "Failed am at None!"
        """ #to view map!
        xclosed=[]
        yclosed=[]
        for node in self.C:#this shows all the points visited
            xclosed.append(node.x)
            yclosed.append(node.y)
    #    print "length of C",len(self.C)
        #to print self.nodemap and display path
        xs=[]
        ys=[]
        for y in range(self.row_len):
            for x in range(self.row_len):
                if self.nodemap[y][x]!=None:
                    xs.append(x)
                    ys.append(y)
        plt.plot(xs,ys,'.')#plot all points, then points visited then path
        plt.plot(xclosed,yclosed,"g.")
        plt.plot(xpath,ypath,"r-")
        plt.show()
        """
        path.reverse()
        for p in range(len(path)):
            path[p]=[path[p][0]*self.scale,path[p][1]*self.scale]
        return path



if __name__=='__main__':
    rospy.init_node("path_planner",anonymous=True)
    planner=AStarPlanner(.05,3,600)
    path=planner.PlanPath(currloc=[8,0],goal=[-1.5,-9])
    #algorithm fails at point goal=0,-3 or mathplot lib 300,240 because that point is a known point that is not
    #connected to any other points (so Gmapping's fault) therefore the A* algorith just keeps' searching
    #could fix it by checking goal isn't landlocked by the unkown--but that could be a potentially long and expensive computation
    #so user must know there is a path or wait a very long time for the algorithm to search every node and fail
    #path=planner.PlanPath(.05,currloc=[8,1],goal=[0,-3])

    #print path






















"""
global map, shapes,edges,self.row_len

shapes=[]#np.array([])
edges=[]#np.array([])
all=[]
    for y in range(len(map)):
        for x in range(len(map[y])):
            pt=[x,y]
            #print pt
            #print x,y
            NotInShape=True
            for i in shapes: #this is alternative to just doing in shapes
                if pt in i: NotInShape=False
            if map[x][y]==0 and NotInShape:
            #    shapes=shapes.tolist()
                shapes.append([]) #add a new shape
            #    shapes=np.array(shapes)
            #    edges=edges.tolist()
                edges.append([]) #add a new edges list
            #    edges=np.array(edges)
                #flood_fill(pt,len(shapes)-1)#send point and index of new shape
                all.append(node(x,y,dist_to_goal([x,y])))
                shapes[len(shapes)-1].append(pt) #add it to a shapes
                if x-1==0 or x+1>=self.row_len or y-1==0 or y+1>=self.row_len or map[x-1][y]!=100 or map[x+1][y]!=100 or map[x][y+1]!=100 or map[x][y-1]!=100:
                    edges[len(shapes)-1].append(pt)
    for n in all:
        x,y=n.x,n.y
        if y-1>=0 and map[x][y-1]==0:
            for i in all:
                if i.x==n.x and i.y==n.y-1:
                    n.up=i
                    break
        if x+1<self.row_len and map[x+1][y]==0:
            for i in all:
                if i.x==n.x+1 and i.y==n.y:
                    n.right=i
                    break
        if y+1<self.row_len and map[x][y+1]==0:
            for i in all:
                if i.x==n.x and i.y==n.y+1:
                    n.down=i
                    break
        if x-1>=0 and map[x-1][y]==0:
            for i in all:
                if i.x==n.x-1 and i.y==n.y:
                    n.left=i
                    break

                    """
#    print edges
#    print shapes
"""xs,ys=[],[]
    for s in shapes:#flip all the y values
        for p in range(len(s)):
            y,x=s[p][0],s[p][1]
            s[p]=[x,y]
            xs.append(x)
            ys.append(y)
    for e in edges:#flip all the y values
        for p in range(len(e)):
            y,x=e[p][0],e[p][1]
            e[p]=[x,y]
            xs.append(x)
            ys.append(y)
    plt.plot(xs,ys,'.')
    plt.show()"""
"""
    hres=.0025 #this is half the resolution in meters per pixel
    fmap=FeatureEnvironment(robotRadius=.5)
    for s in shapes:
        for pt in s:
            x,y=pt[0],pt[1]
            fmap.addFeature([(x-hres,y-hres),(x-hres,y+hres),(x+hres,y+hres),(x+hres,y-hres)])
    planner=RRTPlanner(fmap,2)
    planner.makeTree(200)
    pathNodes=planner.findPath((0,0,0),(1,0,0))
    """
