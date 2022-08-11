#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.path as mpltPath
import math, tf, time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
# from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

cost_update = OccupancyGridUpdate()
result = MoveBaseActionResult()
newPose = PoseStamped()
q = Quaternion()

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]


class jackal_explore:
    def __init__(self):
        self.costmap = OccupancyGrid()
        self.flagm = 0
        self.flagg = 0
        self.init = 0
        self.newcost = []
        self.grid = OccupancyGrid()
        self.feedback = MoveBaseActionFeedback()
        self.prevpnt = -1
        self.nextpnt = np.asarray([0, 0])
        self.sample = []
        self.stuckpnt = []
        self.isstuck = False
        self.offsetcnt = 0
        self.firstgoal = 0
        self.checkpnt = 0
        self.pntlist = []
        self.send = True
        self.currentpos =[]
	self.heading = 0
	self.area = 0
	self.num_unknown = 0
	self.scaledprevpnt = np.asarray([0, 0])
	self.map_res = 0
	self.map_width = 0
	self.map_height = 0
	self.map_origX = 0
	self.map_origY = 0
	self.min_bndX = -10
	self.max_bndX = 10
	self.min_bndY = -10
	self.max_bndY = 10
	self.min_bnd_indX = 0
	self.max_bnd_indX = 0
	self.min_bnd_indY = 0
	self.max_bnd_indY = 0
	self.scanPts = []
	self.poseX = 0
	self.poseY = 0
	self.pubScanLocation = 0
	self.voxelPts = []

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb,queue_size=1)
        self.global_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costCb,queue_size=1)
        self.globalup_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costupCb,queue_size=1)
        self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
        self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.line_pub = rospy.Publisher('/visualization_marker',Marker,queue_size = 10)
	self.laser_sub = rospy.Subscriber('/front/scan', LaserScan, self.LidarCb,queue_size=1)
	self.laserPt_sub = rospy.Subscriber('/scan_cloud', PointCloud2, self.ScanCloud)
	self.localization_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.LocalizationCb)
	self.octomap_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.VoxelCenter)

    def LidarCb(self,data):
	dist = data.ranges
	max_dist = data.range_max
	area = 0
	for i in range(0,len(dist)):
	    if math.isinf(dist[i]):
		area = area + max_dist
	    else :
		area = area + dist[i]
	self.area = area

    def costupCb(self,data):
        self.flagg = 1
        cost_update = data

        if self.init == 1 and self.flagm == 0:
            gpnts = cost_update.data
            gwidth = cost_update.width
            gheight = cost_update.height
            gx = cost_update.x
            gy = cost_update.y
            cnt = 0
            for x in range(gheight):
                for y in range(gwidth):
                    g = gpnts[cnt]
                    self.newcost[x+gy][y+gx] = g
                    cnt += 1

    def costCb(self,data):
	rospy.loginfo('cost map received')

        self.newcost = []
        self.flagm = 1
        self.costmap = data

        pnts = self.costmap.data
        cnt = 0

        for x in range(self.costmap.info.height):
            newcost_x = []
            for y in range(self.costmap.info.width):
                newcost_x.append(pnts[cnt])
                cnt+=1
            self.newcost.append(newcost_x)

    def mapCb(self,data):
        self.grid = data
	cell_val = data.data
	self.num_unknown = cell_val.count(-1)
	rows = len(str(cell_val))
	cols = len(str(cell_val[0]))
	
        self.map_res = self.grid.info.resolution
        self.map_width = self.grid.info.width
        self.map_height = self.grid.info.height 
        self.map_origX = self.grid.info.origin.position.x
        self.map_origY = self.grid.info.origin.position.y

	self.min_bnd_indX = round((self.min_bndX - self.map_origX) / self.map_res) 
	self.max_bnd_indX = round((self.max_bndX - self.map_origX) / self.map_res)
	self.min_bnd_indY = round((self.min_bndY - self.map_origY) / self.map_res)
	self.max_bnd_indY = round((self.max_bndY - self.map_origY) / self.map_res) 
	print rows, cols
	print self.min_bnd_indX, self.max_bnd_indX, self.min_bnd_indY, self.max_bnd_indY

        if self.flagm == 1:

            self.init = 1
            rospy.loginfo('map received')
            pnts = self.grid.data
            gpnts = self.costmap.data
	    rospy.loginfo([self.map_width, self.map_height, self.map_res, self.map_origX, self.map_origY])

            cnt = 0
            pic_mat = []

            for x in range(self.map_height):
                pic_x = []
                for y in range(self.map_width):
                    p = pnts[cnt]
                    g = gpnts[cnt]

                    if (g>0):
                        p = 0
                    else:
                        if (p ==-1):
                            p = 50
                        elif (p == 0):
                            p = 100
                        else:
                            p = 0
                    pic_x.append(p)
                    cnt += 1
                pic_mat.append(pic_x)
            self.flagm = 0
	    
            # plt.imshow(pic_mat,cmap='gray',origin='lower')
            # plt.draw()
            # plt.pause(0.001)

    def feedbackCb(self,data):
#	rospy.loginfo('move base feadback received')

        offsetx = [1,-1,1,-1]
        offsety = [1,-1,-1,1]

        self.feedback = data
        x = self.feedback.feedback.base_position.pose.position.x
        y = self.feedback.feedback.base_position.pose.position.y
        w = self.feedback.feedback.base_position.pose.orientation.w
	orientation_list = [self.feedback.feedback.base_position.pose.orientation.x, self.feedback.feedback.base_position.pose.orientation.y, self.feedback.feedback.base_position.pose.orientation.z, self.feedback.feedback.base_position.pose.orientation.w]	
	euler = tf.transformations.euler_from_quaternion(orientation_list)
	heading_tmp = euler[2]
	if heading_tmp > math.pi:        
		self.heading = heading_tmp - math.pi
	elif heading_tmp < -math.pi:
		self.heading = heading_tmp + math.pi
	self.sample.append([x,y,w])
        self.currentpos = [x,y]
	
        if self.firstgoal == 0 :
	    self.sendGoal([x+0.0,y-1.0]) #([x+0.5,y+0.5])
            self.firstgoal = 1
	    
        if len(self.sample) > 20 : #15:
            sa = np.array(self.sample)
            mx = np.average(sa[:,0])
            my = np.average(sa[:,1])
            mw = np.average(sa[:,2]) - w
            # rospy.loginfo(calc_distance([x,y],[mx,my]))

#	    if calc_distance([x,y],[mx,my]) < 0.015 and abs(mw) < 0.01:
	    if calc_distance([x,y],[mx,my]) < 0.005 and abs(mw) < 0.005:
                rospy.loginfo('Stuck. Resetting Goal...')
                self.isstuck = True
                xp, yp = x,y

                if len(self.stuckpnt) > 0:
                    sx = self.stuckpnt[0]
                    sy = self.stuckpnt[1]
                    if calc_distance([x,y],[sx,sy]) < 0.05: #0.1:
                        if self.checkpnt < 4 and self.prevpnt != 0:
                            self.prevpnt = 0
                            rospy.loginfo("Changing Point")
                            x = x + 0.5*offsetx[self.offsetcnt]
                            y = y + 0.5*offsety[self.offsetcnt]
                            self.offsetcnt += 1
                            if self.offsetcnt > 3:
                                self.offsetcnt = 0
                        # else:
                        #     rospy.loginfo('Adding taboo point')
                        #     self.pntlist.append(self.nextpnt)
                    elif self.checkpnt > 0 and self.prevpnt != 0:
                        self.prevpnt = 1

                    if self.checkpnt == 4:
                        rospy.loginfo('Adding taboo point')
                        self.pntlist.append(self.nextpnt)
                        self.checkpnt = 0
                self.stuckpnt = [xp,yp]
                self.sendGoal([x,y])
     	        rospy.loginfo('modified goal is published due to get stuck')
            self.sample = []

##
#    	idx = find_nearest_pts(self.nextpnt,self.scanPts)
#	nearestPt = self.scanPts[idx]
#	nearestDist = calc_distance(self.nextpnt,nearestPt)
#	if nearestDist < 0.5 and self.pubScanLocation == 0:
#	    diff = np.array(nearestPt) - np.array(self.nextpnt)
#	    ang = np.arctan2(diff[1],diff[0])
#	    print nearestDist, ang * 180 / math.pi
#	    if ang < 0:
#		ang = ang + math.pi
#	    else :
#		ang = ang - math.pi
#	    newX = round(self.nextpnt[0] + 0.5*math.cos(ang))
#           newY = round(self.nextpnt[1] + 0.5*math.sin(ang))
#	    self.nextpnt = [newX, newY]
#	    self.sendGoal(self.nextpnt)
#	    self.pubScanLocation = 1
	    
# here!!
	pts = np.array(self.voxelPts)
        novv = [0 for i in range(0,9)]
	num_of_visible_voxel = np.array(novv)
	print self.pubScanLocation
	if self.pubScanLocation == 0 :
	    for k in range(0,9) :
		if k == 0 :
		    x = self.nextpnt[0]
		    y = self.nextpnt[1]
		else :
		    ang = 2*math.pi * k / 8
		    x = round(self.nextpnt[0] + 0.5*math.cos(ang))
            	    y = round(self.nextpnt[1] + 0.5*math.sin(ang))
		
		block = [0 for i in range(0,len(pts))]		    
		idx = find_nearest_pts([x, y],self.scanPts)
	  	nearestPt = self.scanPts[idx]
		nearestDist = calc_distance([x, y],nearestPt)
		if nearestDist > 0.5 :
		    for i in range(0,len(pts)-1) :
		        for j in range(0,len(pts)-1) :
			    a = [pts[i,0]-x, pts[i,1]-y]
			    b = [pts[j,0]-x, pts[j,1]-y]
			    da = math.sqrt(a[0]**2 + a[1]**2)
			    db = math.sqrt(b[0]**2 + b[1]**2)
		    	    if db < da :
				tmp = (a[0]*b[0] + a[1]*b[1]) / (da*db)
				if tmp < -1 :
					tmp = -0.999
				if tmp > 1 :
					tmp = 0.999
				ang1 = math.acos(tmp)
				ang2 = abs(math.asin(self.map_res/math.sqrt(2)/db))
				if ang1 < ang2 :
					block[i] = 1
					break

		    cnt = 0
		    for i in range(0,len(block)-1) :
		        if block[i] == 0 :
			    cnt = cnt + 1
		    num_of_visible_voxel[k] = cnt
#		    print cnt

		if num_of_visible_voxel.size is not 0 :
		    max_idx = np.argmax(num_of_visible_voxel)
		    if max_idx is not 0:
			ang = 2*math.pi * max_idx / 8
  		        x = round(self.nextpnt[0] + 0.5*math.cos(ang))
            	        y = round(self.nextpnt[1] + 0.5*math.sin(ang))
			self.sendGoal([x, y])
			print "Scan Location Goal Sent!!"
			time.sleep(3)
		self.pubScanLocation = 1
    		

    def LocalizationCb(self,data):
	self.poseX = data.pose.pose.position.x
	self.poseY = data.pose.pose.position.y


    def VoxelCenter(self,data):
	for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
		self.voxelPts.append([p[0], p[1], p[2]])


    def ScanCloud(self,data):
	for p in point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
		self.scanPts.append([self.poseX + p[0], self.poseY + p[1]])
    	

    def sendNavCb(self,data):
        self.sample = []
        result = data
        frontier_mat = []
        if result.status.text == "Goal reached.":
            if self.flagg == 1 and self.init == 1:
		self.pubScanLocation = 0                
		self.flagg = 0
                rospy.loginfo('Goal reached')
                pnts = self.grid.data
                gpnts = np.array(self.newcost).flatten().tolist()
                cnt = 0
                pic_mat = []

                for x in range(self.map_height):
                    pic_x = []
                    for y in range(self.map_width):
                        p = pnts[cnt]
                        g = gpnts[cnt]

                        if (g>0):
                            p = 0
                        else:
                            if (p ==-1):
                                p = 50
                            elif (p == 0):
                                p = 100
                            else:
                                p = 0
                        pic_x.append(p)
                        cnt += 1
                    pic_mat.append(pic_x)

                ox = self.grid.info.origin.position.x
                oy = self.grid.info.origin.position.y
                mc = interp1d([ox,ox+self.map_width*self.map_res],[0,self.map_width],bounds_error=False)
                mr = interp1d([oy,oy+self.map_height*self.map_res],[0,self.map_height],bounds_error=False)

                r = mr(self.feedback.feedback.base_position.pose.position.y)
                c = mc(self.feedback.feedback.base_position.pose.position.x)
#                rospy.loginfo((int(c),int(r)))

                # plt.imshow(pic_mat,cmap='gray',origin='lower')
                # plt.draw()
                # plt.pause(0.001)

                mw = interp1d([0,self.map_width],[ox,ox+self.map_width*self.map_res],bounds_error=False)
                mh = interp1d([0,self.map_height],[oy,oy+self.map_height*self.map_res],bounds_error=False)

                frontier_pnts = findFrontier(pic_mat)
		xy = []
		for i in range(len(frontier_pnts)):
			xy.append([mw(frontier_pnts[i][0]),mw(frontier_pnts[i][1])])
 		self.publishMarker(xy)

                if len(frontier_pnts) > 0:
                    idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
                    nextpnt = frontier_pnts[idx]
                    pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]
                    if self.pntlist:
                        for i in range(len(self.pntlist)):
                            if calc_distance(pnt1,self.pntlist[i]) < 5:
                                frontier_pnts.remove(nextpnt)
                                if len(frontier_pnts) > 0:
                                    idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
                                    nextpnt = frontier_pnts[idx]
                                    pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]
                                    rospy.loginfo('sending number '+str(i+2) + ' in list')
                                else:
                                    rospy.loginfo('Mapping Done')
                                    self.send = False

                    if self.isstuck == False and self.prevpnt != -1:
                        if calc_distance(nextpnt,self.prevpnt) < 5:
                            rospy.loginfo('repeated point')
			    frontier_pnts.remove(nextpnt)
                            if len(frontier_pnts) > 0:
                                idx = find_closest_frontier([int(c),int(r)],frontier_pnts,self.heading)
                                nextpnt = frontier_pnts[idx]
                                rospy.loginfo('Sent second closest')
                            else:
                                self.send = False
                                rospy.loginfo('Mapping Done')

                    if self.send == True:
#                        rospy.loginfo([nextpnt,self.prevpnt])
                        if self.prevpnt == 0:
                            self.checkpnt+=1
                            rospy.loginfo('checkpoint increased')
#                            rospy.loginfo(self.checkpnt)
                        elif self.prevpnt == 1:
                            rospy.loginfo('wait to change point')
                        else:
                            self.checkpnt = 0

                        self.prevpnt = nextpnt
			self.scaledprevpnt = self.nextpnt
                        self.nextpnt = [mw(nextpnt[0]),mh(nextpnt[1])]
                        self.sendGoal(self.nextpnt)
                        self.isstuck = False
#			print [nextpnt[0] * self.map_res + self.map_origX, nextpnt[1] * self.map_res + self.map_origY]
#			print 'distance from previous position is ' + str(calc_distance(self.nextpnt,self.scaledprevpnt))

                else:
                    rospy.loginfo('No prontier points. Mapping Done!!')
		    self.sendGoal([0,0])

    def sendGoal(self,nextpnt):
	rospy.loginfo('Next point is ' + str(nextpnt[0]) + ', ' + str(nextpnt[1]))
	q = quaternion_from_euler(0,0,0,'sxyz')
        newPose.header.stamp = rospy.Time.now()
        newPose.header.frame_id = "map"
        newPose.pose.position.x = nextpnt[0]
        newPose.pose.position.y = nextpnt[1]
        newPose.pose.orientation.x = q[0]
        newPose.pose.orientation.y = q[1]
        newPose.pose.orientation.z = q[2]
        newPose.pose.orientation.w = q[3]
        self.pub.publish(newPose)
#	print 'Area is ' + str(self.area)
#	print 'num of unknown cell is ' + str(self.num_unknown)
	

    def publishMarker(self,pnt):
        newMarker = Marker()
        newMarker.header.frame_id = "/map"
        newMarker.header.stamp = rospy.Time.now()
        newMarker.ns = "points"
        newMarker.id = 0
        newMarker.type = newMarker.POINTS #SPHERE #LINE_STRIP
        newMarker.action = newMarker.ADD
        newMarker.pose.orientation.w = 1
        newMarker.scale.x = 0.3 #0.05
        newMarker.scale.y = 0.3
	newMarker.scale.z = 0.3
	newMarker.color.r = 1.0
	newMarker.color.g = 0.0
	newMarker.color.b = 0.0
	newMarker.color.a = 1.0
	for j in range(0,len(pnt)):
            newMarker.points.append(Point(pnt[j][0], pnt[j][1], 0))

        self.line_pub.publish(newMarker)
	rospy.loginfo(str(len(pnt)) + ' frontier points marker published!!')


def findFrontier(mat):

    frontier_pnts = []
    # frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    dx = [0,-1,-1,-1,0,1,1,1]
    dy = [1,1,0,-1,-1,-1,0,1]

    frontier_mat = np.array(mat).astype(np.uint8)
    frontier_mat = cv2.Canny(frontier_mat,100,200)
    # plt.imshow(frontier_mat,cmap='gray',origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    free_pnts = np.asarray(np.where(frontier_mat==255)).T.tolist()
    frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    row, col = np.shape(mat)
    for j in range(len(free_pnts)):
        r,c = free_pnts[j]
        if mat[r][c] == 100:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 50:
                        frontier_mat[r][c] = 255
                        break
        elif mat[r][c] == 50:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 100:
                        frontier_mat[r][c] = 255
                        break
    # plt.imshow(frontier_mat,cmap='gray',origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    frontmat = np.array(frontier_mat).astype(np.uint8)
    kernel = np.ones((5,5), np.uint8)*255
    frontmat = cv2.dilate(frontmat,kernel,iterations=3)
    contour = cv2.findContours(frontmat.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(contour) > 0:
        res = np.array(mat).astype(np.uint8)
        for i in range(len(contour)):
            c = contour[i]
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if radius > 15: #25:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                frontier_pnts.append(center)
                res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),75,2)

    return frontier_pnts


def calc_distance(pnt1,pnt2):
    x1,y1 = pnt1
    x2,y2 = pnt2
    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist


def find_nearest_pts(node,nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist = np.sqrt(np.sum((nodes - node)**2, axis=1))
    return np.argmin(dist)


def find_closest_frontier(node,nodes,heading):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    diff = nodes - node
    dist_2 = np.sum(diff**2, axis=1)
    distance = np.sqrt(dist_2)
    ang = np.arctan2(diff[:,1],diff[:,0])
    ang_diff = abs(ang-heading)**2
    return np.argmin(distance*ang_diff)
#    return np.argmin(dist_2)

if __name__ == "__main__":
    rospy.init_node('explore') #make node
    rospy.sleep(1)
    gc = jackal_explore()
    rospy.sleep(1)
    gc.sendGoal([0.0,0.0])
    rospy.sleep(0.5)
    # plt.figure()
    # plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
