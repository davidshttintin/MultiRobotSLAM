"""
@author: Andrius Bernatavicius, 2019

"""
from lib.algorithms import RMHC_SLAM
from lib.sensors import Laser
import numpy as np
import settings
from collections import deque
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import skimage.io as skio
from lib.frontier import *
from lib.pathplanner import lee_planning_path


def color(grid, point, c):
    for i in range(point[0]-15, point[0]+15):
        grid[i][point[1]] = c
    for i in range(point[1]-15, point[1]+15):
        grid[point[0]][i] = c
    return grid

class Controller(object):
    def __init__(self):
        self._ring_buff_capacity = 3
        self._ring_buff = deque([], self._ring_buff_capacity)
    
    def world2robot(self, x, y, theta, point):
        pt = np.array([point[0], point[1], 1])
        G = np.zeros((3, 3))
        G[0][0] = np.cos(theta)
        G[0][1] = -np.sin(theta)
        G[1][0] = np.sin(theta)
        G[1][1] = np.cos(theta)
        G[0][2] = x
        G[1][2] = y
        G[2][2] = 1
        pt2 = np.matmul(np.linalg.inv(G), pt)
        return np.array([pt[0], pt[1]])
    
    # param: current and target position
    # return: velocity for left and right motor
    def p_control(self, current, target):

        e = np.array(target) - np.array(current)
        K1 = 1
        K2 = 1
        K = np.array([[K1, 0], [0, K2]])
        v = np.matmul(K, e)
        theta_dot = np.arctan(v[1]/v[0])
        left = 0.5 * (v[0] + theta_dot)
        right = 0.5 * (v[0] - theta_dot)
        ratio = float(right/left)
        vl = min(1, left)
        vr = ratio * vl
        print(target, current, e, vl, vr)
        return np.array([vl, vr])

    def compute_vel(self, pos, theta, target):
        current_position = [0 + pos[0], 0 + pos[1]] 
        return self.p_control(current_position, self.world2robot(pos[0], pos[1], theta, target))
        

class Pioneer(object):
    def __init__(self, env, controller):
        
        self.env = env
        self.controller = controller
        self.display = None
        
        # Lidar
        self.lidar_names   = ['SICK_TiM310_sensor1', 'SICK_TiM310_sensor2']
        for _ in range(10):
            self.lidar_handles = [self.env.get_handle(x) for x in self.lidar_names]
        self.lidar_data    = []
        self.lidar_angle   = 270.
        self.lidar_points  = 134
        self.lidar         = Laser(self.lidar_points, 2, self.lidar_angle, 10, 2, 50)
        self.scale_factor  = 1000
        # SLAM
        self.slam_engine = RMHC_SLAM(self.lidar, settings.image_size, settings.map_size)

        # Motors
        self.motor_names   = ['Pioneer_p3dx_leftMotor', 'Pioneer_p3dx_rightMotor']
        self.motor_handles = [self.env.get_handle(x) for x in self.motor_names]

        # Positions and angles
        self.angular_velocity = np.zeros(2)
        self.angles = np.zeros(2)
        self.theta  = 0
        self.pos    = [0, 0]
        self.position_history = deque(maxlen=100)

        self.wall_l = False
        self.wall_r = False

        self.change_velocity([0, 0])
        
        self.current_target = ()


    def find_closest(self):
        """
        Returns [[angle_l, angle_r], [distance_l, distance_r]] of the closest objects in the left and right visual fields
        """
        sensor_scan_angle = self.lidar_angle / self.lidar_points
        left  = self.lidar_data[0:68]
        right = self.lidar_data[68:]
        min_left  = min(left)
        min_right = min(right)
        ind_left  = [i for i, x in enumerate(left) if x == min_left][0]
        ind_right = [i for i, x in enumerate(right) if x == min_right][0]
        angle_l = -self.lidar_angle / 2 + ind_left * sensor_scan_angle
        angle_r = 0 + ind_right * sensor_scan_angle
        
        return [[angle_l, angle_r], [min_left, min_right]]

    def angle_closest(self):
        return self.find_closest()[0]

    def distance_closest(self):
        return self.find_closest()[1]

    def wall_left(self, threshold):
        
        if sum(self.lidar_data[47:67]) < threshold*60:
            self.wall_l = True
        else:
            self.wall_l = False
        return self.wall_l

    def wall_right(self, threshold):
        if sum(self.lidar_data[67:87]) < threshold*60:
            self.wall_r = True
        else:
            self.wall_r = False
        return self.wall_r


    def read_lidars(self, mode='blocking'):
        """
        Read the vision sensor in VREP
        """
        self.lidar_data = []
        lidar_data = [self.env.read_lidar(x, mode)[2][1] for x in self.lidar_handles]
        self.lidar_data = lidar_data[0][1::2][0::4] + lidar_data[1][1::2][0::4]
        del self.lidar_data[68]
        del self.lidar_data[0]
        return self.lidar_data

    def slam(self, bytearray):
        """
        Get input from sensors and perform SLAM
        """

        # Mapping
        scan = self.read_lidars()
        scan = list(np.array(scan) * self.scale_factor)
        self.slam_engine.update(scan)
        self.slam_engine.getmap(bytearray) # Draw current map on the bytearray

        # Localization
        x, y, theta = self.slam_engine.getpos()
        self.pos[0] = int(x / float(settings.map_size * 1000) * settings.image_size)
        self.pos[1] = int(y / float(settings.map_size * 1000) * settings.image_size)
        self.position_history.append(tuple(self.pos)) # Append the position deque with a tuple of (x,y)
        self.theta = theta


    def current_speed(self):
        prev_angles = np.copy(self.angles)
        self.angles = np.array([self.env.get_joint_angle(x) for x in self.motor_handles])
        angular_velocity = self.angles - prev_angles
        for i, v in enumerate(angular_velocity):
            # In case radians reset to 0
            if v < -np.pi:
                angular_velocity[i] =  np.pi*2 + angular_velocity[i]
            if v > np.pi:
                angular_velocity[i] = -np.pi*2 + angular_velocity[i]
        self.angular_velocity = angular_velocity
        return self.angular_velocity

    def change_velocity(self, velocities, target=None):
        # Change the current velocity of the robot's wheels
        if target == 'left':
            self.env.set_target_velocity(self.motor_handles[0], velocities)
        if target == 'right':
            self.env.set_target_velocity(self.motor_handles[1], velocities)
        else:
            for i in range(2):
                self.env.set_target_velocity(self.motor_handles[i], velocities[i])

    def drive_to_target(self):
        array = np.frombuffer(self.display.bytearray, dtype=np.uint8)
        gray  = np.reshape(array, [settings.image_size, settings.image_size])
        gray = color(gray, self.pos, 100)
        gray = color(gray, self.current_target, 100)
        skio.imsave("driving.jpg", gray)
        vels = self.controller.compute_vel(self.pos, self.theta, self.current_target)
        self.change_velocity(vels)

class Display(object):
    def __init__(self, agent, wall):
        self.bytearray = bytearray(settings.image_size*settings.image_size)
        self.grid = np.array([])
        self.agent = agent
        self.agent.display = self
        self.im = None
        self.colormap = cv2.COLORMAP_OCEAN
        self.visited = np.ones([settings.image_size, settings.image_size, 3])

        self.centroids = []

        self.wall_disp = wall
        # Agent parameters
        self.agent_radius    = 10
        self.agent_color     = (100, 100, 100)

        # Speed display
        self.speed_location  = [(60,700), (90,700)]
        self.step = 0
        # Create cv2 window
        print("before")
        cv2.namedWindow('Simultaneous Localization and Mapping (SLAM)', cv2.WINDOW_NORMAL)
        print("after")
        cv2.resizeWindow('Simultaneous Localization and Mapping (SLAM)', 700, 700)

    def update(self):
        """
        Updates the current display based on current information of the agent
        """

        if self.step % settings.steps_slam == 0:
            self.agent.slam(self.bytearray)
        
        array = np.frombuffer(self.bytearray, dtype=np.uint8)
        self.grid = np.reshape(array, [settings.image_size, settings.image_size])
        

        self.im = self.to_image()
        self.draw_agent(self.im)      
        
        # self.im = cv2.cvtColor(cv2.Canny(self.im, 100,150), cv2.COLOR_GRAY2RGB)
        self.draw_closest(self.im)
        self.draw_trajectory(self.im)
        self.draw_frontier_centroids(self.im)
        

        self.im = cv2.flip(self.im, 0)
        self.draw_elements(self.im)        
        self.draw_speed(self.im)
        
        # im = cv2.filter2D(im,-1,np.ones((5,5),np.float32)/25)
        self.im = cv2.blur(self.im,(3,3))
        cv2.imshow('Simultaneous Localization and Mapping (SLAM)', self.im)
        key = cv2.waitKey(1) & 0xFF
        self.step += 1

        array = np.frombuffer(self.bytearray, dtype=np.uint8)
        gray  = np.reshape(array, [settings.image_size, settings.image_size])
        #print([x for x in array if x!= 127])
        # skio.imsave("a.jpg", gray)
        
        # planning part
        if self.step % settings.steps_lee == 0 and len(self.centroids) != 0:
            print("start path planning")
            print("agent pos:", self.agent.pos)
            print("target:", self.centroids[0])
            array = np.frombuffer(self.bytearray, dtype=np.uint8)
            grid = np.reshape(array, [settings.image_size, settings.image_size])
            obst = preprocess_grid(grid)
            obst = grow_obstacle(obst)
            waypoints = lee_planning_path(obst, (self.agent.pos[0], self.agent.pos[1]), self.centroids[0])
            print("end path planning")

            """debug"""
            if len(waypoints) < 1:
                colored = color(obst, (self.agent.pos[0], self.agent.pos[1]), 100)
                colored = color(colored, self.centroids[0], 100)
                skio.imsave("error.jpg", colored)
            self.agent.current_target = waypoints[0]

    def draw_closest(self, image):
        """
        Draws lines to the closest objects right and left
        """
        angles, distances = self.agent.find_closest()
        scale = 30
        x = self.agent.pos[0] + int(np.cos((self.agent.theta+angles[0])*np.pi/180) * distances[0]*scale) 
        y = self.agent.pos[1] + int(np.sin((self.agent.theta+angles[0])*np.pi/180) * distances[0]*scale)
        cv2.line(image, (x, y), tuple(self.agent.pos), (150,150,255), 1)
        x = self.agent.pos[0] + int(np.cos((self.agent.theta+angles[1])*np.pi/180) * distances[1]*scale) 
        y = self.agent.pos[1] + int(np.sin((self.agent.theta+angles[1])*np.pi/180) * distances[1]*scale)
        cv2.line(image, (x, y), tuple(self.agent.pos), (150,150,255), 1)


    def draw_trajectory(self, image):
        for i in range(1, len(self.agent.position_history)):
            color = (255-i*5,255-i*5,255-i*5)
            cv2.line(image, self.agent.position_history[i-1], self.agent.position_history[i], color, 1)

    def to_image(self):
        array = np.frombuffer(self.bytearray, dtype=np.uint8)
        gray  = np.reshape(array, [settings.image_size, settings.image_size])
        color = cv2.applyColorMap(gray, self.colormap)
        return color

    def draw_agent(self, image):
        """
        Draws the agent in the map
        """
        cv2.circle(image, (self.agent.pos[0], self.agent.pos[1]), self.agent_radius, self.agent_color, cv2.FILLED)
        # Coordinates of the angle indicator
        x = self.agent.pos[0] + int(np.cos(self.agent.theta*np.pi/180) * 15) 
        y = self.agent.pos[1] + int(np.sin(self.agent.theta*np.pi/180) * 15)
        
        cv2.circle(self.visited, (self.agent.pos[0], self.agent.pos[1]), self.agent_radius, (2,2,2), cv2.FILLED)
        self.draw_line_at_angle(image, 15, 0, (150,150,150))
        self.draw_line_at_angle(image, 15, -135, (150,150,150))
        self.draw_line_at_angle(image, 15, 135, (150,150,150))

    
    def draw_line_at_angle(self, im, length, angle, color):
        x = self.agent.pos[0] + int(np.cos((self.agent.theta+angle)*np.pi/180) * length) 
        y = self.agent.pos[1] + int(np.sin((self.agent.theta+angle)*np.pi/180) * length)
        cv2.line(im, (x, y), tuple(self.agent.pos), (150,150,150), 1)
    
    def draw_speed(self, im=None):
        """
        Draws a small graph of the current speed
        """
        origin = self.speed_location
        off = 500
        if im is None:
            im = self.to_image()
        sp = self.agent.current_speed()
        im=cv2.rectangle(im, origin[0], (origin[0][0]+20, origin[0][1]-int(sp[0]*150)), (240, 150, 150), cv2.FILLED)
        im=cv2.rectangle(im, origin[1], (origin[1][0]+20, origin[1][1]-int(sp[1]*150)), (240, 150, 150), cv2.FILLED)
        cv2.line(im, (50,off+200), (120,off+200), (150,150,150), 1)
        cv2.line(im, (50,off+240), (120,off+240), (150,150,150), 1)
        cv2.line(im, (50,off+160), (120,off+160), (150,150,150), 1)

    def draw_elements(self, img):
        off = 500
        cv2.rectangle(img, (0, off+120), (280,off+300), (150,150,150),cv2.FILLED)
        cv2.rectangle(img, (50,off+140), (120, off+260),(255,255,255), cv2.FILLED)
        cv2.rectangle(img, (50,off+140), (120, off+260),(0,0,0), 1)
        # cv2.line(img, (0,off+120), (800,off+120), (0,0,0),1)
        cv2.putText(img, " L  R", (60,off+155), cv2.FONT_HERSHEY_PLAIN,1, (0,0,0), 1)
        # cv2.putText(img, "Simultaneous Localization and Mapping",(100,30), cv2.QT_FONT_NORMAL,1, (0,0,0), 1)
        cv2.putText(img, "0", (30,off+205), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0), 1)
        cv2.putText(img, "-5", (22,off+245), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0), 1)
        cv2.putText(img, "5", (30,off+165), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0), 1)
        cv2.putText(img, "Step: {}".format(self.step), (150,off+150), cv2.FONT_HERSHEY_PLAIN, (1), (0,0,0),1)
        if self.wall_disp:
            cv2.putText(img, "Wall L: {}".format(self.agent.wall_l), (150,off+175), cv2.FONT_HERSHEY_PLAIN, (1), (0,0,0),1)
            cv2.putText(img, "Wall R: {}".format(self.agent.wall_r), (150,off+200), cv2.FONT_HERSHEY_PLAIN, (1), (0,0,0),1)
        

    def draw_sensor_data(self,im):
        data1 = self.agent.read_lidars()
        data = np.array([data1 for _ in range(20)])
        data *= (255.0/data.max())
        
        offsety = 600
        offsetx = 200
        for i in range(100):
            im[offsetx:offsetx+20,offsety:offsety+len(data1), 0] = data

    def draw_frontier_centroids(self, img):
        cx, cy, clusters = frontier_cluster(self.grid)
        if not cx.size or not cy.size or not clusters.size:
            return
        # draw frontier pts        
        for i in range(len(cx)):
            cv2.drawMarker(img, (int(cy[i]), int(cx[i])), (0,0,255), markerSize=5)
        # draw centroids        
        for gid in range(1, max(clusters)+1):
            print(gid)
            ccopy = np.copy(clusters)
            ccopy[ccopy != gid] = 0
            mask = np.nonzero(ccopy)
            if mask[0].size == 0:
                continue
            xcenter = np.mean(cx[mask])
            ycenter = np.mean(cy[mask])
            self.centroids.append((int(xcenter), int(ycenter)))
            cv2.drawMarker(img, (int(ycenter), int(xcenter)), (0,255,255), markerSize=25, markerType=cv2.MARKER_STAR)
            

        
