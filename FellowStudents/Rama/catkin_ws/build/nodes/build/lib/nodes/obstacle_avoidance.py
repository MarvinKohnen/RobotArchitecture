#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import logging

SAFE_DISTANCE_LIMIT = 0.4
DETECTED_OBSTACLE_LEFT = None
DETECTED_OBSTACLE_FRONT = None
DETECTED_OBSTACLE_RIGHT = None
DETECTED_OBSTACLE_BACK = None
STATE = "OBSTACLE_AVOIDANCE_BEGIN"

# we can only detect the obstacles that are as high or higher than the lidar height.
#seperated the front of the robot to 3 parts :
#front left - front middle - front right

#from distance 0.7 in range[0] the range to be observed is 70 (between 324 and 34)
#from distance 0.38 in range [0] the range to be observed is 101 (between 308 and 50)

class LaserData(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.subscription = self.create_subscription(LaserScan, "/scan", self.check_obstacle, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pub_state = self.create_publisher(String, '/State_machine', 10)
        self.sub_state = self.create_subscription(String, "/State_machine", self.manage_states, 10)

        logger = self.get_logger()
        logger.set_level(logging.INFO)

    def switch(self):

        global DIRECTION

        if DIRECTION == "LEFT":
            DIRECTION = "RIGHT"
            return
        if DIRECTION == "RIGHT":
            DIRECTION = "LEFT"
            return

        print("switched")
        
        

    def move(self, command):

        twist = Twist()

        if command == "rotate_right":
            twist.linear.x = 0.0
            twist.angular.z= -0.4
            
        elif command == "forward":
            twist.angular.z = 0.0
            twist.linear.x = 0.4
            
        elif command == "rotate_left":
            twist.angular.z = 0.4
            twist.linear.x = 0.0
        
        elif command == "stop":
            twist.angular.z = 0.0
            twist.linear.x = 0.0

        elif command == "backward":
            twist.angular.z = 0.0
            twist.linear.x = -0.4
        
        self.publisher.publish(twist)


    def check_obstacle(self, msg):

        global DETECTED_OBSTACLE_BACK
        global DETECTED_OBSTACLE_FRONT
        global DETECTED_OBSTACLE_LEFT
        global DETECTED_OBSTACLE_RIGHT

        
        if STATE == "CHECK_FOR_OBSTACLES" or STATE == "OBSTACLE_AVOIDANCE_BEGIN":
            #self._logger.info("entered check obstacles" + STATE)

            depth_data = msg.ranges


            if depth_data != None:

                front = depth_data[342:359] + depth_data[0:16]
                back = depth_data[100:230]
                right = depth_data[20:40]
                left = depth_data[310:330]

                if min(front) <= SAFE_DISTANCE_LIMIT:
                    #self._logger.info("obstacle on the front side")
                    DETECTED_OBSTACLE_FRONT = True
                elif min(front) > SAFE_DISTANCE_LIMIT:
                    #self._logger.info("NO obstacle on the front side")
                    DETECTED_OBSTACLE_FRONT = None
                
                if min(back) <= SAFE_DISTANCE_LIMIT:
                    #self._logger.info("obstacle at the back")
                    DETECTED_OBSTACLE_BACK = True
                elif min(back) > SAFE_DISTANCE_LIMIT:
                    #self._logger.info("NO obstacle at the back ")
                    DETECTED_OBSTACLE_BACK = None

                if min(right) <= SAFE_DISTANCE_LIMIT:
                    #self._logger.info("obstacle on the right side")
                    DETECTED_OBSTACLE_RIGHT= True
                elif min(right) > SAFE_DISTANCE_LIMIT:
                    #self._logger.info("NO obstacle on the right side")
                    DETECTED_OBSTACLE_RIGHT= None

                if min(left) <= SAFE_DISTANCE_LIMIT:
                    #self._logger.info("obstacle on the left side")
                    DETECTED_OBSTACLE_LEFT = True
                elif min(left) > SAFE_DISTANCE_LIMIT:
                    #self._logger.info("NO obstacle on the left side")
                    DETECTED_OBSTACLE_LEFT = None

            if STATE == "OBSTACLE_AVOIDANCE_BEGIN":
                self.avoid_obstacles()

        

    def avoid_obstacles(self):

        global DETECTED_OBSTACLE_FRONT
        global DETECTED_OBSTACLE_LEFT
        global DETECTED_OBSTACLE_RIGHT

        if STATE == "OBSTACLE_AVOIDANCE_BEGIN":

            if DETECTED_OBSTACLE_LEFT == True or DETECTED_OBSTACLE_FRONT == True or DETECTED_OBSTACLE_RIGHT == True or DETECTED_OBSTACLE_BACK == True:  
                self.move("stop")
                self.move("rotate_right")
                print("Turning right")
            
            if DETECTED_OBSTACLE_LEFT == None and DETECTED_OBSTACLE_FRONT == None and DETECTED_OBSTACLE_RIGHT == None:
                self.move("forward")    #move forward

        


    def manage_states(self, msg_state):
        
        global STATE


        if msg_state.data == "OBSTACLE_AVOIDANCE_BEGIN":
            STATE = "OBSTACLE_AVOIDANCE_BEGIN"
            #self._logger.info("obstacle avoidance begin" + STATE)

        
        if msg_state.data == "RIGHT":
            STATE = "CHECK_FOR_OBSTACLES"
            #self.check_obstacle(msg_laser)
            if DETECTED_OBSTACLE_RIGHT == True:
                STATE = "OBSTACLE_AVOIDANCE_BEGIN"
                #self._logger.info("detected obstacle right after check" + str(DETECTED_OBSTACLE_RIGHT))
            elif DETECTED_OBSTACLE_RIGHT == None:
                self.move("stop")
                self.move("rotate_right")
                #self._logger.info(" no detected obstacle right after check" + str(DETECTED_OBSTACLE_RIGHT))

        if msg_state.data == "LEFT":
            STATE = "CHECK_FOR_OBSTACLES"
            #self.check_obstacle(msg_laser)
            if DETECTED_OBSTACLE_LEFT == True:
                STATE = "OBSTACLE_AVOIDANCE_BEGIN"
                #self._logger.info("detected obstacle left after check" + str(DETECTED_OBSTACLE_LEFT))
            elif DETECTED_OBSTACLE_LEFT == None:
                self.move("stop")
                self.move("rotate_left")
                #self._logger.info(" no detected obstacle left after check" + str(DETECTED_OBSTACLE_LEFT))


        if msg_state.data == "FORWARD":
            STATE = "CHECK_FOR_OBSTACLES"
            #self.check_obstacle(msg_laser)
            if DETECTED_OBSTACLE_FRONT == True:
                STATE = "OBSTACLE_AVOIDANCE_BEGIN"
                #self._logger.info("detected obstacle forward after check" + str(DETECTED_OBSTACLE_FRONT))
            elif DETECTED_OBSTACLE_FRONT == None:
                self.move("forward")
                #self._logger.info("NO detected obstacle forward after check" + str(DETECTED_OBSTACLE_FRONT))

        if msg_state.data == "BACKWARD":
            STATE = "CHECK_FOR_OBSTACLES"
            #self.check_obstacle(msg_laser)
            if DETECTED_OBSTACLE_BACK == True:
                STATE = "OBSTACLE_AVOIDANCE_BEGIN"
                #self._logger.info("detected obstacle backward after check" + str(DETECTED_OBSTACLE_BACK))
            elif DETECTED_OBSTACLE_BACK == None:
                self.move("backward")
                #self._logger.info("NO detected obstacle backward after check" + str(DETECTED_OBSTACLE_BACK))

        if msg_state.data == "STOP":
            STATE = "CHECK_FOR_OBSTACLES"
            #self.check_obstacle(msg_laser)
            if DETECTED_OBSTACLE_BACK == True or DETECTED_OBSTACLE_FRONT == True or DETECTED_OBSTACLE_LEFT == True or DETECTED_OBSTACLE_RIGHT == True:
                STATE = "OBSTACLE_AVOIDANCE_BEGIN"
                #self._logger.info("detected obstacle after check" + str(DETECTED_OBSTACLE_BACK))
            elif DETECTED_OBSTACLE_BACK == None:
                self.move("stop")
                #self._logger.info("NO detected obstacle after check" + str(DETECTED_OBSTACLE_BACK))




def main(args=None):
    rclpy.init(args=args)
    ls = LaserData()
    rclpy.spin(ls)

    ls.destroy_node()
    rclpy.shutdown()