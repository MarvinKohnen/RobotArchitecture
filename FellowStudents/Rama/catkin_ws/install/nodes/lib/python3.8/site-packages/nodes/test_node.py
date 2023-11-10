#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
import numpy as np
import math


depth_data = []
counter = 0
INF = math.inf

#predictions = []

# class Test(Node):
#     def __init__(self):
#         super().__init__('test_node')
#         print("node initiated")
#         self.subscription = self.create_subscription(PointCloud2, "/depth_camera/points", self.callback, 10)
        

#     def callback(self,msg):
#         print("entered callback")
#         global depth_data
#         depth_data = msg.data
#         print("message data : ", msg.data)
#         print("depth data ", depth_data)


class KalmanFilter(Node):

    dt = 1.0/60
    F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    H = np.array([1, 0, 0]).reshape(1, 3)
    Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
    R = np.array([0.05]).reshape(1, 1)
    n = F.shape[1]
    m = H.shape[1]
    x = 0
    B = 0
    P = np.eye(n)

    def __init__(self):
        super().__init__('kf_depthCamera_node')
        print("node initiated")
        #self.subscription = self.create_subscription(PointCloud2, "/depth_camera/points", self.callback, 10)
        self.subscription = self.create_subscription(LaserScan, "/scan", self.callback, 10)
        
        # if(F is None or H is None):
        #     raise ValueError("Set proper system dynamics.")
        # self.n = F.shape[1]
        # self.m = H.shape[1]
        # self.F = F
        # self.H = H
        # self.B = 0 if B is None else B
        # self.Q = np.eye(self.n) if Q is None else Q
        # self.R = np.eye(self.n) if R is None else R
        # self.P = np.eye(self.n) if P is None else P
        # self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        #print("entered predict")
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        #print("entered update")
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        

    def example(self):
        print("entered example function")
        measurements = depth_data
        
        predictions = []
        print("len measurements : ", len(measurements))
        print("len depth data ", len(depth_data))
        print("depth data ", depth_data)
        for z in measurements:
            if z == INF:
                continue
            else:
                #print("z : ", str(z))
                predictions.append(np.dot(self.H,  self.predict())[0])
                self.update(z)
        print("-------------------------------------------------------------------")
        print("end of for predictions : ", predictions)
        print("length of pred : ", len(predictions))

    def callback(self,msg):
        global counter
        global depth_data
        print("entered callback"+ str(counter))
        counter = counter+1
        depth_data = msg.ranges
        print("len msg ranges ", len(msg.ranges))
        self.example()
        


def main(args=None):

    print("kf started")
    rclpy.init(args=args)
    #test = Test()
    kf = KalmanFilter()
    
    rclpy.spin(kf)
    kf.destroy_node()
    rclpy.shutdown()