#!/usr/bin/env python

import numpy as np 
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
mu = 0.4
sigma = 0.04
depth_data = []

#F : the state transition model
#H : the observation model
#Q : covariance of the process noise
#R : covariance of the observartion noise.
#P : Predicted apriori estimated covariance.
#K : optimal kalman gain.


class KalmanFilter(Node):
    global depth_data
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
        
        super().__init__('kf_depthCamera_node')
        
        self.subscription = self.create_subscription(PointCloud2, "/depth_camera/points", self.callback, 10)
        
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")
        self.n = F.shape[1]
        self.m = H.shape[1]
        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    #u is the mean mu
    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

    def example():
        dt = 1.0/60
        F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
        H = np.array([1, 0, 0]).reshape(1, 3)
        Q = np.array([[0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.0, 0.0, 0.0]])
        R = np.array([0.5]).reshape(1, 1)

        #x needs to be set to zero
        x = np.linspace(-10, 10, 100)
        #measurements are added and noised by a random number which is not needed.
        #measurements should be taken from sensor data.
        measurements = depth_data

        kf = KalmanFilter(F = F, H = H, Q = Q, R = R)
        predictions = []

        for z in measurements:
            predictions.append(np.dot(H,  kf.predict())[0])
            kf.update(z)

        
        #plt.plot(range(len(measurements)), measurements, label = 'Measurements')
        #plt.plot(range(len(predictions)), np.array(predictions), label = 'Kalman Filter Prediction')
        #plt.legend()
        #plt.show()

    def callback(self,msg):
        print("entered callback")
        depth_data = msg.data
        print("message data : ", msg.data)
        print("depth data ", depth_data) 


# if __name__ == "__main__":
#     print("working")
        #rospy.init('kalman_filter',anonymous=True)
        #rospy.Subscriber("/depth_camera/points", PointCloud2, callback)
    #    example()

def main(args=None):
    print("node started")
    rclpy.init(args=args)
    kf = KalmanFilter()
    rclpy.spin(kf)
    test.destroy_node()
    rclpy.shutdown()

# if __name__ == "__main__":
#     main()


    
    