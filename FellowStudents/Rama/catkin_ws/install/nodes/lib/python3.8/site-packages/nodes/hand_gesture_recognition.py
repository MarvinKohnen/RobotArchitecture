#!/usr/bin/env python


import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import mediapipe as mp
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String


STATE = "HAND_CONTROL"

class RealsenseCamera:
    def __init__(self):
        
        #pipeline for streaming data from the camera.
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)



    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None
        
        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        
        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        
        # Convert images to numpy arrays
        # distance = depth_frame.get_distance(int(50),int(50))
        # print("distance", distance)
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # cv2.imshow("Colormap", depth_colormap)
        # cv2.imshow("depth img", depth_image)

        return True, color_image, depth_image
    
    def release(self):
        self.pipeline.stop()



class handDetector(Node):

    def __init__(self, mode = False, maxHands = 1, detectionCon = 1, trackCon = 0.5):
        super().__init__('Handgesture_node')
        self.rate = self.create_rate(4)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pub_state = self.create_publisher(String, '/State_machine', 10)
        #self.sub_state = self.create_subscription(String, "/State_machine", self.manage_states, 10)
        
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
     
    def findHands(self,img, draw = True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img, self.results


    def findPosition(self, img, handNo = 0, draw = True):

        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)
        return lmlist


    '''
    function : orientation.
    parameter : coordinate_landmark_0 , coordinate_landmark_9.
    The function decides the orientation of the hand depending on the m-achse and
    x,y coordinates of the landmarks 0 [Wrist] and 9 [middle finger MCP].
    '''

    def orientation(self, img_results, img):

        msg = String()

        if img_results.multi_hand_landmarks != None:
            #print(" land mark 0 : ",img_results.multi_hand_landmarks[-1].landmark[0])
            coordinate_landmark_0 = img_results.multi_hand_landmarks[-1].landmark[0]
            coordinate_landmark_9 = img_results.multi_hand_landmarks[-1].landmark[9]
            coordinate_landmark_13 = img_results.multi_hand_landmarks[-1].landmark[13]
            coordinate_landmark_12 = img_results.multi_hand_landmarks[-1].landmark[12]
            coordinate_landmark_16 = img_results.multi_hand_landmarks[-1].landmark[16]

            coordinate_landmark_3 = img_results.multi_hand_landmarks[-1].landmark[3]
            coordinate_landmark_4 = img_results.multi_hand_landmarks[-1].landmark[4]
            
            x0 = coordinate_landmark_0.x
            y0 = coordinate_landmark_0.y

            x3 = coordinate_landmark_3.x
            y3 = coordinate_landmark_3.y

            x4 = coordinate_landmark_4.x
            y4 = coordinate_landmark_4.y

            
            x9 = coordinate_landmark_9.x
            y9 = coordinate_landmark_9.y

            x12 = coordinate_landmark_12.x
            y12 = coordinate_landmark_12.y

            x13 = coordinate_landmark_13.x
            y13 = coordinate_landmark_13.y

            x16 = coordinate_landmark_16.x
            y16 = coordinate_landmark_16.y
            
            if abs(x9 - x0) < 0.05:      #since tan(0) --> ∞
                m = 1000000000
            else:
                m = abs((y9 - y0)/(x9 - x0))       
                
            if m>=0 and m<=1:
                if x9 > x0:
                    msg.data = "LEFT"
                    cv2.putText(img, "Left", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
                    #self.pub_state.publish("RIGHT")
                    print("published left hand")
                    
                else:
                    msg.data = "RIGHT"
                    cv2.putText(img, "Right", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
                    #self.pub_state.publish("LEFT")
                    print("published right hand")
                    
            if m>1:

                if y9 < y12 and y13 < y16 and y4 < y3:
                    msg.data = "STOP"
                    cv2.putText(img, "STOP", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
                    #self.pub_state.publish("FORWARD")
                    print("published STOP hand")

                elif y9 < y0:       #since, y decreases upwards
                    msg.data = "FORWARD"
                    cv2.putText(img, "Forward", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
                    #self.pub_state.publish("FORWARD")
                    print("published forward hand")

                    
                
                else:
                    msg.data = "BACKWARD"
                    cv2.putText(img, "Backward", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
                    #self.pub_state.publish("BACKWARD")
                    print("published bachward hand")
                    

        else:
            msg.data = "OBSTACLE_AVOIDANCE_BEGIN"
            cv2.putText(img, "obstacle avoidance", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,(0,0,255),2)
            #self.pub_state.publish("OBSTACLE_AVOIDANCE_BEGIN")
        
        self.pub_state.publish(msg)
        

        


def main(args = None):

    pTime = 0
    cTime = 0
    rclpy.init(args=args)
    detector = handDetector()
    realsense = RealsenseCamera()

    
    while True:
        
        ret, color_frame, depth_frame = realsense.get_frame_stream()

        img, img_results = detector.findHands(color_frame)
        lmlist = detector.findPosition(img)
        #hand_label = detector.get_hand_label(img_results)
        #print("hand label : ", hand_label)
        hand_orientation = detector.orientation(img_results, img)
        #print("hand orientation: ", hand_orientation)
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime

        cv2.putText(color_frame,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)
        
        cv2.imshow("Image", color_frame)
        cv2.waitKey(1)





if __name__ == "__main__":
    main()