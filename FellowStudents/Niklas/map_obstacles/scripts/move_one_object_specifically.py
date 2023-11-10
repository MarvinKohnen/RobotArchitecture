import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates


# rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'turtlebot3'
# pose:                                                                     
#   position:
#     x: 0.0
#     y: 0.0
#     z: 0.0"


# so oder so ähnlich

def set_correct_positions_robot():
  # Robot:
  msg = ModelState()
  msg.model_name = 'turtlebot3'
  msg.pose.position.x = -5
  msg.pose.position.y = 0
  msg.pose.position.z = -0.001004

  # msg.pose.
  msg.twist.linear.x = 1

  pub.publish(msg)


def set_correct_positions_obstacle():
  # Obstacle:
  msg = ModelState()
  msg.model_name = 'cylinder_clone1'

  # msg.pose.position.x = 8
  # msg.pose.position.y = 0
  # msg.twist.linear.x = -1
  # msg.twist.linear.y = 0

  msg.pose.position.x = 5.66
  msg.pose.position.y = -5.66
  msg.twist.linear.x = -1/np.sqrt(2)
  msg.twist.linear.y = 1/np.sqrt(2)

  # msg.pose.position.x = 0
  # msg.pose.position.y = -8
  # msg.twist.linear.x = 0
  # msg.twist.linear.y = 1

  pub.publish(msg)



counter = 0
def move_obstacle(te: rospy.timer.TimerEvent):
  global counter

  if counter == 0:
    set_correct_positions_robot()
    set_correct_positions_obstacle()
  counter += 1
  # obstacle movement
  return 



if __name__ == "__main__":
  rospy.init_node("specific_obstacler_mover", anonymous=False)
  pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=2)

  # set_correct_positions()
  timer = rospy.Timer(rospy.Duration(0.5), move_obstacle)
  rospy.spin()