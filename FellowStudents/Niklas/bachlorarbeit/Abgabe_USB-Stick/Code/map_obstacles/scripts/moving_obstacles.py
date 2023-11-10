import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np


CENTRAL_POSITION_OBSTACLES = np.array((0,0))
BOX_WIDTH = 5
BOX_HEIGHT = 5


class Moving_Cylinder():
    """Class to controll a ball in the simulation."""
    def __init__(self, name):
        self.name = name
        self.update_count = 0

        global OBSTACLE_VELOCITY
        random_vector = np.random.randint(-10,10, size=2)
        self.moving_direction = random_vector / np.linalg.norm(random_vector) * OBSTACLE_VELOCITY
    
    def get_pub_msg(self, model_pose) -> ModelState:
        """Returns a message object containing the next movement of the Ball."""
        obstacle_msg = ModelState()
        obstacle_msg.model_name = self.name
        obstacle_msg.pose = model_pose

        # Hindernisse sollen nicht hinfallen
        obstacle_msg.pose.orientation.x = 0
        obstacle_msg.pose.orientation.y = 0
        obstacle_msg.pose.orientation.z = 0
        obstacle_msg.pose.orientation.w = 0

        self._wall_bounds_adjustments(model_pose.position.x, model_pose.position.y)

        obstacle_msg.twist.linear.x = self.moving_direction[0]
        obstacle_msg.twist.linear.y = self.moving_direction[1]

        return obstacle_msg
    
    def _wall_bounds_adjustments(self, x, y):
        """Checks if the Ball hits the virtual Border and adjusts its momentum."""
        moving_back_to_center = np.dot(self.moving_direction, CENTRAL_POSITION_OBSTACLES - np.array((x,y))) > 0
        if (x > CENTRAL_POSITION_OBSTACLES[0] + BOX_WIDTH or x < CENTRAL_POSITION_OBSTACLES[0] - BOX_WIDTH) and not moving_back_to_center:
            self.moving_direction[0] *= -1
        if (y > CENTRAL_POSITION_OBSTACLES[1] + BOX_HEIGHT or y < CENTRAL_POSITION_OBSTACLES[1] - BOX_HEIGHT) and not moving_back_to_center:
            self.moving_direction[1] *= -1

class Moving():
    """Class to controll the moving obstacles."""
    moving_Cylinders: dict
    pub_model: rospy.Publisher

    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.init_already_done = False
        rospy.Subscriber('gazebo/model_states', ModelStates, self.moving)


    def moving(self, model: ModelState):
        """Publishes every movement of the ball-obstacles."""
        if not self.init_already_done:
            self.moving_Cylinders = {}
            for i in range(len(model.name)):
                if model.name[i].startswith('cylinder_clone'):
                    self.moving_Cylinders[model.name[i]] = Moving_Cylinder(model.name[i])
            self.init_already_done = True
        for i in range(len(model.name)):
            if model.name[i] not in self.moving_Cylinders: continue

            self.pub_model.publish(self.moving_Cylinders[model.name[i]].get_pub_msg(model.pose[i]))


def get_velocity_obstacles_param():
    try:
        global OBSTACLE_VELOCITY
        if rospy.has_param('/map_obstacles/obstacle_velocity'):
            OBSTACLE_VELOCITY = float(rospy.get_param('/map_obstacles/obstacle_velocity'))
        else:
            OBSTACLE_VELOCITY = 1
    except rospy.ROSInterruptException:
        rospy.loginfo('-> ERROR - Map_Obstacle.py - obstacle_velocity Parameter')


def main():
    rospy.init_node('moving_obstacle')
    moving = Moving()
    rospy.spin()

if __name__ == '__main__':
    get_velocity_obstacles_param()
    main()