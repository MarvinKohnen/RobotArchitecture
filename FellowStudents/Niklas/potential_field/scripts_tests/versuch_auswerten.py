import time
import subprocess
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
from std_msgs.msg import Int8


GOAL_RANGE = 3
GOAL_REACHED = False
# GOAL = np.array((10, 0))

def count_collisions() -> int:
    command = ['gz','topic', '-e', '/gazebo/default/physics/contacts']
    p = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)
    collision_counter = 0
    STATE_READ = 0

    line1 = ''
    line2 = ''

    while not rospy.is_shutdown() and not GOAL_REACHED:
        line = p.stdout.readline()
        current_line = line.strip()
        
        if 'contact {' in current_line:
            STATE_READ = 1
            continue

        if STATE_READ > 0:
            if STATE_READ == 1:
                line1 = current_line
                STATE_READ += 1
            elif STATE_READ == 2:
                line2 = current_line
                if ('turtlebot3' in line1 and 'cylinder_clone' in line2) or ('turtlebot3' in line2 and 'cylinder_clone' in line1):
                    collision_counter += 1
                STATE_READ = 0
                line1 = ''
                line2 = ''   
            elif STATE_READ >2:
                print('ERROR')
    return collision_counter


def check_goal_reached_and_log_position(model: ModelStates):
    """GOAL ist bei GOAL_RANGE m Abstand erreicht"""
    for i in range(len(model.name)):
        if model.name[i].startswith('turtlebot'):
            current_point = np.array((model.pose[i].position.x,model.pose[i].position.y))
            approx_distance_travelled(current_point)
            if np.linalg.norm(GOAL - current_point) < GOAL_RANGE:
                global GOAL_REACHED
                GOAL_REACHED = True


last_point_logged = None
DISTANCE_TRAVELLED = 0
logged_points = None
def approx_distance_travelled(current_point: np.array):
    """Approximiert die bisherige zurückgelegte Distanz und speichert diese in DISTANCE_TRAVELLED"""
    global last_point_logged, DISTANCE_TRAVELLED, logged_points
    if last_point_logged is None:

        logged_points = open('/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/Points.txt','a')
        # logged_points = open('/home/nik/Documents/TARTW/src/bachlorarbeit/potential_field/Ergebnisse/Points.txt','a')

        

        last_point_logged = current_point
    else:
        DISTANCE_TRAVELLED += np.linalg.norm(last_point_logged - current_point)
        last_point_logged = current_point

        if not GOAL_REACHED:
            logged_points.write(f'{last_point_logged[0]} {last_point_logged[1]}\n')
        elif not logged_points.closed:
            logged_points.write(f'----------------------\n')
            logged_points.close()


def auswerten():
    pub = rospy.Publisher("/potentiel_field/test_finished", Int8, queue_size=1)
    rospy.Subscriber('gazebo/model_states', ModelStates, check_goal_reached_and_log_position)

    start_time = time.time()

    datei = open('/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/ErgebnissePotential_Field_Results.txt','a')
    # datei = open('/home/nik/Documents/TARTW/src/bachlorarbeit/potential_field/Ergebnisse/ErgebnissePotential_Field_Results.txt','a')

    anzahl_kollisionen = count_collisions() # SCHLEIFE BEACHTEN!!!
    test_summary = f'Versuch_{time.time_ns()} - Obstacle_Velocity: {OBSTACLE_VELOCITY} - Kollisionen: {anzahl_kollisionen} - Ziel erreicht: {GOAL_REACHED} - travelled_distance: {DISTANCE_TRAVELLED} - Time in Sek.: {time.time() - start_time}\n'
    pub.publish(Int8(0))
    datei.write(test_summary)
    datei.close()
    # rospy.signal_shutdown('Auswertung abgeschlossen')


def get_ros_params():
    try:
        global OBSTACLE_VELOCITY
        if rospy.has_param('/map_obstacles/obstacle_velocity'):
            OBSTACLE_VELOCITY = int(rospy.get_param('/map_obstacles/obstacle_velocity'))
        else:
            OBSTACLE_VELOCITY = None

        global GOAL
        if rospy.has_param('/potential_field/goal_x') and rospy.has_param('/potential_field/goal_y'):
            GOAL = np.array((float(rospy.get_param('/potential_field/goal_x')), float(rospy.get_param('/potential_field/goal_y'))))
        else:
            GOAL = np.array((10, 0))
            # GOAL = None

    except rospy.ROSInterruptException:
        rospy.loginfo('-> ERROR - Map_Obstacle.py - obstacle_velocity Parameter')


if __name__ == '__main__':
    get_ros_params()
    rospy.init_node("versuch_auswerten", anonymous=False)
    auswerten()