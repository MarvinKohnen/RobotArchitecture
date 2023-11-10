import rospy
from subprocess import call, Popen, PIPE
import time
from std_msgs.msg import Int8
# call(["ls", "-l"])

TEST_ANZAHL = 5
TESTS_DURCHGEFÜHRT = 1

sub_process: Popen = None

def start_test_launch():
    global sub_process
    if sub_process is not None:
        sub_process.terminate()
        rospy.sleep(1)

    # rosservice call /gazebo/reset_world
    call(['rosservice', 'call', '/gazebo/reset_world'])
    rospy.sleep(1)
    call(['rosservice', 'call', '/gazebo/reset_world'])
    rospy.sleep(1)

    # roslaunch potential_field test_durchführen.launch
    sub_process = Popen(['roslaunch', 'potential_field', 'test_durchführen.launch'], stdout=PIPE)
    rospy.loginfo('Versuch gestartet')

def auswertungen_durchführen(status_zahl: int):
    global TESTS_DURCHGEFÜHRT, sub_process
    if status_zahl == Int8(0):
        if TESTS_DURCHGEFÜHRT < TEST_ANZAHL:
            TESTS_DURCHGEFÜHRT += 1
            start_test_launch()
        else:
            if sub_process is not None:
                sub_process.terminate() 
                sub_process = None
            # rospy.sleep(1)
            
            global pub
            pub.publish(Int8(0))

            # rospy.signal_shutdown('Auswertung abgeschlossen')



if __name__ == '__main__':
    rospy.init_node("mehrere_versuch_auswerten", anonymous=False)
    pub = rospy.Publisher("/potentiel_field/seed_finished", Int8, queue_size=1)
    rospy.Subscriber("/potentiel_field/test_finished", Int8, auswertungen_durchführen)

    start_test_launch()
    rospy.spin()