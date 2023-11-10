import rospy
from subprocess import Popen, PIPE
import time
from std_msgs.msg import Int8
# call(["ls", "-l"])

SEED_ANZAHL = 10
SEEDS_DURCHGEFÜHRT = 1

sub_process: Popen = None

def start_seed_launch():
    global sub_process
    if sub_process is not None:
        sub_process.terminate()
        # rospy.sleep(5)

    datei = open('/home/nik/catkin_ws/src/bachlorarbeit/potential_field/Ergebnisse/ErgebnissePotential_Field_Results.txt','a')
    # datei = open('/home/nik/Documents/TARTW/src/bachlorarbeit/potential_field/Ergebnisse/ErgebnissePotential_Field_Results.txt','a')
    
    datei.write(f'Seed_{SEEDS_DURCHGEFÜHRT}\n')
    datei.close()

    # roslaunch potential_field mehrere_test_starten.launch
    sub_process = Popen(['roslaunch', 'potential_field', 'mehrere_test_starten.launch'], stdout=PIPE)

def seed_auswertungen_durchführen(status_zahl: int):
    global SEEDS_DURCHGEFÜHRT, sub_process

    if status_zahl == Int8(0):
        if SEEDS_DURCHGEFÜHRT < SEED_ANZAHL:
            SEEDS_DURCHGEFÜHRT += 1
            start_seed_launch()
        else:
            sub_process.terminate()
            sub_process = None
            rospy.signal_shutdown('Auswertung der Seeds abgeschlossen')



if __name__ == '__main__':
    rospy.init_node("seeds_auswerten", anonymous=False)
    rospy.Subscriber("/potentiel_field/seed_finished", Int8, seed_auswertungen_durchführen)

    start_seed_launch()
    rospy.spin()