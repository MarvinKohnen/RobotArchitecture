from subprocess import Popen
import time

def get_command_list() -> list:
    command_list = []
    obstacle_count = obstacle_count_min
    while obstacle_count <= obstacle_count_max:
        obstacle_vel = obstacle_vel_min
        while obstacle_vel <= obstacle_vel_max:
            robo_max_vel = robo_m_vel_min
            while robo_max_vel <= robo_m_vel_max:
                command_list.append((TEST_COUNTS, obstacle_count, obstacle_vel, robo_max_vel))
                # print(TEST_COUNTS, obstacle_count, obstacle_vel, robo_max_vel)
                robo_max_vel += robo_m_vel_inc
            obstacle_vel += obstacle_vel_inc
        obstacle_count += 1
    command_list.sort(key=lambda c:c[2])    # nach obstacle_vel sortieren
    print(len(command_list), 'Konfigurationen geladen')
    return command_list

def run_tests():
    command_list = get_command_list()

    proc_list = [None for i in range(sub_proc_at_same_time)]

    time_start = time.time()

    try:
        while len(command_list) > 0:
            for i in range(sub_proc_at_same_time):
                if len(command_list) == 0:
                    proc_list[i] = None
                else:
                    a = command_list.pop()
                    command = f'python3 simulation.py {a[0]} {a[1]} {a[2]} {a[3]}'
                    proc_list[i] = Popen(command.split())

            for proc in proc_list:
                if proc is not None:
                    r = proc.wait()
            print('Thread-Block beendet nach Startbeginn: ',sek_to_time_string(time.time() - time_start))
    except:
        for proc in proc_list:
            if proc is not None:
                r = proc.kill()
        print('All proc killed!')

    print('All tests finished in', sek_to_time_string(time.time() - time_start))


def sek_to_time_string(seconds: float):
    return time.strftime("%H:%M:%S",  time.gmtime(int(seconds)))

if __name__== '__main__':
    command_list = []

    ########### Parameter ###########
    # TEST_COUNTS = 100

    # obstacle_count_min = 1
    # obstacle_count_max = 6

    # obstacle_vel_min = 1
    # obstacle_vel_max = 3
    # obstacle_vel_inc = .5

    # sub_proc_at_same_time = 5
    #################################
    TEST_COUNTS = 100

    obstacle_count_min = 2
    obstacle_count_max = 6

    obstacle_vel_min = 0.5
    obstacle_vel_max = 2
    obstacle_vel_inc = .5

    robo_m_vel_min = 1  # ROBO_MAX_VELOCITY
    robo_m_vel_max = 1
    robo_m_vel_inc = 1

    sub_proc_at_same_time = 10

    # get_command_list()
    
    run_tests()
