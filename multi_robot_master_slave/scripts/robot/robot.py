class Robot():
    def __init__(self):
        pass
    
    def generate_message(self, name_robot, current_waypoint, number_poses, nav_time=(0,0,0), max_time=None, name_slave=None):
        hour_nav, min_nav, sec_nav = nav_time
        msg = 'Executing current waypoint ' + str(name_robot) + ': '+ str(current_waypoint + 1) + '/' + str(number_poses) 
        msg += ' - ' + str(hour_nav-19) + ':' + str(min_nav) + ':' + str(sec_nav) 

        if max_time is not None:
            hour_max, min_max, sec_max = max_time
            msg += ' / ' + str(hour_max-19) + ':' + str(min_max) + ':' + str(sec_max)

        if name_slave is not None: 
            msg += " del esclavo: " + name_slave
        
        print(msg)

    def cancel_task(self, nav_robot):
        nav_robot.cancelTask()
        return False
