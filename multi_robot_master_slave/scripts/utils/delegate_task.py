from abc import ABC, abstractmethod

class Handler(ABC):
    def __init__(self, successor=None):
        self._successor = successor

    @abstractmethod
    def handle_request(self, request):
        pass

class FreeSlaveHandler(Handler):
    def handle_request(self, request):
        found_free_slave, free_slave = self.find_free_slave(request['name_master'], request['system_master_slave'])
        if found_free_slave:
            nav_free_slave = request['slaves'][free_slave]["nav_class"]
            request['slaves'][free_slave]["task_queue"][request['name_slave']] = request['goal_poses'][request['current_waypoint']:]
            return nav_free_slave, True
        elif self._successor:
            return self._successor.handle_request(request)
        return None, False

    def find_free_slave(self, name_master, system_master_slave):
        print("---> Buscando esclavo libre que pueda realizar la tarea... <---")
        find_free_slave = False
        slave = None
        
        list_slaves = system_master_slave[name_master]["slaves"]
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 0:
                find_free_slave = True
                slave = name_slave_iter
                break
        
        return find_free_slave, slave

class SlaveWithOneTaskHandler(Handler):
    def handle_request(self, request):
        found_slave_with_one_task, slave_with_one_task = self.find_slave_with_one_task(request['name_master'], request['name_slave'], request['system_master_slave'])
        if found_slave_with_one_task:
            nav_slave_with_one_task = request['slaves'][slave_with_one_task]["nav_class"]
            request['slaves'][slave_with_one_task]["task_queue"][request['name_slave']] = request['goal_poses'][request['current_waypoint']:]
            return nav_slave_with_one_task, True
        elif self._successor:
            return self._successor.handle_request(request)
        return None, False

    def find_slave_with_one_task(self, name_master, name_slave, system_master_slave):
        print("---> Buscando esclavo con una tarea pendiente que pueda realizar la tarea... <---")
        find_slave_with_one_task = False
        slave = None
        
        list_slaves = system_master_slave[name_master]["slaves"]
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 1:
                find_slave_with_one_task = True
                slave = name_slave_iter
                break
        
        return find_slave_with_one_task, slave

class MasterHandler(Handler):
    def handle_request(self, request):
        nav_master = request['system_master_slave'][request['name_master']]["nav_class"]
        request['system_master_slave'][request['name_master']]["slave_tasks"][request['name_slave']] = request['goal_poses'][request['current_waypoint']:]
        self.is_master_busy(nav_master.getFeedback())
        return nav_master, False

    def is_master_busy(self, nav_master_feedback):
        if nav_master_feedback is None:
            print("Master disponible, se efectuará la tarea de manera inmediata.")
        else:                
            print("Master ocupado, está en línea de espera.")