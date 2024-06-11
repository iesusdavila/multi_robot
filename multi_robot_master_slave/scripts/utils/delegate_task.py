from abc import ABC, abstractmethod
from typing import Any, Optional

class Handler(ABC):
    @abstractmethod
    def set_next(self, handler: 'Handler') -> 'Handler':
        pass

    @abstractmethod
    def handle(self, request: dict) -> Optional[Any]:
        pass

class AbstractHandler(Handler):
    _next_handler: Handler = None

    def set_next(self, handler: Handler) -> Handler:
        self._next_handler = handler
        return handler

    @abstractmethod
    def handle(self, request: dict) -> Optional[Any]:
        if self._next_handler:
            return self._next_handler.handle(request)
        return None

class FreeSlaveHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        request["nav_slave"].info("---> Buscando esclavo libre que pueda realizar la tarea... <---")
        
        name_slave = request["nav_slave"].getNameRobot()

        nav_master = request["dict_master"]["nav_class"]
        name_master = nav_master.getNameRobot()
        list_slaves = request["dict_master"]["slaves"]

        found_free_slave, free_slave = self.find_free_slave(name_master, list_slaves)
        if found_free_slave:
            nav_free_slave = list_slaves[free_slave]["nav_class"]
            nav_free_slave.info(f'Para el esclavo {name_slave}, yo el robot {free_slave} estoy libre.')
            return nav_free_slave, True
        else:
            return super().handle(request)

    def find_free_slave(self, name_master, list_slaves):
        find_free_slave = False
        slave = None
        
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 0 and list_slaves[name_slave_iter]["status"] == True:
                find_free_slave = True
                slave = name_slave_iter
                break
        
        return find_free_slave, slave

class SlaveWithOneTaskHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        request["nav_slave"].info("---> Buscando esclavo con una tarea pendiente que pueda realizar la tarea... <---")
        
        name_slave = request["nav_slave"].getNameRobot()
        
        nav_master = request["dict_master"]["nav_class"]
        name_master = nav_master.getNameRobot()
        list_slaves = request["dict_master"]["slaves"]
        
        found_slave_with_one_task, slave_with_one_task = self.find_slave_with_one_task(name_master, name_slave, list_slaves)
        if found_slave_with_one_task:
            nav_slave_with_one_task = list_slaves[slave_with_one_task]["nav_class"]
            nav_slave_with_one_task.info(f'Para el esclavo {name_slave}, yo el robot {slave_with_one_task} tengo una tarea pendiente.')
            return nav_slave_with_one_task, True
        else:
            return super().handle(request)

    def find_slave_with_one_task(self, name_master, name_slave, list_slaves):
        find_slave_with_one_task = False
        slave = None
        
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 1 and list_slaves[name_slave_iter]["status"] == True and name_slave_iter != name_slave:
                find_slave_with_one_task = True
                slave = name_slave_iter
                break
        
        return find_slave_with_one_task, slave

class MasterHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:

        name_slave = request["nav_slave"].getNameRobot()

        nav_master = request["dict_master"]["nav_class"]
        name_master = nav_master.getNameRobot()

        request['dict_master']["slave_tasks"][name_slave] = request['goal_poses'][request['current_waypoint']:]
        if request['dict_master']["slave_tasks"][name_slave]:
            self.is_master_busy(nav_master.getFeedback())
            nav_master.info(f'Para el esclavo {name_slave}, yo el robot {name_master} soy el maestro para ejecutar su tarea.')
            return nav_master, False
        else:
            return super().handle(request)

    def is_master_busy(self, nav_master_feedback):
        if nav_master_feedback is None:
            print("Master disponible, se efectuará la tarea de manera inmediata.")
        else:                
            print("Master ocupado, está en línea de espera.")