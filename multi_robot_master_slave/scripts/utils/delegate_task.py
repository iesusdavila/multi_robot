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
        nav_slave = request['system_master_slave'][request['name_master']]["slaves"][request['name_slave']]["nav_class"]
        nav_slave.info("---> Buscando esclavo libre que pueda realizar la tarea... <---")
        found_free_slave, free_slave = self.find_free_slave(request['name_master'], request['system_master_slave'])
        if found_free_slave:
            request['slaves'][free_slave]["nav_class"].info(f'Para el esclavo {request["name_slave"]}, yo el robot {free_slave} estoy libre.')
            nav_free_slave = request['slaves'][free_slave]["nav_class"]
            return nav_free_slave, True
        else:
            return super().handle(request)

    def find_free_slave(self, name_master, system_master_slave):
        find_free_slave = False
        slave = None
        
        list_slaves = system_master_slave[name_master]["slaves"]
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 0 and list_slaves[name_slave_iter]["status"] == True:
                find_free_slave = True
                slave = name_slave_iter
                break
        
        return find_free_slave, slave

class SlaveWithOneTaskHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        nav_slave = request['system_master_slave'][request['name_master']]["slaves"][request['name_slave']]["nav_class"]
        nav_slave.info("---> Buscando esclavo con una tarea pendiente que pueda realizar la tarea... <---")
        found_slave_with_one_task, slave_with_one_task = self.find_slave_with_one_task(request['name_master'], request['name_slave'], request['system_master_slave'])
        if found_slave_with_one_task:
            request['slaves'][slave_with_one_task]["nav_class"].info(f'Para el esclavo {request["name_slave"]}, yo el robot {slave_with_one_task} tengo una tarea pendiente.')
            nav_slave_with_one_task = request['slaves'][slave_with_one_task]["nav_class"]
            return nav_slave_with_one_task, True
        else:
            return super().handle(request)

    def find_slave_with_one_task(self, name_master, name_slave, system_master_slave):
        find_slave_with_one_task = False
        slave = None
        
        list_slaves = system_master_slave[name_master]["slaves"]
        for name_slave_iter in list_slaves:
            if len(list_slaves[name_slave_iter]["task_queue"]) == 1 and list_slaves[name_slave_iter]["status"] == True and name_slave_iter != name_slave:
                find_slave_with_one_task = True
                slave = name_slave_iter
                break
        
        return find_slave_with_one_task, slave

class MasterHandler(AbstractHandler):
    def handle(self, request: dict) -> Optional[Any]:
        nav_master = request['system_master_slave'][request['name_master']]["nav_class"]
        request['system_master_slave'][request['name_master']]["slave_tasks"][request['name_slave']] = request['goal_poses'][request['current_waypoint']:]
        if request['system_master_slave'][request['name_master']]["slave_tasks"][request['name_slave']]:
            self.is_master_busy(nav_master.getFeedback())
            nav_master.info(f'Para el esclavo {request["name_slave"]}, yo el robot {request["name_master"]} soy el maestro para ejecutar su tarea.')
            return nav_master, False
        else:
            return super().handle(request)

    def is_master_busy(self, nav_master_feedback):
        if nav_master_feedback is None:
            print("Master disponible, se efectuará la tarea de manera inmediata.")
        else:                
            print("Master ocupado, está en línea de espera.")