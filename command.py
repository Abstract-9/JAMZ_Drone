from enum import Enum
from droneLink import DroneLink


class Command:

    class Commands(Enum):
        GOTO = DroneLink.go_to
        RTL = DroneLink.return_home
        ASCEND = DroneLink.take_off
        DESCEND = DroneLink.land

    def __init__(self, command: Commands, lat=None, lon=None):
        self.command = command
        if command == self.Commands.GOTO and lat is not None and lon is not None:
            self.lat = lat
            self.lon = lon

    def to_dict(self):
        if self.command != self.Commands.GOTO:
            return {"command": self.command.name}
        else:
            return {"command": self.command.name,
                    "lat": self.lat,
                    "lon": self.lon}
