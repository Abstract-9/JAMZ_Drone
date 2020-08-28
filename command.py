from enum import Enum
import json

class Command:

    class Commands(Enum):
        GOTO = 1
        RTL = 2
        ASCEND = 3
        DESCEND = 4

    def __init__(self, command: Commands, lat=None, lon=None, alt=None):
        self.command = command
        if command == self.Commands.GOTO and lat is not None and lon is not None:
            self.lat = lat
            self.lon = lon
        elif command == self.Commands.ASCEND and alt is not None:
            self.alt = alt

    def to_dict(self):
        if self.command != self.Commands.GOTO:
            return {"command": self.command.name}
        else:
            return {"command": self.command.name,
                    "lat": self.lat,
                    "lon": self.lon}

    @staticmethod
    def json_to_list(commands):
        result = []
        for command in commands:
            for name, item in Command.Commands.__members__.items():
                if command['command'] == name:
                    if name == "GOTO":
                        result.append(Command(item, command['lat'], command['lon']))
                    elif name == "ASCEND":
                        result.append(Command(item, command['alt']))
                    else:
                        result.append(Command(item))
        return result
