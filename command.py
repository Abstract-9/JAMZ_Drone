from enum import Enum
import json


class Command:
    # Command Types
    GOTO = "GOTO"
    ASCEND = "ASCEND"
    DESCEND = "DESCEND"
    RTL = "RTL"

    # Command Status
    wasExecuted = False

    def __init__(self, command, lat=None, lon=None, alt=None):
        self.command = command
        if command == self.GOTO and lat is not None and lon is not None:
            self.lat = lat
            self.lon = lon
        if alt is not None:
            self.alt = alt

    def to_dict(self):
        if self.command != self.GOTO:
            return {"command": self.command.name}
        else:
            return {"command": self.command.name,
                    "lat": self.lat,
                    "lon": self.lon}

    @staticmethod
    def json_to_list(commands):
        result = []
        for command in commands:
            if command['command'] == Command.GOTO:
                result.append(Command(Command.GOTO, lat=float(command['lat']), lon=float(command['lon']), alt=float(command['alt'])))
            elif command['command'] == Command.ASCEND:
                result.append(Command(Command.ASCEND, alt=float(command['alt'])))
            else:
                result.append(Command(command['command']))
        return result
