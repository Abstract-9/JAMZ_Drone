from argparse import ArgumentParser

from jamz_drone.autopilot.droneLink import DroneLink

from aiocoap.numbers.codes import Code
from jamz_drone.command import Command

import aiocoap
import serial
import asyncio
import time
import json

instance = None


async def run():
    await get_instance().initialize()


def get_instance():
    # Using a global is necessary here, but its ok in this scenario
    global instance
    if instance:
        return instance
    else:
        instance = _App()
        return instance


class _App:

    def __init__(self):
        self.args = self.parse_args()
        self.commands = []
        self.current_command = None

    def parse_args(self):
        parser = ArgumentParser(description=__doc__)

        parser.add_argument("--device", required=True, help="mavlink device connection")
        parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
        parser.add_argument("--home", required=False, default="coap://loganrodie.me", help="Brain url")
        parser.add_argument("--interval", required=False, default=1,
                            help="heartbeat interval in seconds. Defaults to 1.")
        return parser.parse_args()

    async def init_home(self, client):
        request = aiocoap.Message(code=Code.PUT, uri="coap://" + self.args.home + "/drones/initialize")

        try:
            response = await client.request(request).response
        except Exception as e:
            print("Failed to call home. I won't function without home :(")
            print(e)
            return None
        else:
            if response.code == Code.CREATED or response.code == Code.VALID:
                return json.loads(response.payload)['id']
            else:
                return None

    async def main_loop(self, client, drone):
        while True:
            status = await self.heartbeat(client, drone)
            if isinstance(status, aiocoap.Message):
                # We have updates! forward them to the drone
                self.commands.extend(Command.json_to_list(json.loads(status.payload)['commands']))
            # This means home isn't responding.
            elif status is None:
                await asyncio.sleep(1)

            # Heartbeat has been handled. Next: manage flight
            # Lets base our decision on the current flight status

            # In this scenario, we cycle to the next command
            if drone.status == drone.STATUS_IDLE or drone.status == drone.STATUS_DONE_COMMAND:
                if len(self.commands) == 0:
                    # Case: All commands have been executed
                    await asyncio.sleep(1)
                else:
                    self.current_command = self.commands.pop(0)
                    asyncio.create_task(drone.execute_command(self.current_command))
            # In this scenario, we ensure command execution
            elif drone.status == drone.STATUS_EXECUTING_COMMAND:
                if self.current_command:
                    asyncio.create_task(drone.execute_command(self.current_command))
            # In this scenario, we handle obstacle avoidance
            elif drone.status == drone.STATUS_OBSTACLE_INTERRUPTED:
                asyncio.create_task(drone.reroute())
            await asyncio.sleep(1)

    async def heartbeat(self, client, drone):
        print(drone.get_location())
        payload_object = drone.get_location()
        payload_object['battery'] = drone.drone.battery.level
        payload = bytes(json.dumps(payload_object).encode('UTF-8'))

        request = aiocoap.Message(code=Code.GET, payload=payload, uri="coap://" + self.args.home + "/drones/heartbeat")

        try:
            response = await client.request(request).response
        except Exception as e:
            print("heartbeat failed.")
            return None
        else:
            if response.code == Code.VALID:
                print("Connected to home. " + str(asyncio.get_event_loop().time()))
                return True
            elif response.code == Code.CONTENT:
                print("Home has updates!" + str(asyncio.get_event_loop().time()))
                return response

    async def initialize(self):
        client = await aiocoap.Context.create_client_context()
        drone_id = await self.init_home(client)
        while drone_id is None:
            time.sleep(5)
            drone_id = 0  # call_home()
        print("DRONE ID: %d" % drone_id)
        drone = DroneLink(self.args.device, drone_id, self.args.home)
        print(drone.get_status())

        asyncio.create_task(self.sensor_loop(drone))
        await asyncio.create_task(self.main_loop(client, drone))

    async def sensor_loop(self, drone):
        port = serial.Serial("/dev/ttyS0")
        while True:
            message = port.read_until(b"|")
            obj = json.loads(message)
            # TODO a bunch of stuff regarding sensors
            print(message)
            # Example
            if 'detection_event' in obj:
                await drone.on_detection()


if __name__ == '__main__':
    asyncio.run(get_instance().initialize())
