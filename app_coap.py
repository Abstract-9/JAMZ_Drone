from argparse import ArgumentParser
from droneLink import DroneLink
import aiocoap.resource as resource
from aiocoap.numbers.codes import Code

import aiocoap
import asyncio
import time

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--home", required=False, default="coap://loganrodie.me", help="Brain url")
args = parser.parse_args()


class Location(resource.ObservableResource):
    def __init__(self, drone):
        super().__init__()
        self.handle = None
        self.drone = drone

    def notify(self):
        self.updated_state()
        self.reschedule()

    def reschedule(self):
        self.handle = asyncio.get_event_loop().call_later(5, self.notify)

    def update_observation_count(self, count):
        if count and self.handle is None:
            print("Starting the clock")
            self.reschedule()
        if count == 0 and self.handle:
            print("Stopping the clock")
            self.handle.cancel()
            self.handle = None

    async def render_get(self, request):
        location = self.drone.get_location()
        payload = '{}|{}|{}'.format(location['lat'], location['lon'], location['alt'])
        return aiocoap.Message(payload=payload)


async def call_home(client):
    request = aiocoap.Message(code=Code.PUT, uri="coap://" + args.home + "/drone/initialize")

    try:
        response = await client.request(request).response
    except Exception as e:
        print("Failed to call home. I won't function without home :(")
        print(e)
        return None
    else:
        if response.code == Code.CREATED or response.code == Code.CHANGED:
            return response.payload
        else:
            return None


async def initialize():
    client = await aiocoap.Context.create_client_context()
    drone_id = await call_home(client)
    while drone_id is None:
        time.sleep(5)
        drone_id = 0  # call_home()
    drone = DroneLink(args.device, drone_id, args.home)
    print(drone.get_status())
    print("DRONE ID: %d" % drone_id)
    root = resource.Site()
    root.add_resource(['.well-known', 'core'], resource.WKCResource(root.get_resources_as_linkheader()))
    root.add_resource(['location'], Location(drone))

    asyncio.Task(aiocoap.Context.create_server_context(root))
    asyncio.get_event_loop().run_forever()

if __name__ == '__main__':
    asyncio.run(initialize())
