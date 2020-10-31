from flask import Flask, request
from argparse import ArgumentParser
from autopilot.droneLink import DroneLink

import asyncio
import requests
import time

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--home", required=False, default="http://loganrodie.me:4000", help="Brain url")
args = parser.parse_args()


def call_home():
    try:
        response = requests.get(args.home + "/drones/initialize")
    except Exception as e:
        print("Home isn't responding. I won't function without home :(")
        return None
    if 'id' in response.json():
        return response.json()['id']


def app_factory(drone):
    app = Flask(__name__)

    @app.route('/test_ascend', methods=['POST'])
    def test_ascend():
        drone.take_off(int(request.form['height']))

    @app.route('/test_descend', methods=['POST'])
    def test_descend():
        asyncio.create_task(drone.land())
        return 200

    @app.route('/test_arm', methods=['POST'])
    def test_arm():
        drone.arm()
        return 200

    @app.route("/test_goto", methods=['POST'])
    def test_goto():
        asyncio.create_task(drone.goto([float(request.form['lat']), float(request.form['lon']), float(request.form['alt'])]))
        return 200

    @app.route("/test_RTL", methods=['POST'])
    def test_rtl():
        asyncio.create_task(drone.return_home())
        return 200

    @app.route("/test_disarm", methods=['POST'])
    def test_disarm():
        drone.disarm()
        return 200

    @app.route("/location", methods=['GET'])
    def location():
        return drone.get_location()

    @app.route("/acquire/commands", methods=['POST'])
    def receive_commands():
        drone.receive_commands(request.get_json())
        return 200

    return app


def initialize():
    drone_id = 0  # call_home() // Testing
    while drone_id is None:
        time.sleep(5)
        drone_id = 0  # call_home()
    drone = DroneLink(args.device, drone_id, args.home)
    print(drone.get_status())
    print("DRONE ID: %d" % drone_id)
    app = app_factory(drone)
    app.run(host='0.0.0.0')

if __name__ == '__main__':
    initialize()


