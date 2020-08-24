from flask import Flask, request
from argparse import ArgumentParser
from droneLink import DroneLink

import requests
import time

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--home", required=False, default="http://loganrodie.me:4000", help="Brain url")
args = parser.parse_args()

app = Flask(__name__)

drone = None


@app.route('/test_ascend', methods=['POST'])
def test_ascend():
    drone.take_off(int(request.form['height']))


@app.route('/test_descend', methods=['POST'])
def test_descend():
    drone.land()


@app.route("/test_goto", methods=['POST'])
def test_goto():
    drone.go_to([float(request.form['lat']), float(request.form['lon'])], request.form['speed'])


@app.route("/test_RTL", methods=['POST'])
def test_rtl():
    drone.return_home()


@app.route("/test_disarm", methods=['POST'])
def test_disarm():
    drone.disarm()


@app.route("/location", methods=['GET'])
def location():
    return drone.get_location()


def call_home():
    try:
        response = requests.get(args.home + "/drones/initialize")
    except Exception as e:
        print("Home isn't responding. I won't function without home :(")
        return False
    if response.json()['id']:
        return response.json()['id']


if __name__ == '__main__':
    drone_id = call_home()
    while not drone_id:
        time.sleep(5)
        drone_id = call_home()
    drone = DroneLink(args.device, drone_id, args.home)
    print(drone.get_status())
    app.run()


