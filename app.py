from flask import Flask
from argparse import ArgumentParser
from droneLink import DroneLink

app = Flask(__name__)

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int,
                    help="master port baud rate", default=115200)
args = parser.parse_args()


@app.route('/test_ascend')
def hello_world():
    return 'Hello World!'


def init_droneLink(args):
    # Connect to device, specifying the serial port for IO
    drone = DroneLink(args.device, None)
    drone.get_status()
    print(drone.get_altitude())
    print(drone.take_off(20))
    print(drone.get_status())
    drone.close_connection()


if __name__ == '__main__':
    init_droneLink(args)
    app.run()


