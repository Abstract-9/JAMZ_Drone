from flask import Flask, request
from argparse import ArgumentParser
from droneLink import DroneLink

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
args = parser.parse_args()

app = Flask(__name__)

drone = DroneLink(args.device)


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


if __name__ == '__main__':
    print(drone.get_status())
    app.run()


