from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import requests
import math
from command import Command


class DroneLink:
    drone = None
    # Define drone default altitude. It'll be much higher in prod
    DEFAULT_ALTITUDE = 15
    # Define the amount of time that the drone can continue its mission without talking to the controller.
    NETWORK_TIMEOUT = 120  # 120 seconds
    command_running = False

    def get_status(self):
        return {
                "GPS": self.drone.gps_0,
                "Battery": self.drone.battery,
                "Last Heartbeat": self.drone.last_heartbeat,
                "Armable": self.drone.is_armable,
                "Status": self.drone.system_status.state,
                "Mode": self.drone.mode,
                "Altitude": self.drone.location.global_relative_frame.alt
                }

    def close_connection(self):
        self.drone.close()

    def arm(self):
        while not self.drone.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        self.drone.mode = VehicleMode("GUIDED")
        while not self.drone.mode.name == "GUIDED":
            print("Changing to GUIDED...")
            self.drone.mode = "GUIDED"
            time.sleep(1)
        self.drone.armed = True
        while not self.drone.armed:
            print("Arming motors... Vehicle Mode: ")
            time.sleep(1)
        print("Armed!")

    def disarm(self):
        self.drone.mode = VehicleMode("STABILIZE")
        while not self.drone.mode.name == "STABILIZE":
            print("Changing to STABILIZE...")
            self.drone.mode = "STABILIZE"
            time.sleep(1)
        self.drone.armed = False
        while self.drone.armed:
            print("Disarming motors... Vehicle Mode: ")
            time.sleep(1)
        print("Disarmed!")

    def get_home_location(self):
        while not self.drone.home_location:
            cmds = self.drone.commands
            cmds.download()
            cmds.wait_ready()
            if not self.drone.home_location:
                print("Waiting for home location...")
        return self.drone.home_location

    def take_off(self, altitude=5):
        # Can't take off without arming
        if not self.drone.armed:
            self.arm()
        # Ready to go!
        print("Taking off!")

        # Make sure takeoff happens
        self.drone.simple_takeoff(altitude)
        while self.drone.location.global_relative_frame.alt < 1:
            self.drone.mode = VehicleMode("GUIDED")
            self.drone.simple_takeoff(altitude)
        while True:
            print(" Altitude: ", self.drone.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.drone.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude: Ready for mission")
                break
            time.sleep(1)

    def goto(self, coordinates):
        current_location = self.drone.location.global_relative_frame
        target_location = LocationGlobalRelative(coordinates[0], coordinates[1], 5)
        target_distance = get_distance_metres(current_location, target_location)
        self.drone.simple_goto(target_location, groundspeed=5)

        while self.drone.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            remaining_distance = get_distance_metres(self.drone.location.global_frame, target_location)
            print("Distance to target: ", remaining_distance)
            if remaining_distance <= target_distance * 0.01:  # Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)

    def land(self):
        self.drone.mode = VehicleMode("LAND")
        while self.drone.location.global_relative_frame.alt > 0.5:
            print("Landing...")
            time.sleep(1)
        print("Touchdown!")

    def return_home(self):
        self.drone.home_location = self.home_location
        self.drone.mode = VehicleMode("RTL")
        while not self.drone.mode.name == "RTL":
            print("Setting RTL mode...")
            self.drone.mode = VehicleMode("RTL")
            time.sleep(1)
        print("Returning home :)")

    def get_location(self):
        return {
            "lat": self.drone.location.global_relative_frame.lat,
            "lon": self.drone.location.global_relative_frame.lon,
            "alt": self.drone.location.global_relative_frame.alt
        }

    def send_heartbeat(self):
        try:
            data = self.get_location().update(
                {"state": self.drone.system_status.state,
                 "mode": self.drone.mode.name,
                 "velocity": self.drone.velocity,
                 "heading": self.drone.heading})
            response = requests.post("%s/drones/%d/heartbeat" % (self.controller, self.drone_id), data=data)
        except Exception as e:
            print("uh oh, can't reach home. Keep flying for now...")
            self.time_without_network += 5
            if self.time_without_network == self.NETWORK_TIMEOUT:
                print("network timeout reached. Returning home.")
                self.return_home()
            return
        if response.status_code == 505:
            # 505 means we need to change our ID. This should almost never happen
            self.drone_id = response.json()['id']

    def receive_commands(self, commands):
        self.commands.extend(Command.json_to_list(commands))
        self.run_commands()

    def run_commands(self):
        if len(self.commands) != 0 and not self.command_running:
            self.command_running = True
            while len(self.commands) > 0:
                command = self.commands[0]
                if command.command == Command.Commands.GOTO:
                    self.goto([command.lat, command.lon])
                elif command.command == Command.Commands.ASCEND:
                    self.take_off(command.alt)
                elif command.command == Command.Commands.DESCEND:
                    self.land()
                else:
                    self.return_home()
                self.commands.pop(0)
                time.sleep(1)

    def __init__(self, device, drone_id, home):
        self.drone = connect(device)
        print("Drone connected!")
        # Variables for talking to the controller
        self.drone_id = drone_id
        self.controller = home
        self.commands = []
        # Variables for controlling network timeout
        self.heartbeat_counter = 0
        self.time_without_network = 0
        # Wait for flight controller to be ready
        self.drone.wait_ready()
        cmds = self.drone.commands
        cmds.download()
        cmds.wait_ready()

        self.home_location = self.drone.home_location
        while self.home_location is None:
            self.home_location = self.drone.home_location

        print("Ready to go! Home location: %s" % self.home_location)

        @self.drone.on_message("HEARTBEAT")
        def on_heartbeat(drone, name, msg):
            print("LAT: %f" % drone.location.global_relative_frame.lat)
            print("LNG: %f" % drone.location.global_relative_frame.lon)
            print("ALT: %f" % drone.location.global_relative_frame.alt)
            print("STATUS: %s | MODE: %s" % (drone.system_status.state, self.drone.mode))
            print("AIRSPEED: %d" % drone.airspeed)
            self.heartbeat_counter += 1
            if self.heartbeat_counter == 5:
                self.send_heartbeat()
                self.heartbeat_counter = 0


def get_distance_metres(location1, location2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5