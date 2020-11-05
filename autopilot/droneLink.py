from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import asyncio
import requests
import math
from command import Command

from . import FlightStatus


class DroneLink:
    # Create pre-defined flight status objects
    STATUS_IDLE = FlightStatus(0)
    STATUS_DONE_COMMAND = FlightStatus(1)
    STATUS_EXECUTING_COMMAND = FlightStatus(2)
    STATUS_OBSTACLE_INTERRUPTED = FlightStatus(3)

    # Variables for handling obstacle avoidance
    previous_command = None

    # Define drone default altitude. It'll be much higher in prod
    DEFAULT_ALTITUDE = 5
    # Define drone ground speed in m/s.
    GROUND_SPEED = 1
    # Define the amount of time that the drone can continue its mission without talking to the controller.
    NETWORK_TIMEOUT = 120  # 120 seconds

    ################# CONTROL SECTION #################
    # This section contains methods for changing various states in the flight controller

    async def arm(self):
        while not self.drone.is_armable:
            print(" Waiting for vehicle to initialise...")
            await asyncio.sleep(1)
        self.drone.mode = VehicleMode("GUIDED")
        while not self.drone.mode.name == "GUIDED":
            print("Changing to GUIDED...")
            self.drone.mode = "GUIDED"
            await asyncio.sleep(1)
        self.drone.armed = True
        while not self.drone.armed:
            print("Arming motors... Vehicle Mode: ")
            await asyncio.sleep(1)
        print("Armed!")

    async def disarm(self):
        self.drone.armed = False
        while self.drone.armed:
            print("Disarming motors... Vehicle Mode: ")
            await asyncio.sleep(1)
        print("Disarmed!")

    ############### OBSTACLE AVOIDANCE ###############
    # This section contains the methods for handling obstacle avoidance
    # I'm sure this is going to be quite in-depth, so it'll probably have it's own file eventually

    # This will be called upon a detection event. We will probably classify different events
    # and act differently based on whats going on. Right now it's a basic example.
    async def on_detection(self):
        self.status = self.STATUS_OBSTACLE_INTERRUPTED

        self.drone.mode = VehicleMode("LOITER")
        while self.drone.mode.name != "LOITER":
            self.drone.mode = VehicleMode("LOITER")
            print("Flight controller isn't responding! Trying to loiter...")
            await asyncio.sleep(0.5)

    # This will be used to attempt to fly around an obstacle. It will definitely be very involved
    # and require the output of the sensors, so I should make those values accessible.
    async def reroute(self):
        # Use this to try and do something to avoid
        pass

    ################# FLIGHT SECTION #################
    # This section contains the methods for piloting the drone

    async def take_off(self, command):
        self.status = self.STATUS_EXECUTING_COMMAND

        # Can't take off without arming
        if not self.drone.armed:
            await self.arm()
        # Ready to go!
        print("Taking off!")

        self.altitude = command.alt
        # Make sure takeoff happens
        self.drone.simple_takeoff(command.alt)
        while self.drone.mode.name != "GUIDED":
            print("Taking off | Vehicle mode: {}".format(self.drone.mode.name))
            self.drone.mode = VehicleMode("GUIDED")
            self.drone.simple_takeoff(command.alt)
        command.wasExecuted = True
        await self.ensure_takeoff()
        # while True:
        #     print(" Altitude: ", self.drone.location.global_relative_frame.alt)
        #     # Break and return from function
        #     print(" Altitude: ", self.drone.location.global_relative_frame.alt)
        #     # Break and return from function just below target altitude.
        #     if self.drone.location.global_relative_frame.alt >= command.alt * 0.95:
        #         print("Reached target altitude: Ready for mission")
        #         break
        #     await asyncio.sleep(1)

    async def ensure_takeoff(self):
        def current_distance():
            return self.altitude - self.drone.location.global_relative_frame.alt
        check = current_distance()

        await asyncio.sleep(0.25)
        if current_distance() < check:
            return current_distance()
        elif current_distance() < 0.25: # Close enough, we're done. Goto will ensure flight altitude anyways
            self.status = self.STATUS_DONE_COMMAND
            return self.status
        else:
            self.drone.simple_takeoff(self.altitude)

    async def goto(self, command):
        self.status = self.STATUS_EXECUTING_COMMAND

        self.target_location = LocationGlobalRelative(command.lat, command.lon, command.alt)
        self.drone.simple_goto(self.target_location)

        command.wasExecuted = True
        await self.ensure_goto()
        # while self.drone.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
        #     remaining_distance = get_distance_metres(self.drone.location.global_frame, target_location)
        #     print("Distance to target: ", remaining_distance)
        #     if remaining_distance <= target_distance * 0.01:  # Just below target, in case of undershoot.
        #         print("Reached target")
        #         break
        #     time.sleep(2)

    # Ensures that the drone continues to its destination.
    async def ensure_goto(self):
        current_location = self.drone.location.global_relative_frame
        target_distance = get_distance_metres(current_location, self.target_location)

        await asyncio.sleep(0.25)
        remaining_distance = get_distance_metres(self.drone.location.global_frame, self.target_location)
        print("GOTO: Remaining Distance: {}".format(remaining_distance))
        if remaining_distance < target_distance:  # If this is true, goto is working.
            return remaining_distance

        # We will need a better method of determining arrival, maybe combine airspeed check?
        # TODO: Yes, lets do that. implement a vector transform to get absolute velocity.
        elif remaining_distance < 0.75:
            self.status = self.STATUS_DONE_COMMAND
            return self.status

        # Ensure goto
        else:
            self.drone.simple_goto(self.target_location)

    async def land(self, command):
        self.status = self.STATUS_EXECUTING_COMMAND
        self.drone.mode = VehicleMode("LAND")
        command.wasExecuted = True
        await self.ensure_land()

    async def ensure_land(self):
        if self.drone.mode.name == "LAND":
            if self.drone.velocity[2] < 0.05:  # It's not moving, so we've landed
                self.status = self.STATUS_DONE_COMMAND
                return self.status
        else:
            self.drone.mode = VehicleMode("LAND")

    async def return_home(self):
        self.drone.home_location = self.home_location
        self.drone.mode = VehicleMode("RTL")
        while not self.drone.mode.name == "RTL":
            print("Setting RTL mode...")
            self.drone.mode = VehicleMode("RTL")
            await asyncio.sleep(1)
        print("Returning home :)")

    def set_yaw(self, heading, relative=False):
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.drone.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.drone.send_mavlink(msg)

    # Command mapping. There's definitely a better way to do this.
    async def execute_command(self, command):
        # Command bindings
        command_bindings = {
            Command.GOTO: [self.goto, self.ensure_goto],
            Command.ASCEND: [self.take_off, self.ensure_takeoff],
            Command.DESCEND: [self.land, self.ensure_land],
            Command.RTL: self.return_home
        }

        if not command.wasExecuted:
            await command_bindings[command.command][0](command)
        else:
            await command_bindings[command.command][1]()

    ################# INFORMATION SECTION #################
    # This section stores methods for accessing various information from the flight controller

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

    def get_home_location(self):
        while not self.drone.home_location:
            cmds = self.drone.commands
            cmds.download()
            cmds.wait_ready()
            if not self.drone.home_location:
                print("Waiting for home location...")
        return self.drone.home_location

    def get_location(self):
        return {
            "lat": self.drone.location.global_relative_frame.lat,
            "lon": self.drone.location.global_relative_frame.lon,
            "alt": self.drone.location.global_relative_frame.alt
        }

    ################# Utility section #################
    # This section contains various utility and I/O methods

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

    def close_connection(self):
        self.drone.close()

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

        # Setup initial flight params
        self.drone.groundspeed = self.GROUND_SPEED

        # Variables for controlling flight
        self.altitude = 0
        self.target_location = None
        self.status = self.STATUS_IDLE
        self.home_location = self.drone.home_location
        while self.home_location is None:
            self.home_location = self.drone.home_location

        print("Ready to go! Home location: %s" % self.home_location)


# The following is utility functions for various calculations

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


def get_bearing(location1, location2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = location2.lon - location1.lon
    off_y = location2.lat - location1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

