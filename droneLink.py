from dronekit import connect, VehicleMode
import dronekit_sitl
import time


class DroneLink:
    drone = None
    # Define drone loiter altitude. It'll be much higher in prod
    LOITER_ALTITUDE = 15

    def get_status(self):
        print("Get some self.drone attribute values:")
        print(" GPS: %s" % self.drone.gps_0)
        print(" Battery: %s" % self.drone.battery)
        print(" Last Heartbeat: %s" % self.drone.last_heartbeat)
        print(" Is Armable?: %s" % self.drone.is_armable)
        print(" System status: %s" % self.drone.system_status.state)
        print(" Mode: %s" % self.drone.mode.name)  # settable
        print(" Altitude: %s" % self.drone.location.global_relative_frame.alt)

    def close_connection(self):
        self.drone.close()
        if self.testing:
            self.sitl.stop()

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
        while not self.drone.mode.name == "STABILIZE" and not self.drone.armed:
            print("Arming motors... Vehicle Mode: ")
            time.sleep(1)
        print("Armed!")

    def get_home_location(self):
        while not self.drone.home_location:
            cmds = self.drone.commands
            cmds.download()
            cmds.wait_ready()
            if not self.drone.home_location:
                print("Waiting for home location...")
        return self.drone.home_location

    def take_off(self, altitude):
        # Can't take off without arming
        if not self.drone.armed:
            self.arm()
        # Can't take off without being in GUIDED mode
        # Ready to go!
        print("Taking off!")
        self.drone.simple_takeoff(altitude)  # Take off
        while True:
            print(" Altitude: ", self.drone.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.drone.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude: Ready for mission")
                break
            time.sleep(1)

    def return_home(self):
        self.drone.mode = VehicleMode("RTL")
        while not self.drone.mode.name == "RTL":
            print("Setting RTL mode...")
            self.drone.mode = VehicleMode("RTL")
            time.sleep(1)
        print("Returning home :)")

    def get_altitude(self):
        return self.drone.location.global_relative_frame.alt

    def __init__(self, device, testing=False):
        if testing:
            self.testing = True
            self.sitl = dronekit_sitl.start_default()
            self.drone = connect(self.sitl.connection_string(), wait_ready=True)
            print("Drone connected on SITL")
        else:
            self.testing = False
            self.drone = connect(device)
            print("Drone connected!")
        cmds = self.drone.commands
        cmds.download()
        cmds.wait_ready()
        print("Ready to go! Home location: %s" % self.drone.home_location)
