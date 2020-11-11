
class FlightStatus:

    def __init__(self, status):
        self.status = status

    def __eq__(self, other):
        return self.status == other.status
