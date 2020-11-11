import asyncio
from argparse import ArgumentParser

from jamz_drone import initialize

parser = ArgumentParser(description=__doc__)

parser.add_argument("--device", required=True, help="mavlink device connection")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
parser.add_argument("--home", required=False, default="coap://loganrodie.me", help="Brain url")
parser.add_argument("--interval", required=False, default=1, help="heartbeat interval in seconds. Defaults to 1.")
args = parser.parse_args()

if __name__ == '__main__':
    asyncio.run(initialize(args))
