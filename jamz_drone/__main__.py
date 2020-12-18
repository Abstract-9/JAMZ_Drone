import asyncio
from argparse import ArgumentParser

from .app import get_instance

if __name__ == '__main__':
    asyncio.run(get_instance().initialize())
