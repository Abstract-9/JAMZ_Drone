from aiocoap import *
import asyncio

async def main():
    protocol = await Context.create_client_context()
    msg = Message(code=GET, uri="coap://192.168.0.10/test")

    try:
        response = await protocol.request(msg).response
    except Exception as e:
        print('Failed to fetch resource:')
        print(e)
    else:
        print('Result: %s\n%r' % (response.code, response.payload))

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())