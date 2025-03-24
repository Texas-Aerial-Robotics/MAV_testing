import zmq
import zmq.asyncio
import asyncio
import logging
import json
import re
import os
from datetime import datetime

log_filename = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.log")

logger = logging.getLogger("logger")
logger.setLevel(logging.DEBUG if os.getenv("DEBUG") else logging.INFO)

file_handler = logging.FileHandler(log_filename)
file_handler.setFormatter(
    logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
)

console_handler = logging.StreamHandler()
console_handler.setFormatter(
    logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
)

if not os.getenv("NO_LOG"):
    logger.addHandler(file_handler)
logger.addHandler(console_handler)


async def log_receiver(address="tcp://10.42.0.2:7777"):
    context = zmq.asyncio.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"Listening for logs on {address}...")

    while True:
        message = await socket.recv_string()
        if "position_result" in message:
            logger.debug(f"Attempting to decode message {message}...")
            try:
                message_stripped = re.sub(r"^[A-Z]* - ", "", message)
                pos_data = json.loads(message_stripped)
                latitude = pos_data["position_result"]["latitude"]
                longitude = pos_data["position_result"]["longitude"]

                logger.info(f"Receiver got position! {latitude}, {longitude}")

                # TODO send reply

                # TODO: use data...

                continue
            except Exception as err:
                logger.debug(f"Failed to decode position message {message}: {err}.")

        logger.info(f"Received log: {message}")


async def main():
    while True:
        #print("Waiting...")
        await asyncio.sleep(5)


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.create_task(log_receiver())
    loop.create_task(main())

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print("Shutting down.")
