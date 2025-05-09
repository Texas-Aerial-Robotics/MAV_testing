import logging
import zmq
import zmq.asyncio
from datetime import datetime
from time import sleep
import json
import os

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


class ZMQLoggingHandler(logging.Handler):
    def __init__(self, address="tcp://127.0.0.1:7777"):
        super().__init__()
        self.address = address
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.address)
        self.socket.send_string("Connection started")

    def emit(self, record):
        try:
            msg = self.format(record)
            self.socket.send_string(msg)
        except Exception:
            self.handleError(record)


zmq_handler = ZMQLoggingHandler(
    address=os.getenv("ZMQ_ADDRESS", "tcp://10.42.0.2:7777")
)
formatter = logging.Formatter("%(levelname)s - %(message)s")
zmq_handler.setFormatter(formatter)

if not os.getenv("NO_LOG"):
    logger.addHandler(file_handler)
logger.addHandler(console_handler)
logger.addHandler(zmq_handler)


if __name__ == "__main__":
    while True:
        pos_result = {"position_result": {"latitude": 32.7302207, "longitude": -97.1270119, "altitude": 181.65000915527344}}
        
        logger.info(json.dumps(pos_result))
        sleep(1)
