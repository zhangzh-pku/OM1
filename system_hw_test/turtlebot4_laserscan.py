import time

import zenoh

"""
ros2 topic echo /pi/scan
"""


def listener(sample):
    print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")


if __name__ == "__main__":

    # initiate logging
    # zenoh.init_logger()
    # configure zenoh in peer mode
    # conf = zenoh.Config.from_json5('{"mode": "peer"}')

    with zenoh.open(zenoh.Config()) as session:
        print("Zenoh is open")
        sub = session.declare_subscriber("pi/scan", listener)
        while True:
            print("Waiting for pi/scan messages")
            time.sleep(1)
