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
    # "connect": { "endpoints": ["tcp/localhost:7447"]
    # config = zenoh.Config.from_json5('{\
    #     "mode": "peer",\
    #     "connect": { "endpoints":["tcp/193.168.1.193:7445"]}\
    #     } ')
    # with zenoh.open(config) as session:

    with zenoh.open(zenoh.Config()) as session:
        print("Zenoh is open")
        scan = session.declare_subscriber("pi/scan", listener)
        batt = session.declare_subscriber("battery_state", listener)
        while True:
            print("Waiting for pi/scan and pi/battery_state messages")
            time.sleep(1)
