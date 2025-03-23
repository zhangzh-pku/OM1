import time

import zenoh

from zenoh_idl import sensor_msgs

"""
ros2 topic echo /pi/scan
ros2 topic echo /battery_state
"""


def listenerScan(sample):
    # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")
    scan = sensor_msgs.LaserScan.deserialize(sample.payload.to_bytes())
    print(f"Scan {scan}")


def listenerBattery(sample):
    # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")
    battery = sensor_msgs.BatteryState.deserialize(sample.payload.to_bytes())
    print(f"Battery {battery}")


if __name__ == "__main__":

    with zenoh.open(zenoh.Config()) as session:
        print("Zenoh is open")
        scan = session.declare_subscriber("pi/scan", listenerScan)
        batt = session.declare_subscriber("battery_state", listenerBattery)
        while True:
            print("Waiting for pi/scan and battery_state messages")
            time.sleep(1)
