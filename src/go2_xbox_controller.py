import logging
import time

from unitree.unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient

try:
    import hid
except ImportError:
    logging.warning(
        "HID library not found. Please install the HIDAPI library to use this plugin."
    )
    hid = None

ChannelFactoryInitialize(0, "en10")

sport_client = SportClient()
sport_client.Init()


def _move_robot(move_speed_x, move_speed_y, rotate_speed=0.0) -> None:
    try:
        if move_speed_x == 0.0 and move_speed_y == 0.0 and rotate_speed == 0.0:
            return
        print(
            f"Moving robot: move_speed_x={move_speed_x}, move_speed_y={move_speed_y}, rotate_speed={rotate_speed}"
        )
        sport_client.Move(move_speed_x, move_speed_y, rotate_speed)
    except Exception as e:
        logging.error(f"Error moving robot: {e}")


# from actions.base import ActionConfig, ActionConnector
# from actions.move_safe.interface import MoveInput
# from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient

if hid is not None:
    for device in hid.enumerate():
        logging.debug(f"device {device['product_string']}")
        if "Xbox Wireless Controller" in device["product_string"]:
            vendor_id = device["vendor_id"]
            product_id = device["product_id"]
            gamepad = hid.Device(vendor_id, product_id)
            logging.info(
                f"Connected {device['product_string']} {vendor_id} {product_id}"
            )
            break

rt_previous = 0
lt_previous = 0

move_speed = 0.5
turn_speed = 0.5

d_pad_previous = 0

if gamepad:
    while True:
        logging.info(f"Gamepad found: {gamepad}")

        data = list(gamepad.read(64))

        rt_value = data[11]  # Right Trigger
        lt_value = data[9]  # Left Trigger

        rt_changed = abs(rt_value - rt_previous) > 5
        lt_changed = abs(lt_value - lt_previous) > 5
        # Process triggers for rotation
        # RT is typically on byte 9, LT on byte 8 for Xbox controllers
        rt_value = data[11]  # Right Trigger
        lt_value = data[9]  # Left Trigger

        print(f"RT: {rt_value}, LT: {lt_value}")
        if rt_changed or lt_changed or rt_value == 255 or lt_value == 255:
            rt_previous = rt_value
            lt_previous = lt_value

            rt_normalized = rt_value / 255.0
            lt_normalized = lt_value / 255.0

            print(
                f"RT: {rt_value} ({rt_normalized:.2f}), LT: {lt_value} ({lt_normalized:.2f})"
            )

            if rt_normalized > 0.1 and rt_normalized > lt_normalized:
                _move_robot(0.0, 0.0, -turn_speed)

            # Left Trigger - counter-clockwise rotation
            elif lt_normalized > 0.1 and lt_normalized > rt_normalized:
                _move_robot(0.0, 0.0, turn_speed)

            # Both triggers released or below threshold
            elif rt_normalized <= 0.1 and lt_normalized <= 0.1:
                logging.debug("Triggers released - Stopping rotation")
                _move_robot(0.0, 0.0)

        d_pad_value = data[13]

        print(f"D-Pad Value: {d_pad_value}")
        if d_pad_value != d_pad_previous:
            d_pad_previous = d_pad_value
            print(f"D-Pad Value: {d_pad_value}")

            if d_pad_value == 1:  # Up
                print("D-Pad Up")
                start_time = time.time()
                _move_robot(move_speed, 0.0)
                end_time = time.time()
                elapsed_time = end_time - start_time
                print(f"Elapsed time: {elapsed_time:.2f} seconds")
            elif d_pad_value == 5:  # Down
                print("D-Pad Down")
                _move_robot(-move_speed, 0.0)
            elif d_pad_value == 7:  # Left
                print("D-Pad Left")
                _move_robot(0.0, move_speed)
            elif d_pad_value == 3:  # Right
                print("D-Pad Right")
                _move_robot(0.0, -move_speed)
            elif d_pad_value == 0:  # Nothing pressed
                print("D-Pad Nothing")
                _move_robot(0.0, 0.0)

        time.sleep(0.1)
