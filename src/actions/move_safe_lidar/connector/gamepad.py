import logging

from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient

try:
    import hid
except ImportError:
    logging.warning(
        "HID library not found. Please install the HIDAPI library to use this plugin."
    )
    hid = None


class Go2XboxController:
    """
    Xbox controller connector
    """

    def __init__(self):
        """
        Initialize the Xbox controller connector.
        """
        super().__init__()

        self.sport_client = SportClient()
        self.sport_client.Init()

        self.gamepad = None

        if hid is not None:
            for device in hid.enumerate():
                logging.debug(f"device {device['product_string']}")
                if "Xbox Wireless Controller" in device["product_string"]:
                    self.vendor_id = device["vendor_id"]
                    self.product_id = device["product_id"]
                    self.gamepad = hid.Device(self.vendor_id, self.product_id)
                    logging.info(
                        f"Connected {device['product_string']} {self.vendor_id} {self.product_id}"
                    )
                    break
        else:
            # we want this to fail gracefully
            # do not create exception
            logging.warn("No HID library found - check your path")

        if self.gamepad is None:
            # we want this to fail gracefully
            # do not create exception
            logging.warn("No Xbox found - is it connected?")

        # Triggers
        self.rt_previous = 0
        self.lt_previous = 0

        # D pad
        self.d_pad_previous = 0

        # Buttons
        self.button_previous = 0

        # Movement speed
        self.move_speed = 0.5
        self.turn_speed = 0.5

    def _move_robot(self, vx: float, vy: float, vturn: float = 0.0) -> None:
        """
        Move the robot using the sport client.

        Parameters
        ----------
        vx : float
            The speed in the x direction.
        vy : float
            The speed in the y direction.
        vturn : float
            The speed of rotation.

        Returns
        -------
        None
        """
        try:
            if vx == 0.0 and vy == 0.0 and vturn == 0.0:
                return
            logging.debug(f"Moving via Xbox: vx={vx}, vy={vy}, vturn={vturn}")
            self.sport_client.Move(vx, vy, vturn)
        except Exception as e:
            logging.error(f"Error moving robot via Xbox controller: {e}")

    def IsThereACommand(self) -> bool:

        if not self.gamepad:
            return False
        else:
            data = list(self.gamepad.read(64))

            #####################################
            ##### Check the two front triggers
            #####################################

            # Process triggers for rotation
            # RT is typically on byte 9, LT on byte 8 for Xbox controllers
            rt_value = data[11]  # Right Trigger
            lt_value = data[9]  # Left Trigger

            # Trigger values usually range from 0 to 255
            # Check if the triggers have changed significantly
            rt_changed = abs(rt_value - self.rt_previous) > 5
            lt_changed = abs(lt_value - self.lt_previous) > 5

            if rt_changed or lt_changed:

                # Update previous values
                self.rt_previous = rt_value
                self.lt_previous = lt_value

                # Normalize trigger values from 0-255 to 0-1.0
                rt_normalized = rt_value / 255.0
                lt_normalized = lt_value / 255.0

                # Right Trigger - clockwise rotation
                if rt_normalized > 0.1 and rt_normalized > lt_normalized:
                    self._move_robot(0.0, 0.0, -self.turn_speed)

                # Left Trigger - counter-clockwise rotation
                elif lt_normalized > 0.1 and lt_normalized > rt_normalized:
                    self._move_robot(0.0, 0.0, self.turn_speed)

                # Both triggers released or below threshold
                elif rt_normalized <= 0.1 and lt_normalized <= 0.1:
                    logging.debug("Triggers released - Stopping rotation")
                    self._move_robot(0.0, 0.0)

                return True  # yes, there was an action

            d_pad_value = data[13]

            #####################################
            ##### Check the D pad
            #####################################

            if d_pad_value != self.d_pad_previous:

                self.d_pad_previous = d_pad_value

                if d_pad_value == 1:  # Up
                    self._move_robot(self.move_speed, 0.0)
                elif d_pad_value == 5:  # Down
                    self._move_robot(-self.move_speed, 0.0)
                elif d_pad_value == 7:  # Left
                    self._move_robot(0.0, self.move_speed)
                elif d_pad_value == 3:  # Right
                    self._move_robot(0.0, -self.move_speed)
                elif d_pad_value == 0:  # Nothing pressed
                    self._move_robot(0.0, 0.0)

                return True  # yes, there was an action

            #####################################
            ##### Check the Buttons
            #####################################

            button_value = data[14]

            if self.button_previous == 0 and button_value > 0:
                # User just pressed a button
                logging.info(f"Gamepad button depressed edge {button_value}")

                self.button_previous = button_value

                # A button
                if button_value == 1:
                    self._execute_sport_command_sync("StandUp")
                    logging.info("Controller unitree: stand_up")
                # B button
                elif button_value == 2:
                    self._execute_sport_command_sync("StandDown")
                    logging.info("Controller unitree: lay_down")
                # X button
                elif button_value == 8:
                    self._execute_sport_command_sync("Hello")
                    logging.info("Controller unitree: say_hello")
                # Y button
                elif button_value == 16:
                    self._execute_sport_command_sync("Stretch")
                    logging.info("Controller unitree: stretch")

                return True  # yes, there was an action

    # def _execute_command_thread(self, command: str) -> None:

    #     if not self.sport_client:
    #         return

    #     logging.info(f"_execute_command_thread {command}")

    #     try:
    #         if command == "StandUp" and self.current_state == RobotState.STANDING:
    #             logging.info("Already standing, skipping command")
    #             return
    #         elif command == "StandDown" and self.current_state == RobotState.SITTING:
    #             logging.info("Already sitting, skipping command")
    #             return

    #         code = getattr(self.sport_client, command)()
    #         logging.info(f"Unitree command {command} executed with code {code}")

    #         if command == "StandUp":
    #             self.current_state = RobotState.STANDING
    #         elif command == "StandDown":
    #             self.current_state = RobotState.SITTING

    #     except Exception as e:
    #         logging.error(f"Error in command thread {command}: {e}")
    #     finally:
    #         self.thread_lock.release()

    # def _execute_sport_command_sync(self, command: str) -> None:
    #     if not self.sport_client:
    #         return

    #     if not self.thread_lock.acquire(blocking=False):
    #         logging.info("Action already in progress, skipping")
    #         return

    #     try:
    #         thread = threading.Thread(
    #             target=self._execute_command_thread, args=(command,), daemon=True
    #         )
    #         thread.start()
    #     except Exception as e:
    #         logging.error(f"Error executing Unitree command {command}: {e}")
    #         self.thread_lock.release()

    # async def _execute_sport_command(self, command: str) -> None:

    #     logging.info(f"async _execute_sport_command1 {command}")

    #     if not self.thread_lock.acquire(blocking=False):
    #         logging.info("Action already in progress, skipping")
    #         return

    #     try:
    #         thread = threading.Thread(
    #             target=self._execute_command_thread, args=(command,), daemon=True
    #         )
    #         thread.start()
    #     except Exception as e:
    #         logging.error(f"Error executing Unitree command {command}: {e}")
    #         self.thread_lock.release()
