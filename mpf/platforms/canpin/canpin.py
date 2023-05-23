"""CANPIN hardware platform.

Contains the hardware interface and drivers for the CANPIN Pinball platform
hardware, using CAN-BUS to communicate with discrete driver boards.
"""
import asyncio
from operator import contains
import os
from collections import defaultdict
from copy import deepcopy
from packaging import version
from serial import SerialException
from typing import Dict, Set, Optional

from mpf.exceptions.runtime_error import MpfRuntimeError
from mpf.platforms.canpin.canpin_bus import CanPinBusCommunicator
from mpf.platforms.canpin.defines import CanPinMessages
from mpf.platforms.base_serial_communicator import HEX_FORMAT

from mpf.core.platform import SwitchPlatform, LightsPlatform, DriverPlatform, SwitchSettings, DriverSettings, \
    DriverConfig, SwitchConfig, RepulseSettings
from mpf.platforms.interfaces.driver_platform_interface import DriverPlatformInterface, PulseSettings, HoldSettings
from mpf.platforms.interfaces.switch_platform_interface import SwitchPlatformInterface
from mpf.core.utility_functions import Util
from mpf.core.logging import LogMixin

# from mpf.platforms.system11 import System11OverlayPlatform, System11Driver

# pylint: disable-msg=too-many-instance-attributes
# from mpf.platforms.interfaces.light_platform_interface import LightPlatformInterface


class CanPinIoBoard:
    """IoBoard class for a single CANPIN board."""

    __slots__ = [ "board_id", "switch_count", "firmware_version", "driver_count", "pixel_strip_count", "platform", "input_state_bits" ]

    # pylint: disable-msg=too-many-arguments
    def __init__(self, board_id, switch_count, driver_count, pixel_strip_count, platform):
        """Initialise CanPinIoBoard."""
        self.board_id = board_id
        self.switch_count = switch_count
        self.firmware_version = None
        self.driver_count = driver_count
        self.pixel_strip_count = pixel_strip_count
        self.platform = platform
        self.input_state_bits = 0

    def get_name(self):
        return "CANPIN {}".format(self.board_id)

    def get_info_string(self) -> str:
        """Return description string."""
        return "Board {} - Firmware: {} Switches: {} Drivers: {} Pixel strips: {}".format(
            self.board_id,
            self.firmware_version,
            self.switch_count,
            self.driver_count,
            self.pixel_strip_count
        )


class CanPinDriver(DriverPlatformInterface):

    """Driver class for CANPIN output."""

    __slots__ = ["io_board", "_max_pulse_time", "_recycle_time", "index", "has_rule"]     # type: List[str]

    def __init__(self, config, number, index, io_board):
        """Initialise driver."""
        super().__init__(config, number)
        self.io_board = io_board
        self._max_pulse_time = None
        self._recycle_time = None
        self.index = index
        self.has_rule = False

    def get_board_name(self):
        return self.io_board.get_name()

    def configure_recycle(self, recycle_time):
        """Configure recycle time."""
        if recycle_time > 254:
            recycle_time = 254
        elif recycle_time < 0:
            recycle_time = 0

        if self._recycle_time != recycle_time:
            self._recycle_time = recycle_time
            self.io_board.send_cmd(CanPinMessages.SetRecycleTime,
                                   bytes([int(self.index), recycle_time]))

    def configure_max_pulse_ms(self, max_pulse_time):
        """Configure max pulse ms for this driver if it changed."""
        if max_pulse_time > 254:
            max_pulse_time = 254
        elif max_pulse_time < 0:
            max_pulse_time = 0
        if max_pulse_time != self._max_pulse_time:
            self._max_pulse_time = max_pulse_time
            self.io_board.send_cmd(CanPinMessages.SetMaxPulseTime,
                                    bytes([int(self.index), max_pulse_time]))

    def pulse(self, pulse_settings: PulseSettings):
        """Pulse driver."""
        if pulse_settings.power != 1.0:
            raise AssertionError("Pulse power != 1.0 is not supported.")
        pulse_time = pulse_settings.duration
        if pulse_time > 254:
            pulse_time = 254
        elif pulse_time < 0:
            pulse_time = 0
        self.io_board.send_cmd(CanPinMessages.CoilPulse, bytes([int(self.index), pulse_time]))

    def timed_enable(self, pulse_settings: PulseSettings, hold_settings: HoldSettings):
        """Pulse and enable the coil for an explicit duration."""
        raise NotImplementedError

    def enable(self, pulse_settings: PulseSettings, hold_settings: HoldSettings):
        """Enable driver."""
        raise NotImplementedError
        # del hold_settings
        # self._configure_pulse_ms(pulse_settings.duration)
        # self.io_board.send_cmd(CanPinDefines.SolenoidsSetSolenoidToOn, bytes([int(self.index)]))

    def disable(self):
        """Disable driver."""
        raise NotImplementedError
        # self.io_board.send_cmd(CanPinDefines.SolenoidsSetSolenoidToOff, bytes([int(self.index)]))

    def configure_hardware_rule(self, switch1, switch2, pulse_settings, recycle, hold_settings, flags1, flags2):
        """Configure hardware rule in for this driver."""
        if pulse_settings.duration > 254:
            raise AssertionError("Pulse settings to long for CANPIN protocol. Got pulse_settings: {}".format(
                pulse_settings))
        if pulse_settings.power > 1.0:
            raise AssertionError("Pulse power setting too high for CANPIN protocol. Got pulse_settings: {}".format(
                pulse_settings))
        if recycle > 254:
            raise AssertionError("Recycle settings to long for CANPIN protocol. Recycle: {}".format(recycle))

        if switch2:
            switch2_value = switch2.hw_switch.index + (0x80 if switch2.invert else 0)
        else:
            switch2_value = 0

        self.has_rule = True
        self.configure_recycle(recycle)

        data = bytearray([self.index,
                          switch1.hw_switch.index + (0x80 if switch1.invert else 0),
                          switch2_value,
                          int(pulse_settings.duration),
                          int(pulse_settings.power * 254),
                          int(hold_settings.power * 254) if hold_settings else 0,
                          flags1,
                          flags2])
        self.io_board.send_cmd(CanPinMessages.ConfigureHardwareRule, data)

    def clear_hw_rule(self):
        """Clear hw rule for driver."""
        if not self.has_rule:
            return
        self.has_rule = False

        data = bytearray([self.index,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
                          ])
        self.io_board.send_cmd(CanPinRs232Intf.ConfigureHardwareRule, data)


class CanPinSwitch(SwitchPlatformInterface):

    """Switch class for CANPIN input."""

    __slots__ = ["io_board", "index"]  # type: List[str]

    def __init__(self, config, io_board, number, platform):
        """Initialise switch."""
        super().__init__(config, number, platform)
        board_str, index_str = self.platform._parse_number(number)
        self.index = int(index_str)

        self.io_board = io_board

    def get_board_name(self):
        return self.io_board.get_name()


class CanPinHardwarePlatform(SwitchPlatform, DriverPlatform, LogMixin):

    """Platform class for the CANPIN hardware controller."""

    __slots__ = [ "config", "canbus_connection", "_watchdog_task", "hw_switch_data", "io_boards", "num_canpin_brd", "_poll_task", "_poll_response_received" ]

    def __init__(self, machine):
        """Initialise CanPin hardware platform.

        Args:
        ----
            machine: The main ``MachineController`` instance.
        """
        super().__init__(machine)

        print("self.machine.config " + str(self.machine.config.keys()))

        self.config = self.machine.config_validator.validate_config("canpin", self.machine.config['canpin'])
        self._configure_device_logging_and_debug("canpin", self.config)

        self._watchdog_task = None
        self.hw_switch_data = {}
        self.canbus_connection = None                       # type: CanPinBusCommunicator]
        self.io_boards = {}                                 # type: Dict[int, CanPinIoBoard]
        
        # # Only including responses that should be received
        # self.canpin_commands = {
        #     ord(CanPinRs232Intf.INV_CMD): self.inv_resp,
        #     ord(CanPinRs232Intf.EOM_CMD): self.eom_resp,
        #     ord(CanPinRs232Intf.START_BOARD_IO): self.read_start_board_io_resp,
        #     ord(CanPinRs232Intf.READ_GEN2_INP_CMD): self.read_gen2_inp_resp_initial
        # }

    def get_info_string(self):
        """Dump infos about boards."""
        infos = ""
        infos += "\nBoards:\n"
        for board in self.io_boards.values():
            infos += board.get_info_string() + "\n"
        return infos

    async def initialize(self):
        """Initialise platform."""
        await self._connect_to_hardware()

    def stop(self):
        """Stop platform and close connections."""
        self.canbus_connection = None

    async def start(self):
        """Start polling and listening for commands."""
        # start polling

        # start listening for commands
        await self.canbus_connection.start_read_loop()

    def __repr__(self):
        """Return str representation."""
        return '<Platform.CANPIN>'

    async def _connect_to_hardware(self):
        """Connect to Can-Bus.

        This process will cause the CanPinBusCommunicator to become bus 'master' and to register themselves.
        """
        #port_chain_serial_map = {v: k for k, v in self.config['chains'].items()}
        communicator = CanPinBusCommunicator(platform=self, port=self.config['port'], interface=self.config['interface'], baud=self.config['baud'])
        await communicator.connect()

    def register_processor_connection(self, communicator : CanPinBusCommunicator):
        self.canbus_connection = communicator

    def register_io_board(self, board_id):
        """Create and register IO board if it is new.  Ok for board to be already registered. """
        if board_id not in self.io_boards:
            if board_id not in self.config['boards']:
                raise AssertionError(f'Board connected to canbus that is not in config.  Board_id {board_id}')
            else:
                board_config = self.config['boards'][board_id]
                board_index = board_config['board_index']
                self.io_boards[board_id] = CanPinIoBoard(board_index, len(board_config['input_pins']), len(board_config['output_pins']), len(board_config['led_pins']), self)
                self.configure_io_board(board_id, self.io_boards[board_id])

    def configure_io_board(self, board_id, io_board: CanPinIoBoard):
        """Send commands to setup this board"""
        self.canbus_connection.send_board_index(io_board.board_id, io_board.board_index)

    def clear_hw_rule(self, switch: SwitchSettings, coil: DriverSettings):
        """Clear hw rule for driver."""
        pass

    def set_pulse_on_hit_and_enable_and_release_and_disable_rule(self, enable_switch: SwitchSettings,
                                                                 eos_switch: SwitchSettings, coil: DriverSettings,
                                                                 repulse_settings: Optional[RepulseSettings]):
        """Set pulse on hit and enable and release and disable rule on driver.
        Pulses a driver when a switch is hit. When the switch is released
        the pulse is canceled and the driver gets disabled. When the eos_switch is hit the pulse is canceled
        and the driver becomes disabled. Typically used on the main coil for dual-wound coil flippers with eos switch.
        """
        assert coil.hold_settings is None

    def set_pulse_on_hit_and_enable_and_release_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
        """Set pulse on hit and enable and release rule on driver."""
        """Pulse and enable a driver. Cancel pulse and enable if switch is released."""
        assert coil.hold_settings.power > 0

    def set_pulse_on_hit_and_release_and_disable_rule(self, enable_switch: SwitchSettings,
                                                      eos_switch: SwitchSettings, coil: DriverSettings,
                                                      repulse_settings: Optional[RepulseSettings]):
        """Set pulse on hit and enable and release and disable rule on driver.
        Pulses a driver when a switch is hit. When the switch is released
        the pulse is canceled and the driver gets disabled. When the eos_switch is hit the pulse is canceled
        and the driver becomes disabled. Typically used on the main coil for dual-wound coil flippers with eos switch.
        """
        assert coil.hold_settings is None

    def set_pulse_on_hit_and_release_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
        """Set pulse on hit and release rule to driver."""
        """Pulse a driver but cancel pulse when switch is released."""
        assert not coil.hold_settings or coil.hold_settings.power == 0

    def set_pulse_on_hit_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
        """Set pulse on hit rule on driver."""
        """Always do the full pulse. Even when the switch is released. Pulse is delayed accurately by the hardware."""
        assert not coil.hold_settings or coil.hold_settings.power == 0

    # def read_gen2_inp_resp_initial(self, chain_serial, msg):
    #     """Read initial switch states.

    #     Args:
    #     ----
    #         chain_serial: Serial of the chain which received the message.
    #         msg: Message to parse.
    #     """
    #     self.debug_log("read_gen2_inp_resp_initial")
    #     # Verify the CRC8 is correct
    #     if len(msg) < 7:
    #         raise AssertionError("Received too short initial input response: " + "".join(HEX_FORMAT % b for b in msg))
    #     crc8 = CanPinRs232Intf.calc_crc8_part_msg(msg, 0, 6)
    #     if msg[6] != ord(crc8):
    #         self._bad_crc(chain_serial, msg)
    #     else:
    #         board = msg[0]
    #         if board not in self.io_boards:
    #             self.log.warning("Got input response for invalid board at initial request: %s. Msg: %s.", msg[0],
    #                              "".join(HEX_FORMAT % b for b in msg))
    #             return

    #         io_board = self.io_boards[board]

    #         input_state_bits = (msg[2] << 24) | \
    #             (msg[3] << 16) | \
    #             (msg[4] << 8) | \
    #             msg[5]

    #         io_board.input_state_bits = input_state_bits

    #         for switch in range(io_board.switch_count):
    #             self.hw_switch_data[str(board) + '-' + str(switch)] = (input_state_bits >> switch) & 1

    # def read_gen2_inp_resp(self, chain_serial, msg):
    #     """Read switch changes.

    #     Args:
    #     ----
    #         chain_serial: Serial of the chain which received the message.
    #         msg: Message to parse.
    #     """
    #     # Single read gen2 input response.  Receive function breaks them down
    #     self.debug_log("read_gen2_inp_resp")

    #     # Verify the CRC8 is correct
    #     if len(msg) < 7:
    #         self.log.warning("Msg too short: %s.", "".join(HEX_FORMAT % b for b in msg))
    #         self.serial_connections[chain_serial].lost_synch()
    #         return

    #     crc8 = CanPinRs232Intf.calc_crc8_part_msg(msg, 0, 6)
    #     if msg[6] != ord(crc8):
    #         self._bad_crc(chain_serial, msg)
    #     else:
    #         board = msg[0]
    #         if board not in self.io_boards:
    #             self.log.warning("Got input response for invalid board: %s. Msg: %s.", msg[0],
    #                              "".join(HEX_FORMAT % b for b in msg))
    #             return
    #         io_board = self.io_boards[board]

    #         new_state = (msg[2] << 24) | \
    #             (msg[3] << 16) | \
    #             (msg[4] << 8) | \
    #             msg[5]

    #         # Update the state which holds inputs that are active
    #         changes = io_board.input_state_bits ^ new_state
    #         if changes != 0:
    #             print("input changes board %d %x", board, changes)
    #             curr_bit = 1
    #             for index in range(0, 32):
    #                 if (curr_bit & changes) != 0:
    #                     if (curr_bit & new_state) == 0:
    #                         self.machine.switch_controller.process_switch_by_num(
    #                             state=1,
    #                             num=str(board) + '-' + str(index),
    #                             platform=self)
    #                     else:
    #                         self.machine.switch_controller.process_switch_by_num(
    #                             state=0,
    #                             num=str(board) + '-' + str(index),
    #                             platform=self)
    #                 curr_bit <<= 1
    #         io_board.input_state_bits = new_state

    #     # we can continue to poll
    #     print("_poll_response_received keys: ", self._poll_response_received.keys())
    #     self._poll_response_received[chain_serial].set()

    # async def _poll_sender(self, serial_connection : CanPinBusCommunicator):
    #     """Poll switches."""
    #     chain_serial = serial_connection.chain_serial

    #     # Send initial poll
    #     for node_id in self.canpin_addr_arr[chain_serial]:
    #         print("Send poll to ", chain_serial, " len=", len(self.io_boards[node_id].read_input_msg))
    #         serial_connection.platform.send( node_id, self.io_boards[node_id].read_input_msg )

    #     while True:
    #         # wait for previous poll response
    #         timeout = 1 / self.config['poll_hz'] * 25
    #         try:
    #             await asyncio.wait_for(self._poll_response_received[chain_serial].wait(), timeout)
    #         except asyncio.TimeoutError:
    #             self.log.warning("Poll took more than %sms for %s", timeout * 1000, chain_serial)
    #         else:
    #             self._poll_response_received[chain_serial].clear()
    #         # send poll
    #         for node_id in self.canpin_addr_arr[chain_serial]:
    #             serial_connection.platform.send( node_id, self.io_boards[node_id].read_input_msg )
    #         #mjc self.send_to_processor(chain_serial, self.read_input_msg[chain_serial])
    #         await self.serial_connections[chain_serial].writer.drain()
    #         # the line above saturates the link and seems to overwhelm the hardware. limit it to 100Hz
    #         await asyncio.sleep(1 / self.config['poll_hz'])

    # def set_pulse_on_hit_and_enable_and_release_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
    #     """Set pulse on hit and enable and release rule on driver."""
    #     """Pulse and enable a driver. Cancel pulse and enable if switch is released."""
    #     assert coil.hold_settings.power > 0

    #     self._configure_hardware_rule(coil, enable_switch, None, 3, 0)

    # def set_pulse_on_hit_and_release_and_disable_rule(self, enable_switch: SwitchSettings,
    #                                                   eos_switch: SwitchSettings, coil: DriverSettings,
    #                                                   repulse_settings: Optional[RepulseSettings]):
    #     """Set pulse on hit and enable and release and disable rule on driver.

    #     Pulses a driver when a switch is hit. When the switch is released
    #     the pulse is canceled and the driver gets disabled. When the eos_switch is hit the pulse is canceled
    #     and the driver becomes disabled. Typically used on the main coil for dual-wound coil flippers with eos switch.
    #     """
    #     assert coil.hold_settings is None
    #     self._configure_hardware_rule(coil, enable_switch, eos_switch, 3, 2)

    # def set_pulse_on_hit_and_enable_and_release_and_disable_rule(self, enable_switch: SwitchSettings,
    #                                                              eos_switch: SwitchSettings, coil: DriverSettings,
    #                                                              repulse_settings: Optional[RepulseSettings]):
    #     """Set pulse on hit and enable and release and disable rule on driver.

    #     Pulses a driver when a switch is hit. Then enables the driver (may be with pwm). When the switch is released
    #     the pulse is canceled and the driver becomes disabled. When the eos_switch is hit the pulse is canceled
    #     and the driver becomes enabled (likely with PWM).
    #     Typically used on the coil for single-wound coil flippers with eos switch.
    #     """
    #     self._configure_hardware_rule(coil, enable_switch, eos_switch, 3, 2)

    # def set_pulse_on_hit_and_release_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
    #     """Set pulse on hit and release rule to driver."""
    #     """Pulse a driver but cancel pulse when switch is released."""
    #     assert not coil.hold_settings or coil.hold_settings.power == 0
    #     self._configure_hardware_rule(coil, enable_switch, None, 3, 0)

    # def set_pulse_on_hit_rule(self, enable_switch: SwitchSettings, coil: DriverSettings):
    #     """Set pulse on hit rule on driver."""
    #     """Always do the full pulse. Even when the switch is released. Pulse is delayed accurately by the hardware."""
    #     assert not coil.hold_settings or coil.hold_settings.power == 0
    #     self._configure_hardware_rule(coil, enable_switch, None, 1, 0)

    # def clear_hw_rule(self, switch: SwitchSettings, coil: DriverSettings):
    #     """Clear hw rule for driver."""
    #     del switch
    #     coil.hw_driver.clear_hw_rule()

    # # pylint: disable-msg=too-many-arguments
    # def _configure_hardware_rule(self, coil: DriverSettings, switch1: SwitchSettings,
    #                              switch2: Optional[SwitchSettings], flags1, flags2):
    #     """Configure hardware rule in CANPIN."""
    #     recycle = coil.pulse_settings.duration * 2 if coil.recycle else coil.pulse_settings.duration
    #     coil.hw_driver.configure_hardware_rule( switch1, switch2, coil.pulse_settings, recycle, coil.hold_settings, flags1, flags2 )

    @staticmethod
    def _parse_number(number):
        board_str, index_str = number.split("-")
        board = int(board_str)
        index = int(index_str)
        return (board, index)

    def _parse_number(self, number):
        try:
            board_str, index_str = number.split("-")
        except ValueError:
            self.raise_config_error(
                f"Could not parse pin number {number}. Please verify the number format is " +
                "board-index. Board and index should be a integer values.", 7)

        board = int(board_str)
        index = int(index_str)

        if board not in self.io_boards:
            raise AssertionError(f"Board {board} does not exist for driver {number}")

        return (board, index)

    def _parse_driver_number(self, number):

        board, driver = self._parse_number(number)
        if self.io_boards[board].driver_count <= driver:
            raise AssertionError(f"Board {board} only has {self.io_boards[board].driver_count} drivers. "
                                 "Driver: {number}")

        index = 0
        for board_number, board_obj in self.io_boards.items():
            if board_number >= board:
                continue
            index += board_obj.driver_count

        return Util.int_to_hex_string(index + driver)

    def _parse_switch_number(self, number):

        board, switch = self._parse_number(number)
        if self.io_boards[board].switch_count <= switch:
            raise AssertionError(f"Board {board} only has {self.io_boards[board].switch_count} switches. "
                                 "Switch: {number}")

        index = 0
        for board_number, board_obj in self.io_boards.items():
            if board_number >= board:
                continue
            index += board_obj.switch_count

        return Util.int_to_hex_string(index + switch)

    def configure_driver(self, config: DriverConfig, number: str, platform_settings: dict) -> DriverPlatformInterface:
        """Configure a driver."""

        if not number:
            raise AssertionError("Driver needs a number")

        board, driver_index = self._parse_number(number)

        print("configure_driver: " + number)
        io_board = self.io_boards[board]

        print("configure canpin driver")

        driver = CanPinDriver(config=config, number=number, index=driver_index, io_board=io_board)
        recycle_time = config.default_pulse_ms * 2 if config.default_recycle else config.default_pulse_ms
        driver.configure_recycle(recycle_time)
        if (config.max_pulse_ms):
            driver.configure_max_pulse_ms(config.max_pulse_ms)
        return driver

    def configure_switch(self, number: str, config: SwitchConfig, platform_config: dict) -> SwitchPlatformInterface:
        """Configure a switch."""

        if not number:
            raise AssertionError("Switch needs a number")

        board, driver = self._parse_number(number)
        print("configure_switch: ", number)
        # number = self._parse_switch_number(number)

        io_board = self.io_boards[board]

        if number not in self.hw_switch_data:
            raise AssertionError("Invalid switch number {}. Platform reports the following switches as "
                                 "valid: {}".format(number, list(self.hw_switch_data.keys())))

        return CanPinSwitch(config=config, io_board=io_board, number=number, platform=self)

    async def get_hw_switch_states(self):
        """Return hardware states."""
        return self.hw_switch_data
