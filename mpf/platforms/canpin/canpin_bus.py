"""CanPin CanBus communicator."""
import can

from mpf.platforms.canpin.canpin_rs232_intf import CanPinRs232Intf
from mpf.platforms.base_serial_communicator import BaseSerialCommunicator, HEX_FORMAT
from mpf.core.utility_functions import Util

MYPY = False
if MYPY:    # pragma: no cover
    from mpf.platforms.canpin.canpin import CanPinHardwarePlatform   # pylint: disable-msg=cyclic-import,unused-import

class CanPinBusCommunicator:

    """Manages a CanBus connection to a chain of CanPin devices."""

    __slots__ = ["part_msg", "chain_serial", "_lost_synch", "read_task"]

    # pylint: disable=too-many-arguments
    def __init__(self, platform: "CanPinHardwarePlatform", port, baud) -> None:
        """Initialise CanBus Connection to CanPin Hardware."""
        self.part_msg = b""
        self.chain_serial = None    # type: str
        self._lost_synch = False
        self.read_task = None       # type: (method)
    
        print("CanPinBusCommunicator port: " + str(port) + " baud: " + str(baud))

        can_interface = 'can0'
        canbus = can.interface.Bus(can_interface, bustype='socketcan')
        can.Message(arbitration_id=0x7de,data=[0, 25, 0, 1, 3, 1, 4, 1])

        super().__init__(platform, port, baud)
        self.platform = platform    # hint the right type

    async def start_read_loop(self):
        """Start the read loop."""
        self.read_task = self.machine.clock.loop.create_task(self._socket_reader())
        self.read_task.add_done_callback(Util.raise_exceptions)

    async def _read_id(self):
        msg = bytearray([0, CanPinRs232Intf.GET_ENDPOINT_SER_NUM_CMD[0], 0x00, 0x00, 0x00, 0x00])
        msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
        msg.extend(CanPinRs232Intf.EOM_CMD)
        self.send(bytes(msg))

        resp = await self.read(8)
        if resp[7] != ord(CanPinRs232Intf.EOM_CMD):
            raise AssertionError("Failed to read ID from {}. Missing EOM.".format(self.port))

        if ord(CanPinRs232Intf.calc_crc8_whole_msg(resp[0:6])) != resp[6]:
            raise AssertionError("Failed to read ID from {}. Wrong CRC.".format(self.port))

        if resp[1] != 0:
            raise AssertionError("Failed to read ID from {}. Wrong CMD.".format(self.port))

        return (resp[2] << 24) + (resp[3] << 16) + (resp[4] << 8) + resp[5]

    async def _identify_connection(self):
        """Identify which processor this serial connection is talking to."""
        # keep looping and wait for an ID response
        count = 0
        # read and discard all messages in buffer

        print("CanPinSerialCommunicator _identify_connection")

        self.send(CanPinRs232Intf.EOM_CMD)
        await asyncio.sleep(.01)
        await self.read(1000)
        while True:
            if (count % 10) == 0:
                self.log.debug("Sending EOM command to port '%s'",
                               self.port)
            count += 1
            self.send(CanPinRs232Intf.EOM_CMD)
            await asyncio.sleep(.01)
            resp = await self.read(30)
            if resp.startswith(CanPinRs232Intf.EOM_CMD):
                break
            if count == 100:
                raise AssertionError('No response from CanPin hardware: {}'.format(self.port))

        self.log.debug("Got ID response: %s", "".join(HEX_FORMAT % b for b in resp))

        # get ID from hardware
        connection_device_id = await self._read_id()
        self.chain_serial = '%x' % connection_device_id

        if self.chain_serial in self.platform.io_boards:
            raise AssertionError("Duplicate chain serial {} on ports: {} and {}. Each CanPin board has to have a "
                                 "unique ID. You can overwrite this using the chains (NOT CURRENTLY IMPLEMENTED) "
                                 "setting.".format(self.chain_serial, self.port,
                                                   self.platform.canpin_connection[self.chain_serial]))

        # Send inventory command to figure out number of cards
        msg = bytearray()
        msg.extend(CanPinRs232Intf.INV_CMD)
        msg.extend(CanPinRs232Intf.EOM_CMD)
        cmd = bytes(msg)

        self.log.debug("Sending inventory command: %s", "".join(HEX_FORMAT % b for b in cmd))
        self.send(cmd)

        resp = await self.readuntil(b'\xff')

        # resp will contain the inventory response.
        self.platform.process_received_message(self.chain_serial, resp)

        # now we can set the board indices
        self.send_board_index_cmd()

        # and now send the pin configs for the known boards
        self.send_board_configration_cmd()

        # and start up the io!
        self.send_start_board_io_cmd()

        """Read response to the 'START_BOARD_IO' command."""
        for board_id in self.platform.canpin_boardid_arr[self.chain_serial]:
            self.platform.log.warning("Reading START_BOARD_IO response: ")
            resp = await self.readuntil(b'\xff', 4)
            self.platform.log.warning("Got START_BOARD_IO response: %s", "".join(HEX_FORMAT % b for b in resp))
            self.platform.process_received_message(self.chain_serial, resp)
        self.platform.log.warning("done reading START_BOARD_IO response: ")

        # # Now send get gen2 configuration message to find populated wing boards
        # self.send_get_gen2_cfg_cmd()
        # resp = await self.readuntil(b'\xff', 6)

        # # resp will contain the gen2 cfg responses.  That will end up creating all the
        # # correct objects.
        # self.platform.process_received_message(self.chain_serial, resp)

        # Rather than doing the gen2 configuration request just register based on the boards we know and their setup yaml.
        self.platform.register_known_io_boards(self.chain_serial)

        # get initial value for inputs
        self.log.debug("Getting initial inputs states for %s", self.chain_serial)
        for io_board_id, io_board in self.platform.io_boards.items():
            self.send(io_board.read_input_msg)
            resp = await self.readuntil(b'\xff')
            self._parse_msg(resp)

        self.log.info("Init of CanPin connection %s done", self.chain_serial)
        self.platform.register_processor_connection(self.chain_serial, self)

    def send_get_gen2_cfg_cmd(self):
        """Send get gen2 configuration message to find populated wing boards."""
        whole_msg = bytearray()
        for card_addr in self.platform.canpin_addr_arr[self.chain_serial]:
            msg = bytearray()
            msg.append(card_addr)
            msg.extend(CanPinRs232Intf.GET_GEN2_CFG)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.append(0)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

        cmd = bytes(whole_msg)
        self.log.debug("Sending get Gen2 Cfg command: %s", "".join(HEX_FORMAT % b for b in cmd))
        self.send(cmd)

    def send_board_index_cmd(self):
        """Send message to all the known boards to set the board's index."""
        whole_msg = bytearray()
        for board_id_str in self.platform.canpin_boardid_arr[self.chain_serial]:
            board_index = self.platform.config['boards'][board_id_str]['board_index']
            board_id = int(board_id_str, 16)

            msg = bytearray()
            msg.append(board_index)
            msg.extend(CanPinRs232Intf.SET_BOARD_INDEX)
            msg.append((board_id >> 24) & 0xff)
            msg.append((board_id >> 16) & 0xff)
            msg.append((board_id >>  8) & 0xff)
            msg.append((board_id      ) & 0xff)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

        cmd = bytes(whole_msg)
        self.log.debug("Sending set board index command: %s", "".join(HEX_FORMAT % b for b in cmd))
        self.send(cmd)

    def send_board_configration_cmd(self):
        """Send message to all the known boards to set the board's pin configurations."""
        whole_msg = bytearray()
        for board_id in self.platform.canpin_boardid_arr[self.chain_serial]:
            board_config = self.platform.config['boards'][board_id]
            board_index = board_config['board_index']

            msg = bytearray()
            msg.append(board_index)
            msg.extend(CanPinRs232Intf.SET_BOARD_INPUTPINS)
            for pin in board_config['input_pins']:
                msg.append(pin)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

            msg = bytearray()
            msg.append(board_index)
            msg.extend(CanPinRs232Intf.SET_BOARD_OUTPUTPINS)
            for pin in board_config['output_pins']:
                msg.append(pin)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

            msg = bytearray()
            msg.append(board_index)
            msg.extend(CanPinRs232Intf.SET_BOARD_LEDPINS)
            for pin in board_config['led_pins']:
                msg.append(pin)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

        cmd = bytes(whole_msg)
        self.log.debug("Sending set board pin commands: %s", "".join(HEX_FORMAT % b for b in cmd))
        self.send(cmd)

    def send_start_board_io_cmd(self):
        """Send message to all the known boards to start their io (pins must not be changed after this poi)."""
        whole_msg = bytearray()
        for board_id in self.platform.canpin_boardid_arr[self.chain_serial]:
            board_config = self.platform.config['boards'][board_id]
            board_index = board_config['board_index']

            msg = bytearray()
            msg.append(board_index)
            msg.extend(CanPinRs232Intf.START_BOARD_IO)
            msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
            msg.extend(CanPinRs232Intf.EOM_CMD)
            whole_msg.extend(msg)

        cmd = bytes(whole_msg)
        self.log.debug("Sending start board io commands: %s", "".join(HEX_FORMAT % b for b in cmd))
        self.send(cmd)

    @classmethod
    def _create_vers_str(cls, version_int):     # pragma: no cover
        return ("%02d.%02d.%02d.%02d" % (((version_int >> 24) & 0xff),
                                         ((version_int >> 16) & 0xff), ((version_int >> 8) & 0xff),
                                         (version_int & 0xff)))

    def lost_synch(self):
        """Mark connection as desynchronised."""
        self._lost_synch = True

    def _parse_msg(self, msg):
        self.part_msg += msg
        strlen = len(self.part_msg)
        message_found = 0
        # Split into individual responses
        while strlen > 2:
            if self._lost_synch:
                while strlen > 0:
                    # wait for next gen2 card message
                    if (self.part_msg[0] & 0xe0) == 0x20:
                        self._lost_synch = False
                        break
                    self.part_msg = self.part_msg[1:]
                    strlen -= 1
            elif self.part_msg[0] == ord(CanPinRs232Intf.EOM_CMD):
                self.part_msg = self.part_msg[1:]
                strlen -= 1
            else:
                # Check if read input
                if self.part_msg[1] == ord(CanPinRs232Intf.READ_GEN2_INP_CMD):
                    if strlen >= 7:
                        self.platform.process_received_message(self.chain_serial, self.part_msg[:7])
                        message_found += 1
                        self.part_msg = self.part_msg[7:]
                        strlen -= 7
                    else:
                        # message not complete yet
                        break
                # Check if read matrix input
                # elif self.part_msg[1] == ord(CanPinRs232Intf.READ_MATRIX_INP):
                #     if strlen >= 11:
                #         self.platform.process_received_message(self.chain_serial, self.part_msg[:11])
                #         message_found += 1
                #         self.part_msg = self.part_msg[11:]
                #         strlen -= 11
                #     else:
                #         # message not complete yet
                #         break
                else:
                    # Lost synch
                    self.part_msg = self.part_msg[2:]
                    strlen -= 2
                    self._lost_synch = True

        return message_found
