"""CanPin CanBus communicator."""
import can
import asyncio

from mpf.platforms.canpin.canpin_rs232_intf import CanPinRs232Intf
from mpf.platforms.base_serial_communicator import BaseSerialCommunicator, HEX_FORMAT
from mpf.core.utility_functions import Util
from mpf.platforms.canpin.defines import CanPinMessages

MYPY = False
if MYPY:    # pragma: no cover
    from mpf.platforms.canpin.canpin import CanPinHardwarePlatform   # pylint: disable-msg=cyclic-import,unused-import


class CanPinMessage(can.Message):
    def __init__(self, message_type, board_index, data):
        super().__init__(is_extended_id=False, data=data)
        self.arbitration_id = (message_type<<5) | (board_index&0x1f)


class CanPinBusCommunicator(can.Listener):

    """Manages a CanBus connection to a chain of CanPin devices."""

    __slots__ = ["canbus", "platform"]

    # pylint: disable=too-many-arguments
    def __init__(self, platform: "CanPinHardwarePlatform", port, interface, baud) -> None:
        """Initialise CanBus Connection to CanPin Hardware."""
    
        print("CanPinBusCommunicator port: " + str(port) + " interface: " + str(interface) + " baud: " + str(baud))
        #bustype='socketcan', 
        self.canbus = can.Bus(channel=port, interface=interface, ignore_config=False, receive_own_messages=False)
        self.ready_id=0
        self.board_id = 0xdffffff7
        self.boards = {}

        self.platform = platform

    async def connect(self):

        printer = can.Printer()
        self.canbus_notifier = can.Notifier(self.canbus, [printer, self])

        self.ready_id += 1
        self.send_cmd( CanPinMessages.DeviceReady, 0, [self.board_id&0xff,(self.board_id>>8)&0xff,(self.board_id>>16)&0xff,self.board_id>>24,self.ready_id&0xff,self.ready_id>>8])
        await asyncio.sleep(10)

    def send_board_index(self, board_id, board_index):
        send_cmd( CanPinMessages.AssignDeviceIndex, 0, [board_id&0xff,(board_id>>8)&0xff,(board_id>>16)&0xff,board_id>>24,board_index])

    def send_cmd( self, command, board_index, data):
        self.canbus.send( CanPinMessage(command, board_index, data) )

    async def start_read_loop(self):
        """Start the read loop."""
        pass

    def on_message_received(self, msg):
        if not msg.is_error_frame and not msg.is_remote_frame:
            # msg.is_rx
            # msg.arbitration_id
            # msg.dlc (data size)
            # msg.data (up to 8 bytes of data)
            message_id = msg.arbitration_id>>5
            recipient_id = msg.arbitration_id & 0x1f
            print("on_message_received " + str(message_id) + "\n")

            if message_id == CanPinMessages.DeviceReady:
                pass
            elif message_id == CanPinMessages.WelcomeNewHost:
                host_id = msg.data[0] | (msg.data[1]<<8) | (msg.data[2]<<16) | (msg.data[3]<<24)
                voter_id = msg.data[4] | (msg.data[5]<<8) | (msg.data[6]<<16) | (msg.data[7]<<24)
                if host_id != self.board_id:
                    print(f'Expecting that we are the host (id {self.board_id}) but {voter_id} is voting for {host_id}')
                else:
                    if not voter_id in self.boards:
                        self.boards[voter_id] = {}
                        print(f'Registering board {voter_id}')
                        self.platform.register_io_board(voter_id)
            else:
                print(f'Unhandled message_id {message_id}')

    # def send_get_gen2_cfg_cmd(self):
    #     """Send get gen2 configuration message to find populated wing boards."""
    #     whole_msg = bytearray()
    #     for card_addr in self.platform.canpin_addr_arr[self.chain_serial]:
    #         msg = bytearray()
    #         msg.append(card_addr)
    #         msg.extend(CanPinRs232Intf.GET_GEN2_CFG)
    #         msg.append(0)
    #         msg.append(0)
    #         msg.append(0)
    #         msg.append(0)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #     cmd = bytes(whole_msg)
    #     self.log.debug("Sending get Gen2 Cfg command: %s", "".join(HEX_FORMAT % b for b in cmd))
    #     self.send(cmd)

    # def send_board_index_cmd(self):
    #     """Send message to all the known boards to set the board's index."""
    #     whole_msg = bytearray()
    #     for board_id_str in self.platform.canpin_boardid_arr[self.chain_serial]:
    #         board_index = self.platform.config['boards'][board_id_str]['board_index']
    #         board_id = int(board_id_str, 16)

    #         msg = bytearray()
    #         msg.append(board_index)
    #         msg.extend(CanPinRs232Intf.SET_BOARD_INDEX)
    #         msg.append((board_id >> 24) & 0xff)
    #         msg.append((board_id >> 16) & 0xff)
    #         msg.append((board_id >>  8) & 0xff)
    #         msg.append((board_id      ) & 0xff)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #     cmd = bytes(whole_msg)
    #     self.log.debug("Sending set board index command: %s", "".join(HEX_FORMAT % b for b in cmd))
    #     self.send(cmd)

    # def send_board_configration_cmd(self):
    #     """Send message to all the known boards to set the board's pin configurations."""
    #     whole_msg = bytearray()
    #     for board_id in self.platform.canpin_boardid_arr[self.chain_serial]:
    #         board_config = self.platform.config['boards'][board_id]
    #         board_index = board_config['board_index']

    #         msg = bytearray()
    #         msg.append(board_index)
    #         msg.extend(CanPinRs232Intf.SET_BOARD_INPUTPINS)
    #         for pin in board_config['input_pins']:
    #             msg.append(pin)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #         msg = bytearray()
    #         msg.append(board_index)
    #         msg.extend(CanPinRs232Intf.SET_BOARD_OUTPUTPINS)
    #         for pin in board_config['output_pins']:
    #             msg.append(pin)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #         msg = bytearray()
    #         msg.append(board_index)
    #         msg.extend(CanPinRs232Intf.SET_BOARD_LEDPINS)
    #         for pin in board_config['led_pins']:
    #             msg.append(pin)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #     cmd = bytes(whole_msg)
    #     self.log.debug("Sending set board pin commands: %s", "".join(HEX_FORMAT % b for b in cmd))
    #     self.send(cmd)

    # def send_start_board_io_cmd(self):
    #     """Send message to all the known boards to start their io (pins must not be changed after this poi)."""
    #     whole_msg = bytearray()
    #     for board_id in self.platform.canpin_boardid_arr[self.chain_serial]:
    #         board_config = self.platform.config['boards'][board_id]
    #         board_index = board_config['board_index']

    #         msg = bytearray()
    #         msg.append(board_index)
    #         msg.extend(CanPinRs232Intf.START_BOARD_IO)
    #         msg.extend(CanPinRs232Intf.calc_crc8_whole_msg(msg))
    #         msg.extend(CanPinRs232Intf.EOM_CMD)
    #         whole_msg.extend(msg)

    #     cmd = bytes(whole_msg)
    #     self.log.debug("Sending start board io commands: %s", "".join(HEX_FORMAT % b for b in cmd))
    #     self.send(cmd)

    @classmethod
    def _create_vers_str(cls, version_int):     # pragma: no cover
        return ("%02d.%02d.%02d.%02d" % (((version_int >> 24) & 0xff),
                                         ((version_int >> 16) & 0xff), ((version_int >> 8) & 0xff),
                                         (version_int & 0xff)))

    # def lost_synch(self):
    #     """Mark connection as desynchronised."""
    #     self._lost_synch = True

    # def _parse_msg(self, msg):
    #     self.part_msg += msg
    #     strlen = len(self.part_msg)
    #     message_found = 0
    #     # Split into individual responses
    #     while strlen > 2:
    #         if self._lost_synch:
    #             while strlen > 0:
    #                 # wait for next gen2 card message
    #                 if (self.part_msg[0] & 0xe0) == 0x20:
    #                     self._lost_synch = False
    #                     break
    #                 self.part_msg = self.part_msg[1:]
    #                 strlen -= 1
    #         elif self.part_msg[0] == ord(CanPinRs232Intf.EOM_CMD):
    #             self.part_msg = self.part_msg[1:]
    #             strlen -= 1
    #         else:
    #             # Check if read input
    #             if self.part_msg[1] == ord(CanPinRs232Intf.READ_GEN2_INP_CMD):
    #                 if strlen >= 7:
    #                     self.platform.process_received_message(self.chain_serial, self.part_msg[:7])
    #                     message_found += 1
    #                     self.part_msg = self.part_msg[7:]
    #                     strlen -= 7
    #                 else:
    #                     # message not complete yet
    #                     break
    #             # Check if read matrix input
    #             # elif self.part_msg[1] == ord(CanPinRs232Intf.READ_MATRIX_INP):
    #             #     if strlen >= 11:
    #             #         self.platform.process_received_message(self.chain_serial, self.part_msg[:11])
    #             #         message_found += 1
    #             #         self.part_msg = self.part_msg[11:]
    #             #         strlen -= 11
    #             #     else:
    #             #         # message not complete yet
    #             #         break
    #             else:
    #                 # Lost synch
    #                 self.part_msg = self.part_msg[2:]
    #                 strlen -= 2
    #                 self._lost_synch = True

    #     return message_found
