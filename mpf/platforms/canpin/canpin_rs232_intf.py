"""Defines for CanPin platform."""
from typing import List


class CanPinRs232Intf:

    """Constants for CanPin serial protocol."""

    __slots__ = []  # type: List[str]

    GET_ENDPOINT_SER_NUM_CMD = b'\x00'
    SET_BOARD_INDEX = b'\x04'
    SET_BOARD_INPUTPINS = b'\x05'
    SET_BOARD_OUTPUTPINS = b'\x06'
    SET_BOARD_LEDPINS = b'\x07'
    START_BOARD_IO = b'\x08'
    
    READ_GEN2_INP_CMD = b'\x0c'
    GET_GEN2_CFG = b'\x0d'

    CoilSetRecycleTime = b'\x0e'
    CoilSetMaxPulseTime = b'\x0f'
    CoilPulse = b'\x10'
    ConfigureHardwareRule = b'\x11'

    #SET_SER_NUM_CMD = b'\x03'
    #RESET_CMD = b'\x04'
    #GO_BOOT_CMD = b'\x05'
    #CFG_SOL_CMD = b'\x06'
    #KICK_SOL_CMD = b'\x07'
    #CFG_INP_CMD = b'\x09'
    #SAVE_CFG_CMD = b'\x0b'
    #ERASE_CFG_CMD = b'\x0c'
    #SET_GEN2_CFG = b'\x0e'
    # CHNG_NEO_CMD = b'\x0f'
    # CHNG_NEO_COLOR = b'\x10'
    # CHNG_NEO_COLOR_TBL = b'\x11'
    # SET_NEO_COLOR_TBL = b'\x12'
    # INCAND_CMD = b'\x13'
    # CFG_IND_SOL_CMD = b'\x14'
    # CFG_IND_INP_CMD = b'\x15'
    # SET_IND_NEO_CMD = b'\x16'
    # SET_SOL_INP_CMD = b'\x17'
    # UPGRADE_OTHER_BRD = b'\x18'
    # READ_MATRIX_INP = b'\x19'
    # CFG_SOL_KICK_PWM = b'\x1b'

    INV_CMD = b'\xf0'
    ILLEGAL_CMD = b'\xfe'
    EOM_CMD = b'\xff'

    #CARD_ID_TYPE_MASK = b'\xf0'
    #CARD_ID_GEN2_CARD = b'\x20'
    #CARD_ID_CANPIN_CARD = b'\x30'

    # NUM_G2_WING_PER_BRD = 4
    # WING_SOL = b'\x01'
    # WING_INP = b'\x02'
    # WING_INCAND = b'\x03'
    # WING_SW_MATRIX_OUT = b'\x04'
    # WING_SW_MATRIX_IN = b'\x05'
    # WING_NEO = b'\x06'
    # WING_HI_SIDE_INCAND = b'\x07'
    # WING_NEO_SOL = b'\x08'
    # WING_SW_MATRIX_OUT_LOW_WING = b'\x0a'
    # WING_LAMP_MATRIX_COL_WING = b'\x0b'
    # WING_LAMP_MATRIX_ROW_WING = b'\x0c'

    # NUM_G2_INP_PER_BRD = 32
    # CFG_INP_STATE = b'\x00'
    # CFG_INP_FALL_EDGE = b'\x01'
    # CFG_INP_RISE_EDGE = b'\x02'

    # # Solenoid configuration constants
    # CFG_BYTES_PER_SOL = 3
    # INIT_KICK_OFFSET = 1
    # DUTY_CYCLE_OFFSET = 2
    # NUM_G2_SOL_PER_BRD = 16

    # CFG_SOL_DISABLE = b'\x00'
    # CFG_SOL_USE_SWITCH = b'\x01'
    # CFG_SOL_AUTO_CLR = b'\x02'
    # CFG_SOL_ON_OFF = b'\x04'
    # CFG_SOL_DLY_KICK = b'\x08'
    # CFG_SOL_USE_MTRX_INP = b'\x10'
    # CFG_SOL_CAN_CANCEL = b'\x20'

    # CFG_SOL_INP_REMOVE = b'\x80'

    # NUM_COLOR_TBL = 32
    # NEO_CMD_ON = 0x80
    # SERIAL_LED_CMD_FADE = 0x40

    # INCAND_ROT_LEFT = b'\x00'
    # INCAND_ROT_RIGHT = b'\x01'
    # INCAND_LED_ON = b'\x02'
    # INCAND_LED_OFF = b'\x03'
    # INCAND_BLINK_SLOW = b'\x04'
    # INCAND_BLINK_FAST = b'\x05'
    # INCAND_BLINK_OFF = b'\x06'
    # INCAND_SET_ON_OFF = b'\x07'

    # INCAND_SET_CMD = b'\x80'
    # INCAND_SET_ON = b'\x01'
    # INCAND_SET_BLINK_SLOW = b'\x02'
    # INCAND_SET_BLINK_FAST = b'\x04'

    CRC8_LOOKUP = [
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
        0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
        0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
        0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
        0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
        0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
        0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
        0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
        0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
        0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
        0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3]

    @staticmethod
    def calc_crc8_whole_msg(msg_chars):
        """Calculate CRC for message."""
        crc8_byte = 0xff
        for ind_char in msg_chars:
            ind_int = ind_char
            crc8_byte = CanPinRs232Intf.CRC8_LOOKUP[crc8_byte ^ ind_int]
        return bytes([crc8_byte])

    @staticmethod
    def calc_crc8_part_msg(msg_chars, start_index, num_chars):
        """Calculate CRC for part of a message."""
        crc8_byte = 0xff
        index = 0
        if len(msg_chars) < start_index + num_chars:
            raise AssertionError("String too short for {} chars of CRC: {}". format(
                num_chars,
                "".join(" 0x%02x" % b for b in msg_chars[start_index:])))
        while index < num_chars:
            ind_int = msg_chars[start_index + index]
            crc8_byte = CanPinRs232Intf.CRC8_LOOKUP[crc8_byte ^ ind_int]
            index += 1
        return bytes([crc8_byte])
