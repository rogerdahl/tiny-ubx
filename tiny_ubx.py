#!/usr/bin/env python3

"""A tiny library for configuring u-blox GPS modules

UBX is the proprietary protocol used for configuring u-blox GPS modules. This creates the byte streams for some of the
most common messages and writes them to a device connected by serial or a USB to serial adapter.

Sample NMEA messages sent by a u-blox GPS module that has not obtained a location fix yet.

    $GNRMC,043250.00,V,,,,,,,020620,,,N*65
    $GNGGA,043250.00,,,,,0,00,99.99,,,,,,*78
    $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
    $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
    $GPGSV,1,1,01,27,,,15*79
    $GLGSV,1,1,00*65
    $GNGLL,,,,,043250.00,V,N*54
"""
import logging
import pprint

import struct
import binascii
import time

import serial

log = logging.getLogger(__name__)


CFG_MSG_PACK_LIST = (
    ("B", "msgClass"),
    ("B", "msgID"),
    ("B", "rate"),
)

CFG_MSG_PACK_LIST_MULTI = (
    ("B", "msgClass"),
    ("B", "msgID"),
    ("B", "rate1"),
    ("B", "rate2"),
    ("B", "rate3"),
    ("B", "rate4"),
    ("B", "rate5"),
    ("B", "rate6"),
)

CFG_PRT_PACK_LIST = (
    ("B", "portId"),
    ("B", "reserved0"),
    ("H", "txReady"),
    ("I", "mode"),
    ("I", "baudRate"),
    ("H", "inProtoMask"),
    ("H", "outProtoMask"),
    ("H", "reserved4"),
    ("H", "reserved5"),
)

CFG_PRT_IN_LIST = (("B", "portId"),)

CFG_RST_PACK_LIST = (
    ("H", "navBbrMask"),
    ("B", "resetMode"),
    ("B", "reserved1"),
)

CFG_RATE_PACK_LIST = (
    ("H", "measRate"),
    ("H", "navRate"),
    ("H", "timeRef"),
)

CFG_CFG_PACK_LIST = (
    ("I", "clearMask"),
    ("I", "saveMask"),
    ("I", "loadMask"),
    ("B", "deviceMask"),
)

CFG_NMEA_PACK_LIST = (
    ("B", "filter"),
    ("B", "version"),
    ("B", "numSV"),
    ("B", "flags"),
)

UBX_SYNC_BYTES = 0xB5, 0x62

NMEA_MSG_DICT = {
    # # Proprietary Messages
    "UBX_00": (0xF1, 0x00),  # Lat/Long Position Data
    "UBX_03": (0xF1, 0x03),  # Satellite Status
    "UBX_04": (0xF1, 0x04),  # Time of Day and Clock Information
    # "UBX_05": (0xF1, 0x05),  # Lat/Long Position Data
    # "UBX_06": (0xF1, 0x06),  # Lat/Long Position Data
    # "UBX_40": (0xF1, 0x40),  # Set NMEA message output rate
    # "UBX_41": (0xF1, 0x41),  # Set Protocols and Baudrate
    # # Standard NMEA Messages
    "DTM": (0xF0, 0x0A),  # Datum Reference
    "GBS": (0xF0, 0x09),  # GNSS Satellite Fault Detection
    "GGA": (0xF0, 0x00),  # Global positioning system fix data
    "GLL": (0xF0, 0x01),  # Latitude and longitude, with time of position fix and status
    # "GPQ": (0xF0, 0x40),  # Poll message
    "GRS": (0xF0, 0x06),  # GNSS Range Residuals
    "GSA": (0xF0, 0x02),  # GNSS DOP and Active Satellites
    "GST": (0xF0, 0x07),  # GNSS Pseudo Range Error Statistics
    "GSV": (0xF0, 0x03),  # GNSS Satellites in View
    "RMC": (0xF0, 0x04),  # Recommended Minimum data
    # "THS": (0xF0, 0x0E),  # True Heading and Status
    # "TXT": (0xF0, 0x41),  # Text Transmission
    "VTG": (0xF0, 0x05),  # Course over ground and Ground speed
}

TIMEOUT_SEC = 5.0


class TinyUbx:
    # Flags for CFG clear_mask, save_mask, load_mask
    CLEAR_IO_PORT = 1 << 0  # Port Settings
    CLEAR_MSG_CONF = 1 << 1  # Message Configuration
    CLEAR_INF_MSG = 1 << 2  # INF Message Configuration
    CLEAR_NAV_CONF = 1 << 3  # Navigation Configuration
    CLEAR_RXM_CONF = 1 << 4  # Receiver Manager Configuration
    CLEAR_RINV_CONF = 1 << 9  # Remote Inventory Configuration
    CLEAR_ANT_CONF = 1 << 10  # Antenna Configuration
    CLEAR_ALL = (
        CLEAR_IO_PORT
        | CLEAR_MSG_CONF
        | CLEAR_INF_MSG
        | CLEAR_NAV_CONF
        | CLEAR_RXM_CONF
        | CLEAR_RINV_CONF
        | CLEAR_ANT_CONF
    )

    # Flags for CFG device_mask
    DEVICE_DEV_BBR = 1 << 0  # devSpiFlash device battery backed RAM
    DEVICE_DEV_FLASH = 1 << 1  # device Flash
    DEVICE_DEV_EEPROM = 1 << 2  # device EEPROM
    DEVICE_DEVICE_SPI_FLASH = 1 << 4  # device SPI Flash
    DEVICE_ALL = (
        DEVICE_DEV_BBR | DEVICE_DEV_FLASH | DEVICE_DEV_EEPROM | DEVICE_DEVICE_SPI_FLASH
    )

    def __init__(self, device_path="/dev/ttyUSB0", baud_rate=9600):
        self._device_path = device_path
        self._baud_rate = baud_rate

    def __enter__(self):
        self._open_device()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._close_device()
        if exc_val:
            raise

    def _open_device(self):
        self.dev_f = serial.Serial(
            self._device_path, self._baud_rate, timeout=TIMEOUT_SEC
        )
        log.info(
            f"Opened device: {self.dev_f.name}"
            f'{f" (aliased from {self._device_path})" if self.dev_f.name != self._device_path else ""}'
        )

    def _close_device(self):
        self.dev_f.close()

    def reset_to_factory(self):
        """Reset to factory settings.
        """
        log.debug("Reset to factory settings")
        # Order of execution is clear, save, load. This will copy the factory default
        # settings from ROM to flash, load them from flash, and activate them.
        self.cfg(
            clear_mask=self.CLEAR_ALL,
            save_mask=0,
            load_mask=self.CLEAR_ALL,
            device_mask=self.DEVICE_ALL,
        )
        self.reset_to_cold()

    # CFG-CFG

    def cfg(self, clear_mask=0, save_mask=0, load_mask=0, device_mask=0):
        """Clear, Save and Load configurations.

        Clear is reset to factory settings. Use the class flag constants.
        Order of execution is clear, save, load.
        """
        log.debug("CFG-CFG")
        log.debug(f"clear_mask : {clear_mask:032b}")
        log.debug(f"save_mask  : {save_mask:032b}")
        log.debug(f"load_mask  : {load_mask:032b}")
        log.debug(f"device_mask: {device_mask:8b}")
        self.set_cfg(
            CFG_CFG_PACK_LIST,
            0x06,
            0x09,
            clearMask=clear_mask,
            saveMask=save_mask,
            loadMask=load_mask,
            deviceMask=device_mask,
        )

    # CFG-RST

    def stop(self):
        log.debug("CFG-RST - Stop processing")
        self.set_cfg(CFG_RST_PACK_LIST, 0x06, 0x04, navBbrMask=0x0000, resetMode=0x08)

    def start(self):
        log.debug("CFG-RST - Start processing")
        self.set_cfg(CFG_RST_PACK_LIST, 0x06, 0x04, navBbrMask=0x0000, resetMode=0x09)

    def reset_to_cold(self):
        """Reset to cold start.
        """
        log.debug("CFG-RST - Reset to cold start")
        self.set_cfg(CFG_RST_PACK_LIST, 0x06, 0x04, navBbrMask=0xFFFF, resetMode=0x01)

    def reset_to_warm(self):
        """Reset to warm start.
        """
        log.debug("CFG-RST - Reset to warm start")
        self.set_cfg(CFG_RST_PACK_LIST, 0x06, 0x04, navBbrMask=0x0001, resetMode=0x01)

    def reset_to_hot(self):
        """Reset to hot start.
        """
        log.debug("CFG-RST - Reset to hot start")
        self.set_cfg(CFG_RST_PACK_LIST, 0x06, 0x04, navBbrMask=0x0000, resetMode=0x01)

    # CFG-MSG

    def set_msg_rate_all_current(self, rate=0):
        """Set the output rate for all messages on current port"""
        for msg_name in NMEA_MSG_DICT:
            self.set_msg_rate_single_current(msg_name, rate)

    def set_msg_rate_single_current(self, msg_name, rate):
        log.debug(f"Setting message rate for: {msg_name}")
        msg_class, msg_id = NMEA_MSG_DICT[msg_name]
        self.set_cfg(
            CFG_MSG_PACK_LIST, 0x06, 0x01, msgClass=msg_class, msgID=msg_id, rate=rate
        )

    def set_msg_rate_all(self, rate_int):
        """Set the output rate for all messages on all ports"""
        for msg_name, id_tup in NMEA_MSG_DICT.items():
            msg_class, msg_id = id_tup
            self.set_msg_rate_single(msg_name, rate_int)

    def set_msg_rate_single(self, msg_name, rate):
        log.debug(f"Setting message rate on all ports for: {msg_name}")
        msg_class, msg_id = NMEA_MSG_DICT[msg_name]
        self.set_cfg(
            CFG_MSG_PACK_LIST_MULTI,
            0x06,
            0x01,
            msgClass=msg_class,
            msgID=msg_id,
            rate1=rate,
            rate2=rate,
            rate3=rate,
            rate4=rate,
            rate5=rate,
            rate6=rate,
        )

    # CFG-RATE

    def prt_poll_update_rate(self):
        log.debug("CFG-RATE poll")
        self.poll_cfg({}, CFG_RATE_PACK_LIST, 0x06, 0x08)

    def prt_set_update_rate(self, delay_ms):
        """Set how often an updated block of NMEA messages should be sent by the GPS module.

        :param delay_ms: (int) Number of milliseconds to wait between the start of each each update. A value below 1000
        causes messages to be sent more than once per second.

        The serial baud rate, number of active messages and update rate must be configured in such a way that there's time
        for the module to write the requested messages at the requested frequency.
        """
        log.debug("CFG-RATE set")
        self.set_cfg(
            CFG_RATE_PACK_LIST, 0x06, 0x08, measRate=delay_ms, navRate=1, timeRef=1
        )

    # CFG-PRT (UART, I2C, etc)

    def prt_poll(self, port_id=None):
        """Poll the configuration of the current or given I/O Port"""
        return self.poll_cfg(
            CFG_PRT_IN_LIST, CFG_PRT_PACK_LIST, 0x06, 0x00, portId=port_id
        )

    def prt_set_message_types(
        self,
        port_id=1,
        ubx_input=True,
        ubx_output=True,
        nmea_input=True,
        nmea_output=True,
    ):
        """Enable/disable UBX/NMEA input/output on the port"""
        UBX_BIT = 1
        NMEA_BIT = 1 << 1
        in_proto_mask = (UBX_BIT if ubx_input else 0) | (NMEA_BIT if nmea_input else 0)
        out_proto_mask = (UBX_BIT if ubx_output else 0) | (
            NMEA_BIT if nmea_output else 0
        )
        payload_dict = self.prt_poll(port_id)
        payload_dict.update(
            {"inProtoMask": in_proto_mask, "outProtoMask": out_proto_mask}
        )
        self.set_cfg(CFG_PRT_PACK_LIST, 0x06, 0x00, **payload_dict)

    def set_baud_rate(self, baud_rate=115200):
        """Set the baud rate at which the GPS module will receive and send messages through the serial interface.

        The underlying serial connection is reopened with the new baud rate. This is a no-op if the serial connection
        is already using the given baud rate.
        """
        if baud_rate == self._baud_rate:
            log.debug(f"Already at {baud_rate} bps")
            return
        log.debug(f"Setting baudrate to {baud_rate} bps")
        self.update_cfg(
            CFG_PRT_PACK_LIST, 0x06, 0x00, check_response=False, baudRate=baud_rate
        )
        self._baud_rate = baud_rate
        # TODO: Check how much time is needed.
        time.sleep(5)
        self._close_device()
        time.sleep(5)
        self._open_device()

    def prt_set(self, **payload_dict):
        """Configure port.
        portId
        txReady
        mode
        baudRate
        inProtoMask
        outProtoMask
        """
        self.set_cfg(
            CFG_PRT_PACK_LIST, 0x06, 0x00, **{**self.prt_poll(), **payload_dict}
        )

    # CFG-NMEA

    def nmea_protocol_config(
        self,
        disable_pos_filter=False,
        disable_msk_pos_filter=False,
        disable_time_filter=False,
        disable_date_filter=False,
        disable_sbas_filter=False,
        disable_track_filter=False,
        compatibility_mode=False,
        considering_mode=False,
    ):
        """
        pos_filter (bool): Disable position filtering
        msk_pos_filter (bool): Disable masked position filtering
        time_filter (bool): Disable time filtering
        date_filter (bool): Disable date filtering
        sbas_filter (bool): Enable SBAS filtering
        track_filter (bool): Disable track filtering
        compatibility_mode (bool): Enable compatibility mode. This might be needed for certain applications when customer's NMEA parser expects a fixed number of digits in position coordinates
        considering_mode (bool): Enable considering mode.
        """
        log.debug(f"CFG-NMEA")

        #
        filter = (
            (int(disable_pos_filter) << 0)
            | (int(disable_msk_pos_filter) << 1)
            | (int(disable_time_filter) << 2)
            | (int(disable_date_filter) << 3)
            | (int(disable_sbas_filter) << 4)
            | (int(disable_track_filter) << 5)
        )
        self.set_cfg(
            CFG_NMEA_PACK_LIST,
            0x06,
            0x17,
            filter=0xFF,
            version=0x23,  # NMEA version 2.3
            numSV=0xFF,
            flags=((int(compatibility_mode) << 0) | int(considering_mode) << 1),
        )

    # Misc

    def get_antenna(self):
        return self._read_ubx_message(0x06, 0x13, "HH")

    def update_cfg(self, pack_list, msg_class, msg_id, **payload_dict):
        log.debug("CFG update")
        check_response = payload_dict.pop("check_response", False)
        old_dict = self.poll_cfg({}, pack_list, msg_class, msg_id)
        self.set_cfg(
            pack_list, msg_class, msg_id, **{**self.prt_poll(), **payload_dict},
        )

    def set_cfg(self, pack_list, msg_class, msg_id, **payload_dict):
        """Set configuration (CFG) value(s)"""
        log.debug("CFG set")
        check_response = payload_dict.pop("check_response", True)
        self._send_ubx_message(pack_list, (msg_class, msg_id), payload_dict)
        if check_response:
            self._check_response()

    def poll_cfg(self, pack_list, unpack_list, msg_class, msg_id, **payload_dict):
        log.debug("CFG poll")
        id_tup = msg_class, msg_id
        self._send_ubx_message(pack_list, id_tup, payload_dict)
        return self._get(id_tup, unpack_list)

    # Private

    def _get(self, id_tup, pack_list):
        log.debug(f"UBX get: {self._fmt_id_tup(id_tup)}")
        while True:
            in_id_tup, payload_bytes = self._read_ubx()
            if in_id_tup == id_tup:
                log.debug("Found expected UBX message")
                return self._unpack_payload(pack_list, payload_bytes)
            log.debug(f"Discarding unexpected UBX message: {self._fmt_id_tup(id_tup)}")

    def _send_ubx_message(self, pack_list, id_tup, payload_dict):
        self._send(UBX_SYNC_BYTES)
        self._send(id_tup)
        payload_bytes = self._pack_payload(pack_list, payload_dict)
        len_bytes = struct.pack("<H", len(payload_bytes))
        self._send(len_bytes)
        self._send(payload_bytes)
        self._send(self._calc_checksum(id_tup, len_bytes + payload_bytes))

    def _check_response(self):
        id_tup, payload_bytes = self._read_ubx()
        if id_tup == (0x05, 0x01):
            log.debug("ACK")
            return
        if id_tup == (0x05, 0x00):
            raise UBXError("NAK")
        raise UBXError(
            f"UBX is not ACK-ACK or ACK-NAK. {self._fmt_id_tup(id_tup)}: "
            f"payload_bytes={self._fmt_bytes(payload_bytes)}"
        )

    def _read_ubx(self):
        """Read the next full UBX message."""
        self._read_to_sync()
        id_tup = self._unpack("BB", self._read(2))
        (payload_length,) = self._unpack("<H", self._read(2))
        payload_bytes = self.dev_f.read(size=payload_length)
        return id_tup, payload_bytes

    def _read_to_sync(self):
        """Read and discard bytes until the two UBX sync bytes have been received.
        """
        log.debug("Waiting for UBX sync bytes...")
        cur_list = [0, 0]
        while True:
            b = self.dev_f.read(1)
            cur_list.append(self._unpack("B", b)[0])
            cur_list = cur_list[1:]
            if cur_list == list(UBX_SYNC_BYTES):
                break
        log.debug("UBX sync found")

    def _send(self, v):
        b = bytes(v)
        log.debug(f"< {self._fmt_bytes(b)}")
        self.dev_f.write(b)

    def _read(self, byte_count=1024):
        b = self.dev_f.read(size=byte_count)
        log.debug(f"> {self._fmt_bytes(b)}")
        return b

    def _calc_checksum(self, id_tup, payload_bytes):
        class_int, msg_int = id_tup

        ck_a = (class_int + msg_int) & 0xFF
        ck_b = ((class_int << 1) + msg_int) & 0xFF

        for b in payload_bytes:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_a + ck_b) & 0xFF

        return ck_a, ck_b

    def _fmt_bytes(self, b, max_len=100):
        return f'{self._to_hex(b[:16])} ({b[:16].decode("utf-8", errors="replace")})'

    def _to_bytes(self, hex_str):
        return binascii.unhexlify(hex_str.replace(" ", ""))

    def _to_hex(self, b):
        return binascii.hexlify(b, sep=" ").decode("ascii")

    def _fmt_id_tup(self, id_tup):
        assert len(id_tup) == 2
        return f"msg_class={id_tup[0]:02x}, msg_id={id_tup[1]:02x}"

    def _unpack_payload(self, pack_list, payload_bytes):
        log.debug(f"Unpacking:")
        log.debug(f"pack_list={pack_list}")
        log.debug(f"payload_bytes={self._fmt_bytes(payload_bytes)}")
        fmt = "".join([x[0] for x in pack_list])
        res_dict = {}
        for x in zip(struct.unpack(fmt, payload_bytes), pack_list):
            res_dict[x[1][1]] = x[0]
        log.debug(f"payload={pprint.pformat(res_dict)}")
        return res_dict

    def _unpack(self, fmt, b):
        try:
            return struct.unpack(fmt, b)
        except struct.error as e:
            raise UBXError(
                f'Unpack with "{fmt}" requires {struct.calcsize(fmt)} bytes. '
                f"Received {len(b)} bytes"
            )

    def _pack_payload(self, pack_list, payload_dict):
        print(pack_list, payload_dict)
        invalid_dict = set(payload_dict.keys()) - set([x[1] for x in pack_list])
        if invalid_dict:
            raise UBXError(
                f"Payload contains keys that are not in pack list: {invalid_dict}"
            )
        log.debug(f"Packing:\npack_list={pack_list}\npayload_dict={payload_dict}")
        pack_dict = {value_name: fmt_char for fmt_char, value_name in pack_list}
        fmt_list = []
        value_list = []
        for fmt_char, value_name in pack_list:
            if payload_dict.get(value_name) is not None:
                fmt_list.append(fmt_char)
                value_list.append(payload_dict.get(value_name, 0))
        payload_bytes = self._pack("".join(fmt_list), value_list)
        log.debug(f"payload_bytes={self._fmt_bytes(payload_bytes)}")
        return payload_bytes

    def _pack(self, fmt, value_list):
        try:
            return struct.pack(fmt, *value_list)
        except struct.error as e:
            raise UBXError(
                f"e={e}, fmt={fmt}, value_list={value_list}, value_list_length={len(value_list)}"
            )


class UBXError(Exception):
    pass
