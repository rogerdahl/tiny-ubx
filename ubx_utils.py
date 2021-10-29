"""High level methods to perform common configuration operations."""
import logging
import time

import serial

import tiny_ubx


TIMEOUT_SEC = 5.0


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


# CFG_PRT_PACK_LIST = (
#     ("B", "portId"),
#     ("B", "reserved0"),
#     ("H", "txReady"),
#     ("I", "mode"),
#     ("I", "baudRate"),
#     ("H", "inProtoMask"),
#     ("H", "outProtoMask"),
#     ("H", "reserved4"),
#     ("H", "reserved5"),
# )
#
# CFG_PRT_IN_LIST = (("B", "portId"),)
#
# CFG_CFG_PACK_LIST = (
#     ("I", "clearMask"),
#     ("I", "saveMask"),
#     ("I", "loadMask"),
#     ("B", "deviceMask"),
# )
#
# CFG_NMEA_PACK_LIST = (
#     ("B", "filter"),
#     ("B", "version"),
#     ("B", "numSV"),
#     ("B", "flags"),
# )


log = logging.getLogger(__name__)


# Message rates


class UbxUtils:
    def __init__(
        self,
        device_path="/dev/ttyUSB3",
        baud_rate=9600,
        tx_device_path=None,
        tx_baud_rate=None,
    ):
        self._device_path = device_path
        self._baud_rate = baud_rate
        self._tx_device_path = tx_device_path
        self._tx_baud_rate = tx_baud_rate
        self._ubx = None

        self.dev_f = None
        self.tx_dev_f = None

    def __enter__(self):
        self.dev_f = self._open_device(self._device_path, self._baud_rate)
        if self._tx_device_path:
            self.tx_dev_f = self._open_device(self._tx_device_path, self._tx_baud_rate)
        self._ubx = tiny_ubx.TinyUbx(self.dev_f, self.tx_dev_f or self.dev_f)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._close_device()

    def _open_device(self, device_path, baud_rate):
        dev_f = serial.Serial(device_path, baud_rate, timeout=TIMEOUT_SEC)
        log.debug(
            f"Opened device: {dev_f.name} @ {baud_rate} bps, timeout {TIMEOUT_SEC:.02f} sec"
            f'{f" (aliased from {device_path})" if dev_f.name != device_path else ""}'
        )
        return dev_f

    def _close_device(self):
        if self.dev_f:
            self.dev_f.close()
            self.dev_f = None
        if self.tx_dev_f:
            self.tx_dev_f.close()
            self.tx_dev_f = None

    #
    # Device info
    #

    def check_connection(self):
        """Check that we have a working connection to what appears to be a supported u-blox
        module.

        This is just an alias for get_module_info().
        """
        self._log_msg_start("Check connection to module")
        return self.get_module_info()

    def get_hardware_and_software_versions(self):
        self._log_msg_start("Poll hardware and software versions")
        return self._ubx.poll("MON-VER")

    def get_module_info(self):
        """Get some basic information about the module.

        Example:
            iTOW:        69282900
            numCh:       16
            globalFlags
                chipGen: u-blox 6
        """
        self._log_msg_start("Poll basic module info")
        return self._ubx.poll("NAV-SVINFO")

    def status(self):
        self.get_hardware_and_software_versions()

    # CFG-RST

    def reset_to_factory(self):
        """Reset to factory settings."""
        self._log_msg_start("Reset to factory settings")
        # Order of execution is clear, save, load. This will copy the factory default
        # settings from ROM to flash, load from flash, and activate.
        device_mask_dict = dict(
            deviceDevBbr=1,  # devSpiFlash device battery backed RAM
            deviceDevFlash=1,  # device Flash
            deviceDevEeprom=1,  # device EEPROM
            deviceDeviceSpiFlash=1,  # device SPI Flash
        )
        # self._ubx.send(
        #     "CFG-CFG",
        #     clearMask=0xFFFF,
        #     saveMask=0xFFFF,
        #     loadMask=0xFFFF,
        #     deviceMask=device_mask_dict,
        # )
        self._ubx.send(
            "CFG-CFG",
            clearMask=0xFFFF,
            saveMask=0x0000,
            loadMask=0xFFFF,
            deviceMask=device_mask_dict,
        )
        self._ubx.send(
            "CFG-CFG",
            clearMask=0x0000,
            saveMask=dict(
                msgConf=1,
            ),
            loadMask=dict(),
            deviceMask=device_mask_dict,
        )

    def save_settings(self):
        self._log_msg_start("Save the current settings to flash")
        self._ubx.send(
            "CFG-CFG",
            saveMask=0xFFFF,
            deviceMask=dict(
                deviceDevBbr=1,  # devSpiFlash device battery backed RAM
                deviceDevFlash=1,  # device Flash
                deviceDevEeprom=1,  # device EEPROM
                deviceDeviceSpiFlash=1,  # device SPI Flash
            ),
        )

    def stop(self):
        self._log_msg_start("CFG-RST - Stop processing")
        self._ubx.send("CFG-RST", navBbrMask=0x0000, resetMode=0x08)

    def start(self):
        self._log_msg_start("CFG-RST - Start processing")
        self._ubx.send("CFG-RST", navBbrMask=0x0000, resetMode=0x09)

    def reset_to_cold(self):
        """Reset to cold start."""
        self._log_msg_start("CFG-RST - Reset to cold start")
        self._ubx.send("CFG-RST", navBbrMask=0xFFFF, resetMode=0x01)

    def reset_to_warm(self):
        """Reset to warm start."""
        self._log_msg_start("CFG-RST - Reset to warm start")
        self._ubx.send("CFG-RST", navBbrMask=0x0001, resetMode=0x01)

    def reset_to_hot(self):
        """Reset to hot start."""
        self._log_msg_start("CFG-RST - Reset to hot start")
        self._ubx.send("CFG-RST", navBbrMask=0x0000, resetMode=0x01)

    def stop_all_messages(self):
        """Set the device to a known state by resetting it to factory setting, then
        setting update rate to once pr second, and turn off all messages.
        """
        # Update rate and message rate is not the same thing.
        self.set_update_rate(1000)
        self.set_nmea_rate_all(0)

    def poll_position(self):
        """GLL: Latitude and longitude, with time of position fix and status"""
        return self._ubx.poll("NAV-POSLLH")

    # CFG-MSG

    def set_nmea_rate_all_current(self, rate_int=0):
        self._log_msg_start(f"Setting output rate for all messages on current port")
        for msg_name in NMEA_MSG_DICT:
            self.set_msg_rate_single_current(msg_name, rate_int)

    def set_msg_rate_single_current(self, msg_name, rate_int):
        self._log_msg_start(f"Setting message rate for: {msg_name}")
        msg_id, msg_id = NMEA_MSG_DICT[msg_name]
        self._ubx.send("CFG-MSG", cls_id=msg_id, msgID=msg_id, rate_int=rate_int)

    def set_nmea_rate_all(self, rate_int):
        self._log_msg_start("Set the output rate for all messages on all ports")
        for msg_name, id_tup in NMEA_MSG_DICT.items():
            self.set_msg_rate_single_by_id(*id_tup, rate_int)

    def set_msg_rate_single(self, msg_name, rate_int):
        self._log_msg_start(f"Setting message rate on all ports for: {msg_name}")
        msg_id, msg_id = NMEA_MSG_DICT[msg_name]
        self.set_msg_rate_single_by_id(msg_id, msg_id, rate_int)

    def set_msg_rate_single_by_id(self, clsID, msgID, rate_int):
        self._log_msg_start(f"Setting message rate on all ports for: {clsID} {msgID}")
        self._ubx.send(
            "CFG-MSG",
            cls_id=clsID,
            msgID=msgID,
            **{
                "rate[1]": rate_int,
                "rate[2]": rate_int,
                "rate[3]": rate_int,
                "rate[4]": rate_int,
                "rate[5]": rate_int,
                "rate[6]": rate_int,
            },
        )

    def get_update_rate(self):
        self._log_msg_start("Polling update rate")
        return self._ubx.poll("CFG-RATE")["measRate"]

    def stop_occurring_messages(self, listen_sec=60):
        """Set message rate to zero on all ports for all UBX messages that the module
        sends within the listen period.
        """
        start_ts = time.time()
        while time.time() - start_ts < listen_sec:
            try:
                cls_id, msg_id, payload_bytes = self._ubx.read_next_ubx()
            except tiny_ubx.UBXError as e:
                log.debug(repr(e))
            else:
                self.set_msg_rate_single_by_id(cls_id, msg_id, 0)

    def listen(self, listen_sec):
        start_ts = time.time()
        while time.time() - start_ts < listen_sec:
            try:
                self._ubx.read_next_ubx()
            except tiny_ubx.UBXError as e:
                log.debug(repr(e))

    # CFG-RATE

    def set_update_rate(self, delay_ms):
        """Set how often an updated block of NMEA messages should be sent by the GPS module.

        :param delay_ms: (int) Number of milliseconds to wait between the start of each each update. A value below 1000
        causes messages to be sent more than once per second.

        The serial baud rate, number of active messages and update rate must be
        configured in such a way that there's time for the module to write the requested
        messages at the requested frequency.
        """
        self._log_msg_start("Setting NMEA message update rate")
        self._ubx.send("CFG-RATE", measRate=delay_ms, navRate=1, timeRef=1)

    # CFG-PRT (UART, I2C, etc)

    def set_baud_rate(self, baud_rate=115200):
        """Set the baud rate at which the GPS module will receive and send messages
        through the serial interface.
        """
        self._log_msg_start(f"Setting baudrate to {baud_rate} bps")
        self._ubx.send("CFG-PRT", baudRate=baud_rate)
        self._close_device()

    # CFG-NMEA

    def nmea_protocol_config(
        self,
    ):
        self._log_msg_start(f"NMEA protocol configuration")
        self._ubx.send(
            "CFG-NMEA",
            filter=0xFF,
            version=0x23,  # NMEA version 2.3
            numSV=0xFF,
            flags=dict(
                compat=0,
                consider=0,
            ),
        )

    # CFG-NAV5

    def nav5(self, **arg_dict):
        return self._ubx.send("CFG-NAV5", **arg_dict)

    # CFG-TP5

    def brief_led_flash(self):
        """Set PPS LED to flash very briefly to save battery"""
        self._ubx.send('CFG-TP5', pulseLenRatioLock=990000)


    # Misc

    def _log_msg_start(self, msg_str):
        log.debug("-" * 79)
        log.debug(msg_str)
