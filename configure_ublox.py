#!/usr/bin/env python3

"""Configure a u-blox GPS module
"""

import logging

import ubx_utils
import tiny_ubx

log = logging.getLogger(__name__)


def main():
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)8s %(message)s")

    with ubx_utils.UbxUtils("/dev/ttyUSB0", 9600, "/dev/ttyUSB0", 9600) as ubx:
        # with ubx_utils.UbxUtils("/dev/ttyUSB3", 115200, "/dev/ttyUSB3", 115200) as ubx:
        # with ubx_utils.UbxUtils("/dev/ttyUSB3", 9600, "/dev/ttyUSB3", 9600) as ubx:
        try:
            configure(ubx)
        except tiny_ubx.UBXError as e:
            log.error(str(e))
            return 1

        ubx.save_settings()

def configure(ubx):
    print(
        "Work in progress -- edit this file as required in order to perform the "
        "desired configuration."
    )

    # ubx.listen(60)
    # ubx.check_connection()
    # ubx.get_hardware_and_software_versions()
    # ubx.reset_to_factory()
    # ubx.set_baud_rate(115200)
    ubx.set_update_rate(10 * 1000)

    return

    # ubx.nmea_protocol_config()

    # print(ubx.prt_poll_update_rate())

    # Disable all NMEA messages
    ubx.set_nmea_rate_all(0)

    # Enable the UBX-NAV-POSLLH message
    # This is a UBX (binary) message, not a NMEA sentence.
    ubx.set_msg_rate_single_by_id(0x01, 0x02, 10)
    # ubx.set_msg_rate_single_by_id(0x0a, 0x09, 3)

    # Set pedestrian mode
    ubx.nav5(
        mask=dict(
            dyn=1,
        ),
        dynModel="pedestrian",
    )

    # # Set PPS LED to flash very briefly to save battery
    ubx.brief_led_flash()

    # # Disable PPS LED to save battery
    # # u.cfg('CFG-TP5', flags={'active': 0})
    #
    # # ubx.set_all_messages_for_all_protocols()
    # # ubx.stop_occurring_messages(listen_sec=60)
    #
    # # ubx.set_msg_rate_single("GLL", 3)
    # # ubx.generate_usage("CFG-PMS")
    # # ubx.send("CFG-PMS", powerSetupValue=0x00)
    #
    # # ubx.stop()
    # # ubx.start()
    # # ubx.reset_to_cold()
    # # ubx.reset_to_hot()


if __name__ == "__main__":
    main()
