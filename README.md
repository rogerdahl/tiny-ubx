# A Python library, command line utility and machine readable UBX protocol specification for configuring u-blox GPS modules

UBX is a proprietary binary protocol used for configuring u-blox GPS modules. This project supports reading and modifying the most common settings on a u-blox GPS module with Python or the included command line utility.

## Machine readable protocol specification

The library and command line utilities are based on a machine readable and language agnostic UBX protocol specification. 

Only a small subset of messages are currently supported.

We are using HJSON (JSON for humans) as it's more convenient to work with than JSON. See `ubx.hjson` for the currently included messages.

Based on the protocol description, code can be generated to parse or create any UBX message instead of writing code for individual messages by hand. For more on this subject, look up `parser generators`.

This probably only works with genuine u-blox GPS modules. It has been tested with M6, M7 and M8 series modules. The GPS module must be connected to a serial port, usually via a USB to serial adapter.
