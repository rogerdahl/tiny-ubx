#!/usr/bin/env python3

"""A tiny library for parsing and generating UBX messages
"""
import logging
import pathlib
import pprint
import re
import struct
import binascii
import sys
import time

import hjson as hjson

UBX_SYNC_BYTES = bytes([0xB5, 0x62])

# Translate from UBX to Python struct format descriptions. All are little-endian.
UBX_TO_STRUCT_FORMAT_DICT = dict(
    CH="s",  # ASCII / ISO 8859.1
    I1="b",  # Signed Char
    I2="h",  # Signed Short
    I4="l",  # Signed Long
    R4="f",  # IEEE 754 Single Precision
    R8="d",  # IEEE 754 Double Precision
    RU1_3="B",  # Unsigned Char
    U1="B",  # Unsigned Char
    U2="H",  # Unsigned Short
    U4="L",  # Unsigned Long
    X1="B",  # Bitfield, 8 bits
    X2="H",  # Bitfield, 16 bits
    X4="L",  # Bitfield, 32 bits
)

UBX_TO_TYPE_NAME = dict(
    CH="char",  # ASCII / ISO 8859.1
    I1="signed char",  # Signed Char
    I2="s16",  # Signed Short
    I4="s32",  # Signed Long
    R4="float32",  # IEEE 754 Single Precision
    R8="float64",  # IEEE 754 Double Precision
    RU1_3="byte",  # Unsigned Char
    U1="byte",  # Unsigned Char
    U2="u16",  # Unsigned Short
    U4="u32",  # Unsigned Long
    X1="bitfield8",  # Bitfield, 8 bits
    X2="bitfield16",  # Bitfield, 16 bits
    X4="bitfield32",  # Bitfield, 32 bits
)

# FIELD_FORMAT_TO_LEN = {
#     v: struct.calcsize(v) for (k, v) in UBX_TO_STRUCT_FORMAT_DICT.items()
# }
#
# STRUCT_FORMAT_TO_LEN = {
#     k: struct.calcsize(v) for (k, v) in UBX_TO_STRUCT_FORMAT_DICT.items()
# }

FORMAT_TO_LEN = {
    **{v: struct.calcsize(v) for (k, v) in UBX_TO_STRUCT_FORMAT_DICT.items()},
    **{k: struct.calcsize(v) for (k, v) in UBX_TO_STRUCT_FORMAT_DICT.items()},
}


log = logging.getLogger(__name__)


class TinyUbx:
    def __init__(
        self,
        rx_stream,
        tx_stream,
        ubx_json_path="./ubx.hjson",
        timeout_sec=5.0,
    ):
        self._ubx_dict = self._load_json(ubx_json_path)
        self._rx = rx_stream
        self._tx = tx_stream
        self._timeout_sec = timeout_sec

    def cfg(self, msg_name, **arg_dict):
        """Modify one or more configuration settings

        This method is intended to be the main way to modify settings. It performs a
        read-modify-write operation. The current settings for the given message are read
        from the module using a UBX POLL message, merged with the new values provided in
        {arg_dict} and written back with a UBX SEND message.

        If any values omitted in {arg_dict} should be set to zero instead of retaining their
        current values, use send() instead.

        Args:
            msg_name: A UBX message name, starting with "CFG-".
            **arg_dict: One or more arguments matching the payload of the message.
        """
        current_dict = self.poll(msg_name)

        for k, v in arg_dict.items():
            if isinstance(v, dict):
                current_dict[k].update(v)
            else:
                current_dict[k] = v

        self.send(msg_name, **current_dict)


    def send(self, msg_name, **arg_dict):
        """Create a UBX message, write it to the tx_stream and read the response from
        the rx_stream.

        Args:
            msg_name (str):
                The name of the message to send. Must match one of the frame names in the
                ubx.json file.

            arg_dict (dict):
                User provided values for fields and/or bitfield in the frame. May be
                empty. Any value not provided is set to zero.

                Values for bitfield can be provided as a dict containing zero, one or
                more mappings from bitfield names to values, or a single integer, in
                which each bit has already been set correctly. The integer is extended
                or truncated as needed to fit the width of the bitfield.
        """
        log.debug(f"UBX send: {msg_name}")
        # pprint.pprint(arg_dict)
        frame_dict = self._get_frame_by_name(msg_name)
        self._send_ubx_message(frame_dict, arg_dict)

    def poll(self, msg_name):
        log.debug(f"UBX poll: {msg_name}")
        frame_dict = self._get_frame_by_name(msg_name).copy()
        frame_dict["fields"] = []
        return self._send_ubx_message(frame_dict, {})

    def read_next_ubx(self):
        """Read the next full message."""
        self._read_to_sync()
        cls_id, msg_id = self._unpack("BB", self._read("clsID, msgID", 2))
        (payload_length,) = self._unpack("<H", self._read("payload_length", 2))
        payload_bytes = self._read("payload", payload_length)
        return cls_id, msg_id, payload_bytes

    def _send_ubx_message(self, frame_dict, arg_dict):
        # self._dump_dict("Sending message", frame_dict)
        # log.debug(pprint.pformat(frame_dict))
        self._write("sync", UBX_SYNC_BYTES)
        self._write(
            "clsID, msgID", struct.pack("BB", frame_dict["clsID"], frame_dict["msgID"])
        )
        payload_bytes = self._pack_payload(frame_dict, arg_dict)
        len_bytes = struct.pack("<H", len(payload_bytes))
        self._write("payload_length", len_bytes)
        self._write("payload", payload_bytes)
        self._write(
            "checksum",
            self._calc_checksum(
                frame_dict["clsID"], frame_dict["msgID"], len_bytes + payload_bytes
            ),
        )
        return self._get_message_response(frame_dict)


    # Private

    def _load_json(self, ubx_path):
        log.debug(f"Loading UBX protocol JSON: {ubx_path}")
        ubx_dict = hjson.load(pathlib.Path(ubx_path).open("rt"))

        for msg_name, frame_dict in ubx_dict.items():
            exp_field_list = []
            for field_dict in frame_dict["fields"]:
                field_name = field_dict["name"]
                fmt_str = field_dict["format"]
                m = re.match(r"(.*?)(?:\[(\d+)])?$", fmt_str)
                if not m:
                    raise UBXError('Invalid field format string: "{}"')
                fmt_str, repeat_count = m.groups()
                if fmt_str not in UBX_TO_STRUCT_FORMAT_DICT:
                    raise UBXError(
                        f'Invalid format string "{fmt_str}" in field "{field_name}"'
                    )
                if repeat_count:
                    for i in range(int(repeat_count)):
                        field_dict["format"] = fmt_str
                        field_dict["name"] = f"{field_name}[{i + 1}]"
                        exp_field_list.append(field_dict.copy())
                else:
                    exp_field_list.append(field_dict)
            frame_dict["fields"] = exp_field_list
        return ubx_dict

    def _get_message_response(self, frame_dict=None):
        """Possible responses:

        - ACK: Header id_tup=(0x05, 0x01) with frame_dict's id_tup in payload fields
        - NAK: Header id_tup=(0x05, 0x00) otherwise same as ACK
        - GET/POLL (is there a difference..?): A complete message with
            HEADER id_tup matching frame_dict's header id_tup

        Note that they had to move the id_tup into the body for ACK and NAK because
        otherwise, they'd break the clean mapping from id_tup to specific message
        structure.
        """
        if frame_dict:
            if frame_dict.get("skip_response", False):
                log.debug(
                    f'"{frame_dict["msg_name"]}" is not expected to return a response'
                )
                return

            log.debug(
                f"Waiting for response to "
                f'{self._fmt_id(frame_dict["clsID"], frame_dict["msgID"])}...'
            )

        start_ts = time.time()
        result_payload_dict = None

        while True:
            log.debug(f"Response time: {time.time() - start_ts:.02f}")

            if time.time() - start_ts > self._timeout_sec:
                raise UBXError("Timeout while waiting for response")

            # Temporary hack for returning result even if ACK does not follow.
            # TODO: Read up on the flow for poll messages.
            # try:
            cls_id, msg_id, payload_bytes = self.read_next_ubx()
            # except UBXError as e:
            #     log.debug(str(e))
            # self._dump_dict(f"Returned payload", result_payload_dict)
            # return result_payload_dict

            try:
                payload_dict = self._unpack_payload_by_id(cls_id, msg_id, payload_bytes)
            except UBXError as e:
                log.error(str(e))
                continue

            if not frame_dict:
                continue

            exp_id_str = self._fmt_id(frame_dict["clsID"], frame_dict["msgID"])
            id_str = self._fmt_id(cls_id, msg_id)

            if cls_id == 0x05 and msg_id in (0x00, 0x01):
                recv_cls_id, recv_msg_id = payload_dict["clsID"], payload_dict["msgID"]
                recv_id_str = self._fmt_id(recv_cls_id, recv_msg_id)
                is_ack = msg_id == 1
                ack_str = f'{"ACK (OK)" if is_ack else "NAK (error)"}'

                if (recv_cls_id, recv_msg_id) != (
                    frame_dict["clsID"],
                    frame_dict["msgID"],
                ):
                    log.debug(f'Ignoring "{ack_str}" response to "{recv_id_str}"')
                    log.debug(f"Still waiting for response to {exp_id_str}")
                    continue

                msg_str = f"{exp_id_str} returned {ack_str}"
                if not is_ack:
                    raise UBXError(msg_str)
                #
                # if result_payload_dict is None:
                log.debug(f"{exp_id_str} returned {ack_str}")
                return
                # else:
                #     self._dump_dict(
                #         f"{exp_id_str} returned payload", result_payload_dict
                #     )
                #     return result_payload_dict

            else:
                if (cls_id, msg_id) != (frame_dict["clsID"], frame_dict["msgID"]):
                    log.debug(f'Ignoring response to "{id_str}"')
                    self._dump_dict(f"Payload of ignored response", payload_dict)
                    log.debug(f"Still waiting for response to {exp_id_str}")
                    continue

                # log.debug(f"Captured payload for returning after ACK")
                log.debug(f"Returning payload")
                return payload_dict

    def _read_to_sync(self):
        """Read and discard bytes until the two sync bytes have been received."""
        log.debug("Waiting for sync bytes...")
        start_ts = time.time()
        cur_list = [0, 0]
        while True:
            if time.time() - start_ts > self._timeout_sec:
                raise UBXError("Timeout while waiting for sync bytes")
            if not self._rx.in_waiting:
                time.sleep(0.1)
                continue
            b = self._rx.read(1)
            if not b:
                raise UBXError("Timeout while waiting for sync bytes")
            # sys.stdout.write(b.decode("ascii", errors="replace"))
            # We call struct.unpack() directly here to avoid lots of single byte log records.
            cur_list.append(struct.unpack("B", b)[0])
            cur_list = cur_list[1:]
            if cur_list == list(UBX_SYNC_BYTES):
                break
        log.debug(f"Sync found after {time.time() - start_ts:.2f} sec")

    def _write(self, section_str, b):
        log.debug(f"< {self._fmt_bytes(b, section_str)}")
        self._tx.write(b)

    def _read(self, section_str, byte_count):
        b = self._rx.read(size=byte_count)
        log.debug(f"> {self._fmt_bytes(b, section_str)}")
        return b

    def _calc_checksum(self, cls_id, msg_id, payload_bytes):
        ck_a, ck_b = 0x00, 0x00
        for b in bytes([cls_id, msg_id]) + payload_bytes:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_a + ck_b) & 0xFF
        return bytes([ck_a, ck_b])

    # Pack

    def _pack_payload(self, frame_dict, arg_dict):
        """Pack the user provided arguments into the payload fields of the message.

        Args:
            frame_dict (dict):
                The frame description for the message being created, loaded from the
                ubx.json file.
            arg_dict (dict):
                See `send()`.
        """
        self._dump_dict("Packing args", arg_dict)
        return self._pack(frame_dict, arg_dict)

    def _pack(self, frame_dict, arg_dict_in):
        arg_dict = arg_dict_in.copy()
        struct_str = self._get_struct_str(frame_dict["fields"])
        log.debug(f"Packing to target struct_str: {struct_str}")

        # TODO: Collect argument errors and display them all at the same time, so that
        # the user doesn't have to discover them one by one.
        #
        # Note: Since we're matching args up with the frame_dict, errors in the
        # frame_dict can also look like errors in the args. This assumes that the
        # frame_dict is valid. Ideally, the frame_dict would be validated against a
        # schema before we get here.
        assert isinstance(struct_str, str), repr(struct_str)
        value_tup = []
        for field_dict in frame_dict["fields"]:
            assert isinstance(field_dict, dict), repr(field_dict)
            # log.debug(f"field_dict={field_dict}")
            # if 'reserved' in field_dict['name'].lower():
            #     continue
            field_name = field_dict["name"]
            field_value = arg_dict.pop(field_name, None)

            if self._is_bitfield(field_dict):
                # log.debug(f"field_value={field_value}")
                if isinstance(field_value, dict):
                    field_int = self._pack_bitfield(field_dict["bitfield"], field_value)
                elif isinstance(field_value, int):
                    field_int = field_value
                elif field_value is None:
                    field_int = 0
                else:
                    raise UBXError(
                        f"The value for a bitfield must be either a dict or an int, "
                        f'not "{field_value}"'
                    )
            else:
                if isinstance(field_value, str):
                    map_dict = field_dict.get("map", {})
                    for field_int, value_str in map_dict.items():
                        if field_value == value_str:
                            break
                    else:
                        raise UBXError(
                            f'Invalid string value "{field_value}" for field '
                            f'"{field_name}". '
                            f"The field has no map."
                            if not map_dict
                            else f'Valid strings are "{", ".join(map_dict.values())}"'
                        )
                elif isinstance(field_value, int):
                    field_int = field_value
                elif field_value is None:
                    field_int = 0
                else:
                    raise UBXError(
                        f'Invalid value for field "{field_name}". The value '
                        f'must be a str or int, not "{repr(field_value)}"'
                    )

            value_tup.append(int(field_int))

        if arg_dict:
            raise UBXError(
                f'Invalid args for message type "{frame_dict["msg_name"]}": '
                f"{self._to_dot_list(arg_dict)}"
            )

        log.debug(f"Packing: {self._fmt_list(value_tup)}")

        try:
            return struct.pack(struct_str, *value_tup)
        except struct.error as e:
            raise UBXError(
                f"Packing the payload bytes failed. "
                f'Error: {str(e)}, struct_str="{struct_str}", arg_dict={arg_dict_in}'
            )

    def _pack_bitfield(self, bitfield, bitfield_arg_dict):
        # log.debug(f'Packing bitfield:')
        # self._dump_dict('bitfield', bitfield)
        # self._dump_dict('bitfield_arg_dict', bitfield_arg_dict)
        result_int = 0
        bitfield_name = None
        for sub_dict in bitfield:
            assert isinstance(sub_dict, dict), repr(sub_dict)
            bitfield_name = sub_dict["name"]
            assert isinstance(bitfield_name, str), repr(bitfield_name)
            bitfield_value = bitfield_arg_dict.pop(bitfield_name, 0)

            if isinstance(bitfield_value, str):
                map_dict = sub_dict.get("map", {})
                for bitfield_int, value_str in map_dict.items():
                    if bitfield_value == value_str:
                        break
                else:
                    raise UBXError(
                        f'Invalid string value "{bitfield_value}" for bitfield '
                        f'"{bitfield_name}". '
                        f"The field has no map."
                        if not map_dict
                        else f'Valid strings are "{", ".join(map_dict.values())}"'
                    )
            elif isinstance(bitfield_value, int):
                bitfield_int = bitfield_value
            else:
                raise UBXError(
                    f'Invalid value for bitfield "{bitfield_name}". The value '
                    f'must be a str or int, not "{repr(bitfield_value)}"'
                )
            # log.debug(f'bitfield_int={repr(bitfield_int)}')
            bitfield_int = int(bitfield_int)
            try:
                msb_int, lsb_int = sub_dict["range"]
            except TypeError:
                msb_int = lsb_int = sub_dict["range"]
            max_value = 1 << (msb_int + 1 - lsb_int)
            if bitfield_int >= max_value:
                raise UBXError(
                    f"Bitfield {bitfield_name} must be in range 0 to {max_value - 1}, "
                    f"not {bitfield_int}"
                )
            result_int |= bitfield_int << lsb_int

        if bitfield_arg_dict:
            raise UBXError(
                f'Invalid args provided for bitfield "{bitfield_name}": '
                f"{self._to_dot_list(bitfield_arg_dict)}"
            )

        log.debug(f'Returning packed bitfield: {result_int:0b}')
        return result_int

    # Unpack

    def _unpack_payload(self, frame_dict, payload_bytes):
        struct_str = self._get_struct_str(frame_dict["fields"])
        # payload_bytes = self._adjust_payload_for_known_fields(struct_str, payload_bytes)
        value_list = self._unpack(struct_str, payload_bytes)
        payload_dict = {}
        for field_value, field_dict in zip(value_list, frame_dict["fields"]):
            if self._is_bitfield(field_dict):
                field_value = self._unpack_bitfield(field_dict["bitfield"], field_value)
            else:
                field_value = field_dict.get("map", {}).get(
                    str(field_value), field_value
                )
            payload_dict[field_dict["name"]] = field_value
        self._dump_dict("Unpacked payload", payload_dict)
        return payload_dict

    def _unpack(self, struct_str, payload_bytes):
        # b = self._adjust_payload_for_known_fields(struct_str, payload_bytes)
        try:
            value_list = struct.unpack(struct_str, payload_bytes)
            log.debug(f"Unpacked: {self._fmt_list(value_list)}")
            return value_list
        except struct.error as e:
            raise UBXError(
                f"Unpacking the payload bytes failed. "
                f"Error: {str(e)}, "
                f'struct_str="{struct_str}", '
                f'payload_bytes={self._fmt_bytes(payload_bytes, "payload")}'
            )

    def _is_bitfield(self, field_dict):
        return field_dict["format"].startswith("X")

    def _unpack_bitfield(self, bitfield, bitfield_value):
        result_dict = {}
        for field_dict in bitfield:
            try:
                msb_int, lsb_int = field_dict["range"]
            except TypeError:
                msb_int = lsb_int = field_dict["range"]
            value_idx = (bitfield_value & (1 << (msb_int + 1)) - 1) >> lsb_int
            result_dict[field_dict["name"]] = field_dict.get("map", {}).get(
                str(value_idx), value_idx
            )
        return result_dict

    def _unpack_payload_by_id(self, cls_id, msg_id, payload_bytes):
        frame_dict = self._get_frame_by_id(cls_id, msg_id)
        return self._unpack_payload(frame_dict, payload_bytes)

    # Pack and unpack, shared

    def _get_frame_by_name(self, msg_name):
        try:
            frame_dict = self._ubx_dict[msg_name]
        except KeyError:
            raise UBXError(f"Unsupported message: {msg_name}")
        # Copy the frame key into the frame for convenience when passing the frame
        # around by itself.
        frame_dict["msg_name"] = msg_name
        return frame_dict

    def _get_frame_by_id(self, cls_id, msg_id):
        for msg_name in self._ubx_dict:
            # We use this method since _get_frame_by_name() adds msg_name to the
            # frame_dict, and may do more annotation in the future.
            frame_dict = self._get_frame_by_name(msg_name)
            if (frame_dict["clsID"], frame_dict["msgID"]) == (cls_id, msg_id):
                return frame_dict

        raise UBXError(
            f"Unknown or unsupported message type: "
            f"clsID={cls_id:02x} msgID={msg_id:02x}. "
        )

    def _get_struct_str(self, fields):
        """Generate a Python struct format string for packing or unpacking the payload
        based on the frame's field list.
        """
        # log.debug(f'Translated format: {field_dict["format"]} -> {fmt_str}')
        # The format strings are already validated
        return "<" + "".join(
            [UBX_TO_STRUCT_FORMAT_DICT[field_dict["format"]] for field_dict in fields]
        )

    def generate_usage(self, msg_name):
        frame_dict = self._get_frame_by_name(msg_name)
        frame_list = frame_dict["fields"]
        name_list = []
        for field_dict in frame_list:
            # if "reserved" in field_dict["name"].lower():
            #     break
            name = field_dict["name"]
            arg_type = UBX_TO_TYPE_NAME[field_dict["format"]]
            if not self._is_bitfield(field_dict):
                name_list.append((name, arg_type))
            else:
                for sub_dict in field_dict["bitfield"]:
                    name = f'{field_dict["name"]}.{sub_dict["name"]}'
                    try:
                        msb_int, lsb_int = sub_dict["range"]
                    except TypeError:
                        msb_int = lsb_int = sub_dict["range"]
                    max_value = 1 << (msb_int + 1 - lsb_int)
                    arg_type = f"0 - {max_value - 1}"
                    bf_map = sub_dict.get("map", {0: "0", 1: "1"})
                    arg_type += " ({})".format(
                        ", ".join(f"{k}={v}" for (k, v) in bf_map.items())
                    )
                    name_list.append((name, arg_type))

        log.info(f'Arguments for {frame_dict["msg_name"]}:')
        for name_str, type_str in name_list:
            log.info(f"  {name_str}: {type_str}")

    # String formatting

    def _fmt_bytes(self, b, section_str, max_len=32):
        return (
            f'{self._to_hex(b[:max_len]) if b else "<empty>"} '
            f'{"... " if len(b) > max_len else ""}'
            f"({section_str}, {len(b)} bytes)"
        )

    def _fmt_list(self, v_list, max_len=32):
        return (
            f'{", ".join(hex(v) if isinstance(v, int) else v for v in v_list[:max_len])} '
            f'{"... " if len(v_list) > max_len else ""}'
            f"({len(v_list)} items)"
        )

    def _to_bytes(self, hex_str):
        return binascii.unhexlify(hex_str.replace(" ", ""))

    def _to_hex(self, b):
        return binascii.hexlify(b, sep=" ").decode("ascii")

    def _get_field_len_by_format(self, fmt_str):
        return FORMAT_TO_LEN[fmt_str]

    def _fmt_id(self, cls_id, msg_id):
        # 0x06, 0x09 -> (CFG-CFG / 06 09)
        try:
            frame_dict = self._get_frame_by_id(cls_id, msg_id)
        except UBXError:
            msg_name = "unknown"
        else:
            msg_name = frame_dict["msg_name"]
        return f"({msg_name} / {cls_id:02x} {msg_id:02x})"

    def _to_dot_list(self, d):
        """Translate the keys of nested dicts to a list of dot separated names.
        {'a': {'b': 1, 'c': 2}} -> a.b, a.c
        """

        def _walk_dict(d_, p_):
            for k_ in d_:
                dot_list.append(".".join(p_ + [k_]))

        dot_list = []
        _walk_dict(d, [])

        # return dot_list
        return ", ".join(dot_list)

    def _dump_dict(self, msg_str, d):
        def _r(v_):
            if isinstance(v_, dict):
                return {kk_: _r(vv_) for kk_, vv_ in v_.items()}
            elif isinstance(v_, list):
                return [_r(vv_) for vv_ in v_]
            else:
                return v_

        dd = _r(d)

        log.debug(f"{msg_str}:")
        for line in pprint.pformat(dd, sort_dicts=False).splitlines():
            if 'UNIMPLEMENTED' not in line and 'reserved' not in line:
                log.debug(f"  {line}")


# class HexEncoder(hjson.JSONEncoder):
#     def _preprocess(self, o):
#         if isinstance(o, int):
#             return hex(o)
#         elif isinstance(o, dict):
#             return {k: self._preprocess(v) for k,v in o.items()}
#         elif isinstance(o, list):
#             return [self._preprocess(v) for v in o]
#         return o
#
#     def iterencode(self, o, **kwargs):
#         return super().iterencode(self._preprocess(o))


class UBXError(Exception):
    pass
