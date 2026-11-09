#!/usr/bin/env python3
"""通过USB-TTL向USART1发送第一问52字节示例计划并检查0xAC。"""

import argparse
import struct
import sys
import time

EXAMPLE_PACKET = bytes.fromhex(
    "AA 55 01 01 04 "
    "03 03 C8 FE 11 04 E4 02 32 D1 D1 "
    "02 02 2F FC EA 03 59 03 B9 0C 8B "
    "01 05 3C FB 5E 02 FE 02 A2 F4 7C "
    "00 06 61 FE 18 02 23 01 B5 EA 3D "
    "97 55 CC"
)


def read_i16_be(data: bytes) -> int:
    return struct.unpack(">h", data)[0]


def checksum(packet: bytes) -> int:
    return sum(packet[2:49]) & 0xFF


def decode_and_validate(packet: bytes) -> list[dict[str, float | int]]:
    if len(packet) != 52:
        raise ValueError(f"packet length must be 52, got {len(packet)}")
    if packet[:2] != b"\xAA\x55" or packet[-2:] != b"\x55\xCC":
        raise ValueError("bad header or tail")
    if packet[2] != 1 or packet[4] != 4:
        raise ValueError("bad version or piece count")
    if checksum(packet) != packet[49]:
        raise ValueError(
            f"bad checksum: calculated {checksum(packet):02X}, "
            f"received {packet[49]:02X}"
        )

    pieces: list[dict[str, float | int]] = []
    ids: set[int] = set()
    for order in range(4):
        record = packet[5 + order * 11 : 16 + order * 11]
        piece_id = record[0]
        piece = {
            "order": order,
            "id": piece_id,
            "pick_x": read_i16_be(record[1:3]) / 10.0,
            "pick_y": read_i16_be(record[3:5]) / 10.0,
            "place_x": read_i16_be(record[5:7]) / 10.0,
            "place_y": read_i16_be(record[7:9]) / 10.0,
            "rotate": read_i16_be(record[9:11]) / 100.0,
        }

        if piece_id not in range(4) or piece_id in ids:
            raise ValueError(f"bad or duplicate piece ID at order {order}")
        if not 0.0 <= piece["pick_x"] <= 210.0:
            raise ValueError(f"pick X out of A4 range at order {order}")
        if not -148.5 <= piece["pick_y"] <= 0.0:
            raise ValueError(f"pick Y out of upper-half range at order {order}")
        if not 0.0 <= piece["place_x"] <= 210.0:
            raise ValueError(f"place X out of A4 range at order {order}")
        if not 0.0 <= piece["place_y"] <= 148.5:
            raise ValueError(f"place Y out of lower-half range at order {order}")
        if not -180.0 < piece["rotate"] <= 180.0:
            raise ValueError(f"rotation out of range at order {order}")

        ids.add(piece_id)
        pieces.append(piece)

    return pieces


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="例如 COM5；仅在 --send 时必填")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.2)
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="ACK超时后重发完整52字节帧的次数，默认3次",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="真正发送；未指定时只校验并打印示例包",
    )
    args = parser.parse_args()

    pieces = decode_and_validate(EXAMPLE_PACKET)
    print(f"packet({len(EXAMPLE_PACKET)}): {EXAMPLE_PACKET.hex(' ').upper()}")
    print(f"checksum: {checksum(EXAMPLE_PACKET):02X}")
    for piece in pieces:
        print(
            "order={order} id={id} pick=({pick_x:.1f}, {pick_y:.1f}) "
            "place=({place_x:.1f}, {place_y:.1f}) "
            "rotate={rotate:+.2f}".format(**piece)
        )

    if not args.send:
        print("dry run only; add --send to transmit")
        return 0

    if not args.port:
        parser.error("--port is required when --send is used")
    if args.retries < 1:
        parser.error("--retries must be at least 1")

    try:
        import serial
    except ImportError:
        print(
            "pyserial is required for --send: pip install pyserial",
            file=sys.stderr,
        )
        return 2

    with serial.Serial(
        args.port,
        args.baudrate,
        timeout=0.05,
        write_timeout=args.timeout,
    ) as uart:
        uart.reset_input_buffer()
        received = bytearray()
        for attempt in range(1, args.retries + 1):
            print(f"TX attempt {attempt}/{args.retries}")
            uart.write(EXAMPLE_PACKET)
            uart.flush()

            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                block = uart.read(32)
                if block:
                    received.extend(block)
                    if 0xAC in block:
                        print(
                            "ACK received: "
                            f"{received.hex(' ').upper()}"
                        )
                        return 0

            if attempt < args.retries:
                print("ACK not received; resend full packet")

    print(
        f"ACK timeout after {args.retries} attempts; "
        f"received: {received.hex(' ').upper()}",
        file=sys.stderr,
    )
    if received:
        print(
            "Received bytes are not the USART1 protocol ACK. "
            "Check 3.3V TTL level, common GND, baudrate, and crossed "
            "wiring: adapter TX->PB7, adapter RX->PB6.",
            file=sys.stderr,
        )
    else:
        print(
            "No USART1 return data. Check that the latest firmware is "
            "flashed and use USART2 STATUS to see whether V_BYTES "
            "increases by 52 for each attempt.",
            file=sys.stderr,
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
