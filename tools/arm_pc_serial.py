#!/usr/bin/env python3
"""通过USART2文本命令行调试STM32F407五路机械臂。"""

import argparse
import time

import serial


def build_command(args):
    if args.action == "ping":
        return "PING"
    if args.action == "status":
        return "STATUS"
    if args.action == "help":
        return "HELP"
    if args.action == "servo":
        return "SERVO {} {} {}".format(
            args.number, args.angle, args.duration)
    if args.action == "base":
        return "BASE {} {}".format(args.angle, args.duration)
    if args.action == "move":
        return "MOVE {} {} {} {} {} {}".format(
            *args.angles, args.duration)
    if args.action == "goto":
        return "GOTO {} {} {} {} {}".format(
            args.x, args.y, args.z, args.magnet_angle, args.duration)
    if args.action == "magnet":
        return "MAGNET {}".format(args.state.upper())
    if args.action == "home":
        return "HOME {}".format(args.duration)
    if args.action == "calib":
        if args.calib_action == "point":
            return "CALIB POINT {}".format(args.number)
        return "CALIB {}".format(args.calib_action.upper())
    if args.action == "stop":
        return "STOP"
    raise ValueError("unsupported action")


def read_text(port, timeout_s):
    deadline = time.monotonic() + timeout_s
    received = False

    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            received = True
            print("RX:", line.decode("ascii", errors="replace").rstrip())

    if not received:
        print("未收到文本回复")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="例如 COM5")
    parser.add_argument("--baud", type=int, default=115200)
    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("ping")
    subparsers.add_parser("status")
    subparsers.add_parser("help")

    servo = subparsers.add_parser("servo")
    servo.add_argument("number", type=int, choices=range(1, 6))
    servo.add_argument("angle", type=float)
    servo.add_argument("--duration", type=int, default=1000)

    base = subparsers.add_parser("base")
    base.add_argument("angle", type=float)
    base.add_argument("--duration", type=int, default=1000)

    move = subparsers.add_parser("move")
    move.add_argument("angles", type=float, nargs=5,
                      metavar=("J1", "J2", "J3", "J4", "J5"))
    move.add_argument("--duration", type=int, default=1500)

    goto = subparsers.add_parser("goto")
    goto.add_argument("x", type=float)
    goto.add_argument("y", type=float)
    goto.add_argument("z", type=float)
    goto.add_argument("magnet_angle", type=float)
    goto.add_argument("--duration", type=int, default=1500)

    magnet = subparsers.add_parser("magnet")
    magnet.add_argument("state", choices=("on", "off"))

    home = subparsers.add_parser("home")
    home.add_argument("--duration", type=int, default=1500)

    calib = subparsers.add_parser("calib")
    calib.add_argument(
        "calib_action",
        choices=("check", "start", "point", "status", "stop"))
    calib.add_argument("number", type=int, nargs="?", choices=range(1, 7))

    subparsers.add_parser("stop")
    args = parser.parse_args()

    command = build_command(args)
    motion_command = args.action in (
        "servo", "base", "move", "goto", "home")
    calibration_start = (
        args.action == "calib" and args.calib_action == "start")
    if args.action == "calib" and args.calib_action == "point":
        if args.number is None:
            parser.error("calib point需要提供1~6的点号")
    elif args.action == "calib" and args.number is not None:
        parser.error("只有calib point可以提供点号")

    if calibration_start:
        timeout_s = 25.0
    elif motion_command:
        timeout_s = max(2.0, args.duration / 1000.0 + 1.0)
    else:
        timeout_s = 2.0

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        time.sleep(0.1)
        port.reset_input_buffer()
        print("TX:", command)
        port.write((command + "\r\n").encode("ascii"))
        read_text(port, timeout_s)


if __name__ == "__main__":
    main()
