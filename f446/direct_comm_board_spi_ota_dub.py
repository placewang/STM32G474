#!/usr/bin/env python3
"""Direct SPI OTA client matching app_IapUpData.c."""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Optional, Sequence


FRAME_SIZE = 68
PAYLOAD_SIZE = 54
CHECKSUM_OFFSET = 63
HEADER = 0xAA
FOOTER = 0xBB
FLASH_BUFFER_SIZE = 512
MAX_FIRMWARE_SIZE = 0x34000
# bsp_SPI1_FillPack_XORCheck() rewrites bytes 1..2 of every 68-byte TX frame.
WIRE_RESPONSE_COMMAND = 0x0303

SPI_IOC_NRBITS = 8
SPI_IOC_TYPEBITS = 8
SPI_IOC_SIZEBITS = 14
SPI_IOC_NRSHIFT = 0
SPI_IOC_TYPESHIFT = SPI_IOC_NRSHIFT + SPI_IOC_NRBITS
SPI_IOC_SIZESHIFT = SPI_IOC_TYPESHIFT + SPI_IOC_TYPEBITS
SPI_IOC_DIRSHIFT = SPI_IOC_SIZESHIFT + SPI_IOC_SIZEBITS
SPI_IOC_WRITE = 1


class Command(IntEnum):
    UPDATE_REQUEST = 0xFF
    UPDATE_DATA = 0xFE
    UPDATE_LAST_PACK = 0xFD
    UPDATE_SUCCESS = 0xFC
    UPDATE_FAIL = 0xFB
    UPDATE_ACK = 0xFA


@dataclass(frozen=True)
class OtaHeader:
    file_crc32: int
    file_size: int
    firmware_version: int
    hardware_version: int


@dataclass(frozen=True)
class OtaPackage:
    path: Path
    payload: bytes
    header: OtaHeader


@dataclass(frozen=True)
class ParsedFrame:
    command: int
    pack_index: int
    payload_length: int


def _ioc(direction: int, type_: int, nr: int, size: int) -> int:
    return ((direction << SPI_IOC_DIRSHIFT) |
            (type_ << SPI_IOC_TYPESHIFT) |
            (nr << SPI_IOC_NRSHIFT) |
            (size << SPI_IOC_SIZESHIFT))


def spi_ioc_message(count: int) -> int:
    return _ioc(SPI_IOC_WRITE, ord("k"), 0, count * 32)


SPI_IOC_WR_MODE = _ioc(SPI_IOC_WRITE, ord("k"), 1, 1)
SPI_IOC_WR_LSB_FIRST = _ioc(SPI_IOC_WRITE, ord("k"), 2, 1)
SPI_IOC_WR_BITS_PER_WORD = _ioc(SPI_IOC_WRITE, ord("k"), 3, 1)
SPI_IOC_WR_MAX_SPEED_HZ = _ioc(SPI_IOC_WRITE, ord("k"), 4, 4)


def read_le16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little")


def read_le32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 4], "little")


def calculate_checksum(data: bytes) -> int:
    checksum = 0
    full_words = CHECKSUM_OFFSET // 4
    for index in range(full_words):
        checksum ^= read_le32(data, index * 4)
    for index in range(full_words * 4, CHECKSUM_OFFSET):
        checksum ^= data[index]
    return (checksum + HEADER) & 0xFFFFFFFF


def make_frame(command: Command, pack_index: int, payload: bytes) -> bytes:
    if len(payload) > PAYLOAD_SIZE:
        raise ValueError("payload exceeds 54 bytes")

    frame = bytearray([0xFF] * FRAME_SIZE)
    frame[0] = HEADER
    frame[1:3] = int(command).to_bytes(2, "little")
    frame[3:7] = pack_index.to_bytes(4, "little")
    frame[7:9] = len(payload).to_bytes(2, "little")
    frame[9:9 + len(payload)] = payload
    frame[CHECKSUM_OFFSET:CHECKSUM_OFFSET + 4] = calculate_checksum(frame).to_bytes(4, "little")
    frame[-1] = FOOTER
    return bytes(frame)


def make_request_frame(package: OtaPackage) -> bytes:
    payload = bytearray([0xFF] * PAYLOAD_SIZE)
    payload[0:4] = package.header.file_crc32.to_bytes(4, "little")
    payload[4:8] = len(package.payload).to_bytes(4, "little")
    payload[8:12] = package.header.firmware_version.to_bytes(4, "little")
    payload[12:16] = package.header.hardware_version.to_bytes(4, "little")
    return make_frame(Command.UPDATE_REQUEST, 0xFF, bytes(payload))


def parse_frame(raw: bytes) -> Optional[ParsedFrame]:
    if len(raw) != FRAME_SIZE or raw[0] != HEADER or raw[-1] != FOOTER:
        return None
    payload_length = read_le16(raw, 7)
    if payload_length > PAYLOAD_SIZE:
        return None
    if read_le32(raw, CHECKSUM_OFFSET) != calculate_checksum(raw):
        return None
    return ParsedFrame(read_le16(raw, 1), read_le32(raw, 3), payload_length)


def frame_summary(raw: bytes) -> str:
    parsed = parse_frame(raw)
    if parsed is None:
        preview = " ".join(f"{byte:02X}" for byte in raw[:8])
        return f"invalid [{preview} ...]"
    return f"cmd=0x{parsed.command:02X} pack={parsed.pack_index} len={parsed.payload_length}"


def parse_ota_package(path: Path) -> OtaPackage:
    ota_data = path.read_bytes()
    if len(ota_data) < 16:
        raise ValueError("ota file is smaller than its 16-byte header")

    header = OtaHeader(
        file_crc32=read_le32(ota_data, 0),
        file_size=read_le32(ota_data, 4),
        firmware_version=read_le32(ota_data, 8),
        hardware_version=read_le32(ota_data, 12),
    )
    if header.file_size != len(ota_data):
        raise ValueError(f"ota size mismatch: header={header.file_size}, actual={len(ota_data)}")

    payload = ota_data[16:]
    if not payload:
        raise ValueError("ota payload is empty")
    if len(payload) > MAX_FIRMWARE_SIZE:
        raise ValueError(
            f"ota payload exceeds backup capacity: {len(payload)} > {MAX_FIRMWARE_SIZE}"
        )
    actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
    if actual_crc != header.file_crc32:
        raise ValueError(f"ota crc mismatch: header={header.file_crc32:#010x}, actual={actual_crc:#010x}")
    return OtaPackage(path, payload, header)


class SpiMaster:
    def __init__(self, device: str, mode: int, speed_hz: int, bits: int,
                 lsb_first: int, delay_us: int) -> None:
        self.device = device
        self.mode = mode
        self.speed_hz = speed_hz
        self.bits = bits
        self.lsb_first = lsb_first
        self.delay_us = delay_us
        self._file = None
        self.fd: Optional[int] = None

    def open(self) -> None:
        self._file = open(self.device, "rb+", buffering=0)
        self.fd = self._file.fileno()
        fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack("B", self.mode & 0xFF))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", self.speed_hz))
        fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack("B", self.bits & 0xFF))
        fcntl.ioctl(self.fd, SPI_IOC_WR_LSB_FIRST, struct.pack("B", self.lsb_first & 0xFF))

    def transfer(self, tx: bytes) -> bytes:
        if self.fd is None:
            raise RuntimeError("SPI device is not open")
        if len(tx) != FRAME_SIZE:
            raise ValueError("every SPI transaction must be exactly 68 bytes")

        tx_buffer = (ctypes.c_ubyte * FRAME_SIZE).from_buffer_copy(tx)
        rx_buffer = (ctypes.c_ubyte * FRAME_SIZE)()
        transfer = struct.pack(
            "<QQIIH6B",
            ctypes.addressof(tx_buffer),
            ctypes.addressof(rx_buffer),
            FRAME_SIZE,
            self.speed_hz,
            self.delay_us,
            self.bits,
            0, 0, 0, 0, 0,
        )
        fcntl.ioctl(self.fd, spi_ioc_message(1), bytearray(transfer))
        return bytes(rx_buffer)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self.fd = None


def wait_for_enter(enabled: bool, message: str) -> None:
    if enabled:
        input(message)


def print_transfer(label: str, tx: bytes, rx: bytes, verbose: bool) -> None:
    print(f"{time.strftime('%H:%M:%S')} [{label}] TX {frame_summary(tx)} | RX {frame_summary(rx)}")
    rx_head = " ".join(f"{byte:02X}" for byte in rx[:8])
    rx_tail = " ".join(f"{byte:02X}" for byte in rx[-5:])
    print(f"RX: {rx_head} ... {rx_tail}")
    if verbose:
        print("TX:", " ".join(f"{byte:02X}" for byte in tx))


def poll_response(
    spi: SpiMaster,
    expected_commands: Sequence[int],
    expected_pack: int,
    timeout_ms: int,
    poll_interval_ms: int,
    verbose: bool,
    max_attempts: Optional[int] = None,
    log_polls: bool = True,
) -> Optional[ParsedFrame]:
    deadline = time.monotonic() + timeout_ms / 1000.0
    poll_frame = bytes([0xFF] * FRAME_SIZE)
    poll_count = 0

    while max_attempts is not None or time.monotonic() < deadline:
        if max_attempts is not None and poll_count >= max_attempts:
            break
        poll_count += 1
        rx = spi.transfer(poll_frame)
        parsed = parse_frame(rx)
        if verbose and log_polls:
            print_transfer(f"POLL#{poll_count}", poll_frame, rx, verbose)
        if (parsed is not None and parsed.pack_index == expected_pack and
                parsed.command in expected_commands):
            return parsed
        if max_attempts is None or poll_count < max_attempts:
            time.sleep(poll_interval_ms / 1000.0)
    return None


def send_once_and_wait(
    spi: SpiMaster,
    frame: bytes,
    expected_commands: Sequence[int],
    expected_pack: int,
    settle_ms: int,
    timeout_ms: int,
    poll_interval_ms: int,
    label: str,
    interactive: bool,
    verbose: bool,
    max_poll_attempts: Optional[int] = None,
    log_polls: bool = True,
    log_transfer: bool = True,
) -> Optional[ParsedFrame]:
    wait_for_enter(interactive, f"press Enter to send {label}...")
    rx = spi.transfer(frame)
    if verbose or log_transfer:
        print_transfer(label, frame, rx, verbose)

    # app_IapUpData.c prepares the response only after this NSS cycle ends.
    if settle_ms > 0:
        time.sleep(settle_ms / 1000.0)
    return poll_response(
        spi, expected_commands, expected_pack, timeout_ms, poll_interval_ms, verbose,
        max_poll_attempts, log_polls,
    )


def print_package(package: OtaPackage) -> None:
    print("=== OTA Package ===")
    print(f"path             : {package.path.resolve()}")
    print(f"otaFileSize      : {package.header.file_size}")
    print(f"payloadSize      : {len(package.payload)}")
    print(f"payloadCrc32     : {package.header.file_crc32:#010x}")
    print(f"firmwareVersion  : {package.header.firmware_version:#010x}")
    print(f"hardwareVersion  : {package.header.hardware_version:#010x}")
    print()


def run_ota(args: argparse.Namespace, interactive: bool = False, request_only: bool = False) -> int:
    package = parse_ota_package(args.ota_path)
    print_package(package)
    print(f"SPI device       : {args.spi_dev}")
    print()
    spi = SpiMaster(args.spi_dev, args.mode, args.speed, args.bits, args.lsb_first, args.delay_us)
    spi.open()

    try:
        request = make_request_frame(package)
        print(f"{time.strftime('%H:%M:%S')} [REQUEST] sending update request; erase wait={args.request_settle_ms}ms")
        response = send_once_and_wait(
            spi, request, (Command.UPDATE_ACK,), 0xFF,
            args.request_settle_ms, 0, args.request_poll_interval_ms,
            "REQUEST", interactive, args.verbose, 2, log_transfer=args.verbose,
        )
        if response is None:
            print("request ACK timeout; request was not resent", file=sys.stderr)
            return 1
        print(f"{time.strftime('%H:%M:%S')} [ACK] request accepted")
        if request_only:
            return 0
        if args.post_ack_settle_ms > 0:
            time.sleep(args.post_ack_settle_ms / 1000.0)

        total_packets = (len(package.payload) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
        progress_step = max(1, args.progress_step_percent)
        next_progress = 0
        bytes_queued = 0
        for pack_index in range(total_packets):
            chunk = package.payload[pack_index * PAYLOAD_SIZE:(pack_index + 1) * PAYLOAD_SIZE]
            is_last = pack_index == total_packets - 1
            command = Command.UPDATE_LAST_PACK if is_last else Command.UPDATE_DATA
            frame = make_frame(command, pack_index, chunk)

            previous_blocks = bytes_queued // FLASH_BUFFER_SIZE
            bytes_queued += len(chunk)
            current_blocks = bytes_queued // FLASH_BUFFER_SIZE
            if is_last:
                settle_ms = args.final_settle_ms
                poll_interval_ms = args.poll_interval_ms
                max_poll_attempts = args.final_poll_attempts
                expected = (Command.UPDATE_SUCCESS, Command.UPDATE_FAIL)
            elif current_blocks > previous_blocks:
                settle_ms = args.flash_write_settle_ms
                poll_interval_ms = args.packet_poll_interval_ms
                max_poll_attempts = args.packet_poll_attempts
                expected = (Command.UPDATE_DATA,)
            else:
                settle_ms = args.packet_settle_ms
                poll_interval_ms = args.packet_poll_interval_ms
                max_poll_attempts = args.packet_poll_attempts
                expected = (Command.UPDATE_DATA,)

            if is_last:
                print(f"{time.strftime('%H:%M:%S')} [FINAL] sending last packet "
                      f"{pack_index + 1}/{total_packets}; bytes={len(chunk)}; settle={settle_ms}ms")
            response = send_once_and_wait(
                spi, frame, expected, pack_index, settle_ms,
                args.packet_timeout_ms, poll_interval_ms,
                f"PACK#{pack_index}", interactive, args.verbose, max_poll_attempts,
                log_polls=not is_last,
                log_transfer=args.verbose,
            )
            if response is None:
                if is_last:
                    print("final status timeout: neither UPDATE_SUCCESS (0xFC) nor "
                          "UPDATE_FAIL (0xFB) was received", file=sys.stderr)
                else:
                    print(f"packet {pack_index} ACK timeout; packet was not resent", file=sys.stderr)
                return 1
            if is_last and response.command == Command.UPDATE_FAIL:
                print(f"{time.strftime('%H:%M:%S')} [FAILED] board returned UPDATE_FAIL (0xFB)",
                      file=sys.stderr)
                return 1
            if is_last:
                print(f"{time.strftime('%H:%M:%S')} [DONE] board returned UPDATE_SUCCESS (0xFC)")
                return 0
            completed = pack_index + 1
            percent = (completed * 100) // total_packets
            if percent >= next_progress or completed == total_packets - 1:
                print(f"{time.strftime('%H:%M:%S')} [PROGRESS] {percent:3d}% "
                      f"({completed}/{total_packets}, ack pack={response.pack_index})")
                next_progress = ((percent // progress_step) + 1) * progress_step
            if not is_last and args.post_ack_settle_ms > 0:
                time.sleep(args.post_ack_settle_ms / 1000.0)

        print(f"{time.strftime('%H:%M:%S')} [DONE] board returned UPDATE_SUCCESS (0xFC)")
        return 0
    finally:
        spi.close()


def add_timing_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--request-settle-ms", type=int, default=2500,
                        help="wait after request while the board erases flash")
    parser.add_argument("--request-poll-interval-ms", type=int, default=10,
                        help="interval between the two request ACK poll frames")
    parser.add_argument("--packet-settle-ms", type=int, default=5,
                        help="wait after a packet that does not write flash")
    parser.add_argument("--flash-write-settle-ms", type=int, default=20,
                        help="wait when accumulated data crosses a 512-byte flash block")
    parser.add_argument("--packet-poll-interval-ms", type=int, default=10,
                        help="interval between the two UPDATE_DATA ACK poll frames")
    parser.add_argument("--packet-poll-attempts", type=int, default=10,
                        help="maximum 0xFF polls for each UPDATE_DATA ACK")
    parser.add_argument("--post-ack-settle-ms", type=int, default=3,
                        help="wait after a valid ACK before sending the next command/data frame")
    parser.add_argument("--final-settle-ms", type=int, default=500,
                        help="wait after UPDATE_LAST_PACK before final status polling")
    parser.add_argument("--final-poll-attempts", type=int, default=40,
                        help="maximum 0xFF polls for UPDATE_SUCCESS/UPDATE_FAIL")
    parser.add_argument("--packet-timeout-ms", type=int, default=10000,
                        help="data/final response polling timeout")
    parser.add_argument("--poll-interval-ms", type=int, default=50,
                        help="response polling interval")
    parser.add_argument("--progress-step-percent", type=int, default=5,
                        help="print transfer progress every N percent")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Direct SPI OTA client for app_IapUpData.c")
    parser.add_argument("--spi-dev", default="/dev/spidev0.0")
    parser.add_argument("--second-spi-dev", default="/dev/spidev0.1")
    parser.add_argument("--between-devices-ms", type=int, default=50,
                        help="wait after CS0 succeeds before restarting the full flow on CS1")
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--speed", type=int, default=3_000_000)
    parser.add_argument("--bits", type=int, default=8)
    parser.add_argument("--lsb-first", type=int, default=0)
    parser.add_argument("--delay-us", type=int, default=10)
    parser.add_argument("--verbose", action="store_true")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for name, help_text in (
        ("probe", "send only the update request and wait for ACK"),
        ("run", "perform the complete stop-and-wait OTA transfer"),
        ("run-dual", "upgrade CS0 first, then upgrade CS1 after CS0 succeeds"),
        ("step", "run OTA and pause before each command frame"),
    ):
        command = subparsers.add_parser(name, help=help_text)
        command.add_argument("ota_path", type=Path)
        add_timing_arguments(command)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "run-dual":
            devices = (args.spi_dev, args.second_spi_dev)
            results = []

            for index, device in enumerate(devices, start=1):
                device_args = argparse.Namespace(**vars(args))
                device_args.spi_dev = device
                print(f"\n=== Device {index}/2: {device}; starting from UPDATE_REQUEST ===")
                result = run_ota(device_args)
                results.append(result)
                if result == 0:
                    print(f"device {index} returned UPDATE_SUCCESS (0xFC)")
                else:
                    print(f"device {index} upgrade failed or timed out", file=sys.stderr)
                    print("stopping dual-device upgrade", file=sys.stderr)
                    return result

                if index < len(devices):
                    print(f"waiting {args.between_devices_ms} ms before the next device")
                    time.sleep(args.between_devices_ms / 1000.0)

            if all(result == 0 for result in results):
                print("both communication boards returned UPDATE_SUCCESS (0xFC)")
                return 0
            print("one or more communication boards failed", file=sys.stderr)
            return 1

        return run_ota(
            args,
            interactive=args.command == "step",
            request_only=args.command == "probe",
        )
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    except FileNotFoundError as exc:
        print(f"file not found: {exc}", file=sys.stderr)
        return 2
    except PermissionError as exc:
        print(f"permission denied: {exc}", file=sys.stderr)
        return 3
    except (OSError, ValueError, RuntimeError, struct.error) as exc:
        print(f"failed: {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    sys.exit(main())
