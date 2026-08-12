#!/usr/bin/env python3
"""解析并校验 OTA 文件。

当前 OTA 格式：
    0x00-0x03  file_CRC32         原始 bin payload 的 CRC32
    0x04-0x07  file_Size          整个 OTA 文件总长度
    0x08-0x0B  Software_version   软件版本
    0x0C-0x0F  Hardware_version   硬件版本
    0x10-EOF   payload            原始 bin 数据

当前校验规则：
    file_CRC32 只针对 payload 计算，不包含 16 字节头部。
"""

from __future__ import annotations

import argparse
import binascii
import struct
from dataclasses import dataclass
from pathlib import Path


HEADER_FORMAT = "<IIII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
UINT32_MAX = 0xFFFFFFFF


@dataclass(slots=True)
class OtaHeader:
    file_crc32: int
    file_size: int
    software_version: int
    hardware_version: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "OtaHeader":
        if len(data) < HEADER_SIZE:
            raise ValueError("OTA file is smaller than the 16-byte header.")
        return cls(*struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE]))


@dataclass(slots=True)
class OtaImage:
    path: Path
    header: OtaHeader
    payload: bytes
    actual_size: int

    @property
    def payload_crc32(self) -> int:
        return binascii.crc32(self.payload) & UINT32_MAX

    @property
    def is_size_valid(self) -> bool:
        return self.header.file_size == len(self.payload)

    @property
    def is_crc_valid(self) -> bool:
        return self.header.file_crc32 == self.payload_crc32

    def validate(self) -> None:
        if not self.is_size_valid:
            raise ValueError(
                f"Size mismatch: header={self.header.file_size}, payload={len(self.payload)}"
            )
        if not self.is_crc_valid:
            raise ValueError(
                "CRC mismatch: "
                f"header=0x{self.header.file_crc32:08X}, "
                f"payload=0x{self.payload_crc32:08X}"
            )

    def print_summary(self) -> None:
        print(f"File    : {self.path}")
        print(f"Payload : {len(self.payload)} bytes")
        print(f"OTA Size: {self.actual_size} bytes")
        print(f"CRC32   : 0x{self.header.file_crc32:08X}")
        print(f"SW Ver  : 0x{self.header.software_version:08X}")
        print(f"HW Ver  : 0x{self.header.hardware_version:08X}")
        print(f"Check   : size={'OK' if self.is_size_valid else 'FAIL'}")
        print(f"Check   : crc ={'OK' if self.is_crc_valid else 'FAIL'}")


class OtaParser:
    @staticmethod
    def parse(path: Path) -> OtaImage:
        data = path.read_bytes()
        header = OtaHeader.from_bytes(data)
        payload = data[HEADER_SIZE:]
        return OtaImage(
            path=path,
            header=header,
            payload=payload,
            actual_size=len(data),
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Parse and validate an OTA file."
    )
    parser.add_argument("input", nargs="?", type=Path, help="Input .ota file path")
    parser.add_argument(
        "--extract",
        type=Path,
        help="Optional output path for extracting the payload bin.",
    )
    return parser


def select_input_file(script_dir: Path) -> Path:
    ota_files = sorted(script_dir.glob("*.ota"))
    if not ota_files:
        raise FileNotFoundError(f"No .ota files found in {script_dir}")

    if len(ota_files) == 1:
        selected = ota_files[0]
        print(f"Using input file: {selected}")
        return selected

    print("Found multiple .ota files:")
    for index, path in enumerate(ota_files, start=1):
        print(f"  {index}. {path.name}")

    while True:
        choice = input("Select input file number: ").strip()
        try:
            selected_index = int(choice)
        except ValueError:
            print("Please enter a valid number.")
            continue

        if 1 <= selected_index <= len(ota_files):
            selected = ota_files[selected_index - 1]
            print(f"Using input file: {selected}")
            return selected

        print("Selection out of range.")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    script_dir = Path(__file__).resolve().parent

    if args.input is None:
        try:
            input_path = select_input_file(script_dir)
        except Exception as exc:
            print(f"Parse failed: {exc}")
            return 1
    else:
        input_path = args.input.resolve()

    if not input_path.is_file():
        parser.error(f"Input file not found: {input_path}")

    try:
        image = OtaParser.parse(input_path)
        image.validate()
    except Exception as exc:
        print(f"Parse failed: {exc}")
        return 1

    image.print_summary()

    if args.extract:
        output_path = args.extract.resolve()
        output_path.write_bytes(image.payload)
        print(f"Extract : {output_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
