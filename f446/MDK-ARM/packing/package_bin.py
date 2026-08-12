#!/usr/bin/env python3
"""将原始 MCU 固件 .bin 打包为带 16 字节头部的 OTA 文件。

OTA 文件格式：
    整个文件 = 16 字节头部 + 原始 bin 数据

头部布局（4 个 uint32，小端）：
    0x00-0x03  file_CRC32
        原始 bin 数据区的 CRC32。
        该值只针对 bin 内容计算，不包含 16 字节头部。

    0x04-0x07  file_Size
        OTA 文件总长度，单位字节。
        注意这里是“头部 + bin 数据”的总长度，不是 bin 裸数据长度。

    0x08-0x0B  Software_version
        软件版本号，4 字节无符号整数。
        支持十进制、十六进制或点分格式 1.2.3.4。

    0x0C-0x0F  Hardware_version
        硬件版本号，4 字节无符号整数。
        编码规则与软件版本一致。

示例：
    python package_bin.py
    python package_bin.py CommBoard_031.bin
    python package_bin.py CommBoard_031.bin --sw 0x00010002 --hw 0x00000001
"""

from __future__ import annotations

import argparse
import binascii
import struct
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

HEADER_FORMAT = "<IIII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
UINT32_MAX = 0xFFFFFFFF


@dataclass(slots=True)
class PackageConfig:
    """用户配置区。

    平时只需要调整这里的配置，下面的逻辑通常不需要改。
    """

    default_software_version: int = 0xE0000003
    default_hardware_version: int = 0xB0003100
    default_output_suffix: str = ".ota"
    append_date_to_output_filename: bool = True
    default_input_filename: str = "../CommBoard_031/CommBoard_031.bin"

    def build_default_output_path(self, input_path: Path) -> Path:
        """当前路径默认输出路径。"""
        output_name = input_path.stem
        if self.append_date_to_output_filename:
            output_name = f"{output_name}_{datetime.now():%Y%m%d}"
        return Path.cwd() / f"{output_name}{self.default_output_suffix}"

    def resolve_sw_version(self, override: int | None) -> int:
        """优先使用命令行覆盖值，否则使用默认软件版本。"""
        return override if override is not None else self.default_software_version

    def resolve_hw_version(self, override: int | None) -> int:
        """优先使用命令行覆盖值，否则使用默认硬件版本。"""
        return override if override is not None else self.default_hardware_version


@dataclass(slots=True)
class OtaHeader:
    """OTA 文件头。"""

    file_crc32: int
    file_size: int
    software_version: int
    hardware_version: int

    def to_bytes(self) -> bytes:
        """按协议将头部编码为 16 字节小端数据。"""
        return struct.pack(
            HEADER_FORMAT,
            self.file_crc32,
            self.file_size,
            self.software_version,
            self.hardware_version,
        )

    @classmethod
    def provisional(
        cls,
        file_size: int,
        software_version: int,
        hardware_version: int,
    ) -> OtaHeader:
        """构造一个占位头部。"""
        return cls(
            file_crc32=0,
            file_size=file_size,
            software_version=software_version,
            hardware_version=hardware_version,
        )


@dataclass(slots=True)
class PackageResult:
    """打包结果。"""

    input_path: Path
    output_path: Path
    file_crc32: int
    total_size: int
    software_version: int
    hardware_version: int

    def print_summary(self) -> None:
        """打印打包结果，便于人工核对和上位机联调。"""
        print(f"Input   : {self.input_path}")
        print(f"Output  : {self.output_path}")
        print(f"Size    : {self.total_size} bytes")
        print(f"CRC32   : 0x{self.file_crc32:08X}")
        print(f"SW Ver  : 0x{self.software_version:08X}")
        print(f"HW Ver  : 0x{self.hardware_version:08X}")


class VersionParser:
    """版本号解析器。"""

    @staticmethod
    def parse_u32(value: str) -> int:
        """把用户输入解析成 uint32。

        支持三种常用写法：
        - 十进制：123
        - 十六进制：0x01020304
        - 点分格式：1.2.3.4

        点分格式会按每段 1 字节进行编码：
            1.2.3.4 -> 0x01020304
        """
        value = value.strip()
        if "." in value:
            return VersionParser._parse_dotted_version(value)
        return VersionParser._parse_integer(value)

    @staticmethod
    def _parse_dotted_version(value: str) -> int:
        parts = value.split(".")
        if not 1 <= len(parts) <= 4:
            raise argparse.ArgumentTypeError(
                "Dotted version must contain 1 to 4 parts, e.g. 1.2.3.4"
            )

        result = 0
        for part in parts:
            try:
                byte_value = int(part, 10)
            except ValueError as exc:
                raise argparse.ArgumentTypeError(
                    f"Invalid dotted version component: {part}"
                ) from exc

            if not 0 <= byte_value <= 0xFF:
                raise argparse.ArgumentTypeError(
                    f"Dotted version component out of range 0-255: {part}"
                )

            result = (result << 8) | byte_value
        return result

    @staticmethod
    def _parse_integer(value: str) -> int:
        try:
            result = int(value, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid uint32 value: {value}. Use decimal, hex, or 1.2.3.4."
            ) from exc

        if not 0 <= result <= UINT32_MAX:
            raise argparse.ArgumentTypeError(
                f"Value out of range for uint32: {value}"
            )
        return result


class FirmwarePackager:
    """固件打包器。"""

    def __init__(self, config: PackageConfig, script_dir: Path) -> None:
        self.config = config
        self.script_dir = script_dir

    @staticmethod
    def create_parser() -> argparse.ArgumentParser:
        """创建命令行参数解析器。"""
        parser = argparse.ArgumentParser(
            description="Package a raw .bin into an OTA image with a 16-byte header."
        )
        parser.add_argument("input", nargs="?", type=Path, help="Input .bin file path")
        parser.add_argument(
            "-o",
            "--output",
            type=Path,
            help=(
                "Output package path. Defaults to "
                "<input_stem>_YYYYMMDD.ota in the same directory."
            ),
        )
        parser.add_argument(
            "--sw",
            "--software-version",
            dest="sw_version",
            type=VersionParser.parse_u32,
            help=(
                "Optional software version override. "
                "Supports decimal, hex, or dotted bytes such as 1.2.3.4"
            ),
        )
        parser.add_argument(
            "--hw",
            "--hardware-version",
            dest="hw_version",
            type=VersionParser.parse_u32,
            help=(
                "Optional hardware version override. "
                "Supports decimal, hex, or dotted bytes such as 1.0.0.0"
            ),
        )
        return parser

    def package(
        self,
        input_path: Path,
        output_path: Path,
        software_version: int,
        hardware_version: int,
    ) -> PackageResult:
        """执行打包流程。

        CRC32 只针对原始 bin 数据计算，不包含 16 字节 OTA 头部。
        """
        payload = input_path.read_bytes()
        total_size = HEADER_SIZE + len(payload)
        if total_size > UINT32_MAX:
            raise ValueError("Packaged file is too large to fit into a uint32 size field.")

        file_crc32 = binascii.crc32(payload) & UINT32_MAX

        final_header = OtaHeader(
            file_crc32=file_crc32,
            file_size=total_size,
            software_version=software_version,
            hardware_version=hardware_version,
        )
        output_path.write_bytes(final_header.to_bytes() + payload)

        return PackageResult(
            input_path=input_path,
            output_path=output_path,
            file_crc32=file_crc32,
            total_size=total_size,
            software_version=software_version,
            hardware_version=hardware_version,
        )

    def resolve_input_path(self, input_path: Path | None) -> Path:
        """解析本次打包使用的输入文件。"""
        if input_path is not None:
            resolved_path = input_path.resolve()
            if not resolved_path.is_file():
                raise FileNotFoundError(f"Input file not found: {resolved_path}")
            return resolved_path
        return self._select_default_input_file()

    def resolve_output_path(self, input_path: Path, output_path: Path | None) -> Path:
        """解析输出文件路径。"""
        if output_path is not None:
            return output_path.resolve()
        return self.config.build_default_output_path(input_path)

    def _select_default_input_file(self) -> Path:
        """无参数运行时自动选择要打包的 .bin 文件。"""
        if self.config.default_input_filename:
            selected = self.script_dir / self.config.default_input_filename
            if not selected.is_file():
                raise FileNotFoundError(f"Configured input file not found: {selected}")
            # print(f"Using configured input file: {selected}")
            return selected

        bin_files = sorted(self.script_dir.glob("*.bin"))
        if not bin_files:
            raise FileNotFoundError(f"No .bin files found in {self.script_dir}")

        if len(bin_files) == 1:
            selected = bin_files[0]
            # print(f"Using input file: {selected}")
            return selected

        return self._prompt_for_input_file(bin_files)

    @staticmethod
    def _prompt_for_input_file(bin_files: list[Path]) -> Path:
        """当目录下存在多个 .bin 时，让用户选择一个。"""
        print("Found multiple .bin files:")
        for index, path in enumerate(bin_files, start=1):
            print(f"  {index}. {path.name}")

        while True:
            choice = input("Select input file number: ").strip()
            try:
                selected_index = int(choice)
            except ValueError:
                print("Please enter a valid number.")
                continue

            if 1 <= selected_index <= len(bin_files):
                selected = bin_files[selected_index - 1]
                print(f"Using input file: {selected}")
                return selected

            print("Selection out of range.")


class PackApplication:
    """应用入口，负责串联参数解析、路径解析和打包流程。"""

    def __init__(self, config: PackageConfig) -> None:
        self.config = config
        self.script_dir = Path(__file__).resolve().parent
        self.packager = FirmwarePackager(config=config, script_dir=self.script_dir)
    def run(self, argv: list[str] | None = None) -> int:
        """运行应用。

        支持两种方式：
        1. 无参数运行：自动选择默认 bin，并使用配置中的默认版本号
        2. 命令行运行：可指定输入文件、输出路径和版本号覆盖值
        """
        parser = self.packager.create_parser()
        args = parser.parse_args(argv)

        try:
            input_path = self.packager.resolve_input_path(args.input)
            output_path = self.packager.resolve_output_path(input_path, args.output)
            software_version = self.config.resolve_sw_version(args.sw_version)
            hardware_version = self.config.resolve_hw_version(args.hw_version)

            result = self.packager.package(
                input_path=input_path,
                output_path=output_path,
                software_version=software_version,
                hardware_version=hardware_version,
            )
        except FileNotFoundError as exc:
            parser.error(str(exc))
        except Exception as exc:
            print(f"Packaging failed: {exc}", file=sys.stderr)
            return 1

        result.print_summary()
        return 0


def main() -> int:
    """程序入口。"""
    app = PackApplication(config=PackageConfig())
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
