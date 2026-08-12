#!/usr/bin/env python3
"""将原始 MCU 固件 .bin 打包为带 16 字节头部的 OTA 文件。

文件格式说明：
    整个 OTA 文件 = 16 字节头部 + 原始 bin 数据

头部布局（4 个 uint32，小端）：
    0x00-0x03: file_CRC32
        整个 OTA 文件的 CRC32。
        计算时先把本字段临时置 0，再对“整个 OTA 文件”做 CRC32，
        最后把结果回填到本字段。

    0x04-0x07: file_Size
        OTA 文件总大小，单位字节。
        注意：这里不是 bin 裸数据大小，而是“头部 + bin”的总长度。

    0x08-0x0B: Software_version
        软件版本号，4 字节无符号整数。
        可直接写十六进制，例如 0x01020304；
        也支持点分格式 1.2.3.4，脚本会编码为 0x01020304。

    0x0C-0x0F: Hardware_version
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
from pathlib import Path


# =========================
# 用户配置区
# =========================
# 平时只需要改这一段配置，下面的逻辑一般不用动。

# 默认的软件版本号。
# 例如 0x01020304 表示 1.2.3.4
DEFAULT_SOFTWARE_VERSION = 0xE0000001

# 默认的硬件版本号。
# 例如 0x00000001 表示硬件版本 1
DEFAULT_HARDWARE_VERSION = 0xB000031

# 默认输出文件后缀。
# 例如输入是 CommBoard_031.bin，则输出会是 CommBoard_031.ota
DEFAULT_OUTPUT_SUFFIX = ".ota"

# 如果这里填写具体文件名，则无参数运行时优先使用这个 bin 文件。
# 例如：
# DEFAULT_INPUT_FILENAME = "CommBoard_031.bin"
# 留空字符串表示自动扫描脚本目录下的 .bin 文件。
DEFAULT_INPUT_FILENAME = "CommBoard_031.bin"


# =========================
# 协议常量区
# =========================
# OTA 固定头部长度：4 个 uint32 = 16 字节
HEADER_SIZE = 16

# uint32 的最大值，用于范围检查
UINT32_MAX = 0xFFFFFFFF


def parse_u32(value: str) -> int:
    """把用户输入解析成 uint32。

    支持两种输入格式：
    1. 普通整数/十六进制，例如：
       123
       0x01020304

    2. 点分版本号，例如：
       1.2.3.4

    点分版本号会按“每段占 1 字节”的方式编码：
       1.2.3.4 -> 0x01020304
    """
    value = value.strip()
    if "." in value:
        parts = value.split(".")
        if not 1 <= len(parts) <= 4:
            raise argparse.ArgumentTypeError(
                "Dotted version must contain 1 to 4 parts, e.g. 1.2.3.4"
            )

        # 将点分版本逐段拼成一个 32 位整数。
        # 例如 [1, 2, 3, 4] 会得到 0x01020304
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
            # 左移 8 位后拼接下一个字节
            result = (result << 8) | byte_value
        return result

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


def build_header(file_crc32: int, file_size: int, sw_version: int, hw_version: int) -> bytes:
    """按既定协议构造 16 字节头部。

    这里使用小端字节序：
        <IIII

    含义如下：
        <   : 小端
        I   : uint32
        I   : uint32
        I   : uint32
        I   : uint32
    """
    return struct.pack("<IIII", file_crc32, file_size, sw_version, hw_version)


def package_firmware(input_path: Path, output_path: Path, sw_version: int, hw_version: int) -> tuple[int, int]:
    """执行打包流程。

    流程：
    1. 读取原始 bin 数据
    2. 计算 OTA 文件总长度 = 16 字节头 + bin 数据
    3. 先构造一个 CRC 字段为 0 的临时头部
    4. 对“临时头部 + bin 数据”整体计算 CRC32
    5. 将真实 CRC32 写回头部
    6. 输出最终 OTA 文件

    返回值：
        (file_crc32, total_size)
    """
    # 读取原始 MCU 固件内容
    payload = input_path.read_bytes()

    # file_Size 字段定义为“整个 OTA 文件长度”
    total_size = HEADER_SIZE + len(payload)
    if total_size > UINT32_MAX:
        raise ValueError("Packaged file is too large to fit into a uint32 size field.")

    # 先构造一个 CRC 字段为 0 的临时头，用于参与 CRC 计算
    provisional_header = build_header(0, total_size, sw_version, hw_version)
    provisional_image = provisional_header + payload

    # 根据协议，对整个 OTA 文件做 CRC32，但计算时 CRC 字段本身为 0
    file_crc32 = binascii.crc32(provisional_image) & UINT32_MAX

    # 将真实 CRC32 回填到头部，得到最终 OTA 文件
    final_header = build_header(file_crc32, total_size, sw_version, hw_version)
    output_path.write_bytes(final_header + payload)
    return file_crc32, total_size


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
        help="Output package path. Defaults to <input_stem>.ota in the same directory.",
    )
    parser.add_argument(
        "--sw",
        "--software-version",
        dest="sw_version",
        type=parse_u32,
        help="Optional software version override. Supports decimal, hex, or dotted bytes such as 1.2.3.4",
    )
    parser.add_argument(
        "--hw",
        "--hardware-version",
        dest="hw_version",
        type=parse_u32,
        help="Optional hardware version override. Supports decimal, hex, or dotted bytes such as 1.0.0.0",
    )
    return parser


def select_input_file(script_dir: Path) -> Path:
    """在脚本所在目录中自动选择要打包的 .bin 文件。

    规则：
    1. 如果配置了 DEFAULT_INPUT_FILENAME，则优先使用该文件
    2. 如果目录下没有 .bin 文件，则报错
    3. 如果只有一个 .bin 文件，则直接使用
    4. 如果有多个 .bin 文件，则让用户输入序号选择
    """
    # 如果用户在配置区写死了输入文件名，则优先使用该文件
    if DEFAULT_INPUT_FILENAME:
        selected = script_dir / DEFAULT_INPUT_FILENAME
        if not selected.is_file():
            raise FileNotFoundError(
                f"Configured input file not found: {selected}"
            )
        print(f"Using configured input file: {selected}")
        return selected

    bin_files = sorted(script_dir.glob("*.bin"))
    if not bin_files:
        raise FileNotFoundError(f"No .bin files found in {script_dir}")

    if len(bin_files) == 1:
        selected = bin_files[0]
        print(f"Using input file: {selected}")
        return selected

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


def main() -> int:
    """程序入口。

    支持两种使用方式：

    1. 无参数执行
       - 自动在脚本目录查找 .bin
       - 使用默认软件/硬件版本
       - 输出同名 .ota 文件

    2. 命令行执行
       - 可手动指定输入文件
       - 可通过 --sw / --hw 临时覆盖默认版本号
       - 可通过 -o 指定输出路径
    """
    parser = create_parser()
    args = parser.parse_args()

    # 脚本所在目录。
    # 无参数运行时，会在这个目录里自动寻找 .bin 文件。
    script_dir = Path(__file__).resolve().parent

    if args.input is None:
        try:
            input_path = select_input_file(script_dir)
        except Exception as exc:
            print(f"Packaging failed: {exc}", file=sys.stderr)
            return 1
        # 无参数模式下，若用户未额外传入版本号，则使用脚本顶部的默认常量
        sw_version = args.sw_version if args.sw_version is not None else DEFAULT_SOFTWARE_VERSION
        hw_version = args.hw_version if args.hw_version is not None else DEFAULT_HARDWARE_VERSION
        output_path = (
            args.output.resolve()
            if args.output
            else input_path.with_suffix(DEFAULT_OUTPUT_SUFFIX)
        )
    else:
        # 指定了输入文件时，按用户给定路径打包
        input_path = args.input.resolve()
        if not input_path.is_file():
            parser.error(f"Input file not found: {input_path}")

        # 如果命令行没有显式传版本号，则仍然使用脚本顶部默认常量
        sw_version = args.sw_version if args.sw_version is not None else DEFAULT_SOFTWARE_VERSION
        hw_version = args.hw_version if args.hw_version is not None else DEFAULT_HARDWARE_VERSION
        output_path = (
            args.output.resolve()
            if args.output
            else input_path.with_suffix(DEFAULT_OUTPUT_SUFFIX)
        )

    try:
        file_crc32, total_size = package_firmware(
            input_path=input_path,
            output_path=output_path,
            sw_version=sw_version,
            hw_version=hw_version,
        )
    except Exception as exc:
        print(f"Packaging failed: {exc}", file=sys.stderr)
        return 1

    # 打印结果，便于上位机联调或人工核对
    print(f"Input   : {input_path}")
    print(f"Output  : {output_path}")
    print(f"Size    : {total_size} bytes")
    print(f"CRC32   : 0x{file_crc32:08X}")
    print(f"SW Ver  : 0x{sw_version:08X}")
    print(f"HW Ver  : 0x{hw_version:08X}")
    return 0


if __name__ == "__main__":
    # 使用显式的退出码，方便脚本集成到自动化流程
    raise SystemExit(main())
