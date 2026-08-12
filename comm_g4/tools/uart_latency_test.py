#!/usr/bin/env python3
import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    raise


def build_frame(seq: int, length: int) -> bytes:
    if length < 4:
        raise ValueError("frame length must be at least 4")

    payload = bytearray(length)
    payload[0] = 0xA5
    payload[1] = seq & 0xFF
    payload[2] = (seq >> 8) & 0xFF

    for i in range(3, length - 1):
        payload[i] = (seq + i) & 0xFF

    payload[-1] = sum(payload[:-1]) & 0xFF
    return bytes(payload)


def verify_frame(tx: bytes, rx: bytes, echo: bool) -> bool:
    if len(rx) != len(tx):
        return False
    if echo:
        return rx == tx
    return True


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * pct / 100.0))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send fixed-length UART frames and measure echo latency."
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--length", type=int, default=32)
    parser.add_argument("--interval-ms", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--timeout-ms", type=float, default=100.0)
    parser.add_argument(
        "--no-echo-check",
        action="store_true",
        help="Only require the same response length, not identical data.",
    )
    parser.add_argument(
        "--read-grace-ms",
        type=float,
        default=0.0,
        help="Extra sleep after each write before reading; normally keep 0.",
    )
    parser.add_argument(
        "--show-data",
        action="store_true",
        help="Print TX and RX frame data in hex.",
    )
    args = parser.parse_args()

    interval_s = args.interval_ms / 1000.0
    timeout_s = args.timeout_ms / 1000.0
    echo_check = not args.no_echo_check

    latencies_ms = []
    bad_frames = 0
    timeouts = 0

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=timeout_s,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        next_send = time.perf_counter()
        for seq in range(args.count):
            now = time.perf_counter()
            if now < next_send:
                time.sleep(next_send - now)

            frame = build_frame(seq, args.length)
            ser.reset_input_buffer()

            t0 = time.perf_counter()
            ser.write(frame)
            ser.flush()

            if args.read_grace_ms > 0:
                time.sleep(args.read_grace_ms / 1000.0)

            rx = bytearray()
            deadline = t0 + timeout_s
            while len(rx) < args.length and time.perf_counter() < deadline:
                chunk = ser.read(args.length - len(rx))
                if chunk:
                    rx.extend(chunk)
                else:
                    time.sleep(0.0002)

            t1 = time.perf_counter()

            if len(rx) < args.length:
                timeouts += 1
                if args.show_data:
                    print(
                        f"seq={seq:04d} timeout "
                        f"tx={frame.hex(' ')} rx={bytes(rx).hex(' ')}"
                    )
            elif verify_frame(frame, bytes(rx), echo_check):
                latencies_ms.append((t1 - t0) * 1000.0)
                if args.show_data:
                    print(
                        f"seq={seq:04d} ok "
                        f"latency_ms={(t1 - t0) * 1000.0:.3f} "
                        f"tx={frame.hex(' ')} rx={bytes(rx).hex(' ')}"
                    )
            else:
                bad_frames += 1
                if args.show_data:
                    print(
                        f"seq={seq:04d} bad "
                        f"tx={frame.hex(' ')} rx={bytes(rx).hex(' ')}"
                    )

            next_send += interval_s

            if (seq + 1) % 100 == 0:
                print(
                    f"sent={seq + 1} ok={len(latencies_ms)} "
                    f"timeout={timeouts} bad={bad_frames}"
                )

    print()
    print(f"port={args.port} baud={args.baud} length={args.length}")
    print(
        f"interval_ms={args.interval_ms:.3f} count={args.count} "
        f"ok={len(latencies_ms)} timeout={timeouts} bad={bad_frames}"
    )

    if latencies_ms:
        print(f"min_ms={min(latencies_ms):.3f}")
        print(f"avg_ms={statistics.mean(latencies_ms):.3f}")
        print(f"p50_ms={statistics.median(latencies_ms):.3f}")
        print(f"p95_ms={percentile(latencies_ms, 95):.3f}")
        print(f"max_ms={max(latencies_ms):.3f}")

    return 0 if timeouts == 0 and bad_frames == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
