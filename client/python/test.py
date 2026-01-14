#!/usr/bin/env python3
import argparse
import socket
import struct
import time


def _decode_u16_array(data: bytes, count: int, endian: str) -> list[int]:
    fmt = ("<" if endian == "little" else ">") + f"{count}H"
    return list(struct.unpack(fmt, data))


def _auto_decode_pwms(data: bytes, count: int) -> tuple[list[int], str]:
    little = _decode_u16_array(data, count, "little")
    big = _decode_u16_array(data, count, "big")

    def score(vals: list[int]) -> int:
        return sum(1 for v in vals if 800 <= v <= 2200)

    return (big, "big") if score(big) > score(little) else (little, "little")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Listen for ArduPilot rotor PWM packets (RotorControlMessage: 11x uint16)."
    )
    p.add_argument(
        "--host",
        default="0.0.0.0",
        help="Local bind IP (0.0.0.0 listens on all interfaces).",
    )
    p.add_argument("--port", type=int, default=9002, help="Local UDP port (default: 9002).")
    p.add_argument("--count", type=int, default=11, help="uint16 count (default: 11).")
    p.add_argument("--endian", choices=["auto", "little", "big"], default="auto")
    p.add_argument("--filter-ip", default="", help="If set, only accept packets from this source IP.")
    p.add_argument("--print-hz", type=float, default=5.0, help="Max print rate; 0 prints every packet.")
    p.add_argument("--normalize", action="store_true", help="Also print normalized [0..1] values.")
    args = p.parse_args()

    expected_len = args.count * 2
    print_period = 0.0 if args.print_hz <= 0 else 1.0 / args.print_hz

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except OSError:
        pass

    try:
        sock.bind((args.host, args.port))
    except OSError as exc:
        msg = (
            f"Failed to bind UDP {args.host}:{args.port}: {exc}\n"
            "Tips:\n"
            "- Use `--host 0.0.0.0` to listen on all local interfaces.\n"
            "- Don't bind to a *remote* IP (e.g. your WSL guest IP); bind locally and optionally use `--filter-ip`.\n"
        )
        raise SystemExit(msg) from exc
    print(f"Listening on UDP {args.host}:{args.port} (expecting {expected_len} bytes per packet).")
    if args.filter_ip:
        print(f"Filtering: only source IP {args.filter_ip}")

    t0 = time.monotonic()
    last_pkt_t = None
    last_print_t = 0.0
    good = 0
    total = 0

    try:
        while True:
            data, (src_ip, src_port) = sock.recvfrom(4096)
            now = time.monotonic()
            total += 1

            if args.filter_ip and src_ip != args.filter_ip:
                continue

            if len(data) != expected_len:
                if now - last_print_t >= max(print_period, 1.0):
                    print(f"{src_ip}:{src_port} len={len(data)} (expected {expected_len}) data={data.hex()}")
                    last_print_t = now
                continue

            if args.endian == "auto":
                pwms, endian_used = _auto_decode_pwms(data, args.count)
            else:
                pwms = _decode_u16_array(data, args.count, args.endian)
                endian_used = args.endian

            good += 1
            inst_hz = None
            if last_pkt_t is not None:
                dt = now - last_pkt_t
                if dt > 0:
                    inst_hz = 1.0 / dt
            last_pkt_t = now

            avg_hz = good / (now - t0) if now > t0 else 0.0
            if print_period == 0.0 or (now - last_print_t) >= print_period:
                inst_s = "n/a" if inst_hz is None else f"{inst_hz:.1f}"
                line = f"{src_ip}:{src_port} endian={endian_used} pwms={pwms} hz(inst/avg)={inst_s}/{avg_hz:.1f}"
                if args.normalize:
                    norm = [(v - 1000.0) / 1000.0 for v in pwms]
                    line += " norm=[" + ",".join(f"{x:.3f}" for x in norm) + "]"
                print(line)
                last_print_t = now

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        elapsed = time.monotonic() - t0
        print(f"Done. good={good} total={total} elapsed={elapsed:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
