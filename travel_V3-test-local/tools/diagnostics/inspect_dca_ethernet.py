#!/usr/bin/env python3
"""Passively inspect DCA Ethernet headers, including traffic to a wrong MAC/IP.

Requires Linux CAP_NET_RAW (normally run with sudo). Sends no packets, does not
configure the radar, and never saves ADC payloads. Promiscuous membership belongs
to this socket and is automatically removed when it closes.
"""
import argparse
import collections
import json
import select
import socket
import struct
import time
from pathlib import Path


def mac(raw):
    return ":".join(f"{value:02x}" for value in raw)


def decode_header(packet):
    if len(packet) < 14:
        return None
    result = {"source_mac": mac(packet[6:12]), "destination_mac": mac(packet[:6])}
    kind = struct.unpack_from("!H", packet, 12)[0]
    offset = 14
    while kind in (0x8100, 0x88A8):
        if len(packet) < offset + 4:
            return None
        kind = struct.unpack_from("!H", packet, offset + 2)[0]
        offset += 4
    if kind == 0x0806 and len(packet) >= offset + 28:
        hardware, protocol, hlen, plen, operation = struct.unpack_from("!HHBBH", packet, offset)
        if (hardware, protocol, hlen, plen) != (1, 0x0800, 6, 4):
            return None
        result.update(protocol="ARP", operation=operation,
                      source_ip=socket.inet_ntoa(packet[offset + 14:offset + 18]),
                      destination_ip=socket.inet_ntoa(packet[offset + 24:offset + 28]))
        return result
    if kind != 0x0800 or len(packet) < offset + 20:
        return None
    ihl = (packet[offset] & 15) * 4
    total = struct.unpack_from("!H", packet, offset + 2)[0]
    fragment = struct.unpack_from("!H", packet, offset + 6)[0]
    if packet[offset] >> 4 != 4 or ihl < 20 or packet[offset + 9] != 17 or fragment & 0x1FFF:
        return None
    udp = offset + ihl
    if total < ihl + 8 or len(packet) < udp + 8:
        return None
    sport, dport, length, _ = struct.unpack_from("!HHHH", packet, udp)
    if length < 8 or length > total - ihl:
        return None
    result.update(protocol="UDP", source_ip=socket.inet_ntoa(packet[offset + 12:offset + 16]),
                  destination_ip=socket.inet_ntoa(packet[offset + 16:offset + 20]),
                  source_port=sport, destination_port=dport, udp_payload_bytes=length - 8)
    # Only the eight-byte DCA control response is retained; no ADC samples.
    if length == 16 and len(packet) >= udp + 16:
        payload = packet[udp + 8:udp + 16]
        if payload[:2] == b"\x5a\xa5" and payload[-2:] == b"\xaa\xee":
            result["dca_control_hex"] = payload.hex(" ")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eno1")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--board-ip", default="192.168.33.180")
    parser.add_argument("--host-ip", default="192.168.33.30")
    parser.add_argument("--board-mac", default="0c:22:38:4e:5a:0c")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error("--seconds must be between 1 and 120")
    try:
        capture = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
    except PermissionError:
        parser.exit(1, "需要原始网卡抓包权限，请用 sudo python3 运行本脚本。\n")
    host_mac = (Path("/sys/class/net") / args.interface / "address").read_text().strip()
    flows = {}
    counts = collections.Counter()
    started = time.monotonic()
    with capture:
        capture.bind((args.interface, 0))
        capture.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        membership = struct.pack("IHH8s", socket.if_nametoindex(args.interface), 1, 0, b"\0" * 8)
        capture.setsockopt(263, 1, membership)  # SOL_PACKET, PACKET_ADD_MEMBERSHIP
        print(f"READY：监听 {args.interface}，本机 MAC={host_mac}；现在点击 GUI 开始采集。", flush=True)
        try:
            while time.monotonic() - started < args.seconds:
                remaining = args.seconds - (time.monotonic() - started)
                if not select.select([capture], [], [], max(0, min(1, remaining)))[0]:
                    continue
                # Reading only the first 128 bytes is sufficient for these headers.
                packet, address = capture.recvfrom(128)
                header = decode_header(packet)
                if not header:
                    continue
                if args.board_ip not in (header["source_ip"], header["destination_ip"]) and \
                        args.board_mac.lower() not in (header["source_mac"], header["destination_mac"]):
                    continue
                header["packet_type"] = address[2]  # 3=OTHERHOST, 4=OUTGOING
                key = tuple(header.get(field) for field in ("protocol", "source_mac", "destination_mac",
                            "source_ip", "destination_ip", "source_port", "destination_port", "dca_control_hex"))
                if key not in flows:
                    flows[key] = dict(header, packets=0)
                    print(json.dumps(header, ensure_ascii=False), flush=True)
                flows[key]["packets"] += 1
                counts[header["protocol"]] += 1
                if header["protocol"] == "UDP" and header["source_ip"] == args.board_ip and \
                        "dca_control_hex" not in header:
                    counts["non_control_udp_from_board"] += 1
                    if header["destination_ip"] != args.host_ip:
                        counts["different_destination_ip"] += 1
                    if header["destination_mac"] != host_mac:
                        counts["different_destination_mac"] += 1
                    if header["destination_port"] != 4098:
                        counts["different_destination_port"] += 1
        except KeyboardInterrupt:
            pass
    report = {"interface": args.interface, "host_mac": host_mac, "expected_host_ip": args.host_ip,
              "expected_data_port": 4098, "duration_sec": time.monotonic() - started,
              "counts": dict(counts), "flows": list(flows.values()), "adc_payload_saved": False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("SUMMARY", json.dumps(report["counts"], ensure_ascii=False))
    print("REPORT", args.output)


if __name__ == "__main__":
    main()
