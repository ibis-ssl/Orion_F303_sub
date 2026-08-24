"""Orion F303 FW成果物とFlash backupに共通のCRC32Cを計算する。"""

from __future__ import annotations

import argparse
from pathlib import Path


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return (~crc) & 0xFFFFFFFF


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)
    args = parser.parse_args()
    print(f"0x{crc32c(args.file.read_bytes()):08X}")


if __name__ == "__main__":
    main()
