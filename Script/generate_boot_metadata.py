"""F303 subアプリを検査し、bootloader用CRC32C metadataを生成する。"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

from orion_crc32c import crc32c


APP_BASE = 0x08004000
APP_SIZE = 0x0001B800
SRAM_BASE = 0x20000000
SRAM_END = 0x20008000
CCMRAM_BASE = 0x10000000
CCMRAM_END = 0x10002000
METADATA_MAGIC = 0x3157464F
METADATA_FORMAT = 1
METADATA_STATE_CONFIRMED = 4
METADATA_SLOT = 0
METADATA_RECORD_SIZE = 36


def validate_vector(image: bytes) -> tuple[int, int]:
    if len(image) < 8 or len(image) > APP_SIZE:
        raise ValueError(f"invalid application image size: {len(image)}")
    stack_pointer, reset_handler = struct.unpack_from("<II", image)
    stack_valid = SRAM_BASE <= stack_pointer <= SRAM_END or CCMRAM_BASE <= stack_pointer <= CCMRAM_END
    if not stack_valid or stack_pointer % 8 != 0:
        raise ValueError(f"invalid initial stack pointer: 0x{stack_pointer:08X}")
    handler_address = reset_handler & ~1
    if reset_handler & 1 == 0 or not APP_BASE <= handler_address < APP_BASE + len(image):
        raise ValueError(f"invalid application reset handler: 0x{reset_handler:08X}")
    return stack_pointer, reset_handler


def build_metadata(image: bytes, generation: int) -> bytes:
    record = struct.pack(
        "<IHHIIIIII",
        METADATA_MAGIC,
        METADATA_FORMAT,
        METADATA_RECORD_SIZE,
        generation,
        METADATA_STATE_CONFIRMED,
        METADATA_SLOT,
        APP_BASE,
        len(image),
        crc32c(image),
    )
    return record + struct.pack("<I", crc32c(record))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--generation", type=int, default=1)
    args = parser.parse_args()
    image = args.image.read_bytes()
    stack_pointer, reset_handler = validate_vector(image)
    metadata = build_metadata(image, args.generation)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(metadata)
    print(f"size={len(image)}")
    print(f"initial_sp=0x{stack_pointer:08X}")
    print(f"reset_handler=0x{reset_handler:08X}")
    print(f"image_crc32c=0x{crc32c(image):08X}")
    print(f"metadata_crc32c=0x{struct.unpack_from('<I', metadata, 32)[0]:08X}")
    print(f"metadata={args.output}")


if __name__ == "__main__":
    main()
