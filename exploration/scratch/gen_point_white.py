# Generates 16x16 colored circle PNGs for CustomGraphicsPointSet vertex sprites.
# One file per row color: resources/point_00.png through point_11.png.
# Anti-aliased circle, fully opaque center, 1px soft edge.
#
# Run once from the repo root:
#   /usr/bin/python3 exploration/scratch/gen_point_white.py

import struct
import zlib
import math
import os

SIZE   = 16
CENTER = (SIZE - 1) / 2.0   # 7.5
RADIUS = (SIZE / 2.0) - 1.5 # 6.5 — 1.5px margin for anti-aliasing

COLORS = [
    (255, 140,   0),  # 00 orange
    ( 30, 180, 255),  # 01 blue
    ( 50, 220,  50),  # 02 green
    (255, 230,   0),  # 03 yellow (slightly less blinding than pure 255,255,0)
    (255,  60, 255),  # 04 magenta
    (  0, 210, 185),  # 05 cyan
    (255,  80,  80),  # 06 red
    (160, 100, 255),  # 07 purple
    (255, 170,  80),  # 08 peach
    ( 80, 220, 140),  # 09 mint
    (200, 200, 200),  # 10 light grey
    (255, 140, 190),  # 11 pink
]


def _circle_alpha(px: float, py: float) -> int:
    dist = math.sqrt((px - CENTER) ** 2 + (py - CENTER) ** 2)
    t = dist - (RADIUS - 0.5)
    if t <= 0.0:
        return 255
    if t >= 1.0:
        return 0
    return int(round(255 * (1.0 - t)))


def _make_png(pixels_rgba: list) -> bytes:
    raw = b''
    for row in range(SIZE):
        raw += b'\x00'
        for col in range(SIZE):
            r, g, b, a = pixels_rgba[row * SIZE + col]
            raw += bytes([r, g, b, a])

    def chunk(tag, data):
        payload = tag + data
        return struct.pack('>I', len(data)) + payload + struct.pack('>I', zlib.crc32(payload) & 0xFFFFFFFF)

    ihdr = struct.pack('>II', SIZE, SIZE) + bytes([8, 6, 0, 0, 0])
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')


resources = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'src', 'Reverse', 'commands', 'fitSurfaces', 'resources'
)

alpha_mask = [_circle_alpha(col, row) for row in range(SIZE) for col in range(SIZE)]

for idx, (cr, cg, cb) in enumerate(COLORS):
    pixels = [(cr, cg, cb, a) for a in alpha_mask]
    path   = os.path.join(resources, f'point_{idx:02d}.png')
    with open(path, 'wb') as f:
        f.write(_make_png(pixels))
    print(f'  point_{idx:02d}.png  rgb({cr},{cg},{cb})')

print(f'\n{len(COLORS)} files written to {resources}')
