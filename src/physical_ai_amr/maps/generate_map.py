#!/usr/bin/env python3
"""Generates a static occupancy grid (PGM + YAML) matching worlds/empty_world.sdf.

Not installed/run at build time; kept here so the map can be regenerated if the
world's obstacle_box pose or the patrol area changes.
"""
import os

RESOLUTION = 0.05  # meters/pixel
HALF_EXTENT = 6.0  # map covers [-6, 6] m in x and y
WIDTH = HEIGHT = int(2 * HALF_EXTENT / RESOLUTION)
ORIGIN_X = ORIGIN_Y = -HALF_EXTENT

# Matches the obstacle_box <pose> and <size> in worlds/empty_world.sdf.
OBSTACLE_CENTER = (2.0, 0.0)
OBSTACLE_SIZE = (0.4, 2.0)  # (x thickness, y width)

FREE = 254
OCCUPIED = 0


def is_occupied(x, y):
    hx, hy = OBSTACLE_SIZE[0] / 2.0, OBSTACLE_SIZE[1] / 2.0
    return (
        OBSTACLE_CENTER[0] - hx <= x <= OBSTACLE_CENTER[0] + hx
        and OBSTACLE_CENTER[1] - hy <= y <= OBSTACLE_CENTER[1] + hy
    )


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path = os.path.join(out_dir, 'patrol_world.pgm')
    yaml_path = os.path.join(out_dir, 'patrol_world.yaml')

    rows = []
    for row in range(HEIGHT):
        # Image row 0 is the top of the image = the maximum y in the world.
        y = ORIGIN_Y + (HEIGHT - 1 - row + 0.5) * RESOLUTION
        pixels = bytearray(WIDTH)
        for col in range(WIDTH):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            pixels[col] = OCCUPIED if is_occupied(x, y) else FREE
        rows.append(bytes(pixels))

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii'))
        for row in rows:
            f.write(row)

    with open(yaml_path, 'w') as f:
        f.write(
            'image: patrol_world.pgm\n'
            f'resolution: {RESOLUTION}\n'
            f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )

    print(f'Wrote {pgm_path} ({WIDTH}x{HEIGHT}) and {yaml_path}')


if __name__ == '__main__':
    main()
