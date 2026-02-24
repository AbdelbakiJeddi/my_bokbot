#!/usr/bin/env python3
"""
Generate a clean map.pgm for the Eurobot 2026 arena.

Arena dimensions: 3.0m x 2.0m (standard Eurobot)
The arena3D model is placed at (1.026, 1.5255) with yaw=pi in the world.
This means the arena walls span approximately:
  X: from about -1.026 to 1.974  (3.0m total)
  Y: from about -0.4745 to 1.5255 ... but looking at the crates,
     the arena seems centered differently.

From the arena_world.sdf crate positions, the arena Y range goes from
about -1.5 to +1.5 (3.0m) and X range from about -1.0 to +0.5 (but
the arena3D pose suggests otherwise).

We'll use the arena3D pose as reference:
  center = (1.026, 1.5255), rotated 180 deg
  The mesh is 3000mm x 2000mm (scaled by 0.001)
  After 180 deg rotation and offset:
    X: 1.026 - 1.5 to 1.026 + 1.5 => -0.474 to 2.526... 
    
Actually, let's derive from the crate positions directly:
  Crates go from x=-0.875 to x=0.275, y=-1.475 to y=1.375
  The arena3D center at (1.026, 1.5255) with pi rotation means
  the original mesh center maps there.
  
For the competition, the standard Eurobot 2026 arena is 3m x 2m.
The world origin appears to be roughly at the center-left of the arena.

Let's just build the map based on the arena3D pose:
  arena center in world = (1.026, 1.5255)
  arena is 3m (along one axis) x 2m (along the other)
  rotated 180 degrees (which for a rectangle is same as 0)
  
  So arena spans:
    X: 1.026 - 1.5 = -0.474  to  1.026 + 1.5 = 2.526
    Y: 1.5255 - 1.0 = 0.5255  to  1.5255 + 1.0 = 2.5255
    
  But crates are at y=-1.375 to y=+1.375, which doesn't fit above.
  This means the arena mesh is 2m along X and 3m along Y, or the
  pose offset doesn't represent the center.

Let's just use the crate bounds + margin to define the arena:
  X range of crates: -0.875 to 0.275 => center ~ -0.3, span ~ 1.15
  Y range of crates: -1.475 to 1.425 => center ~ 0, span ~ 2.9

The standard Eurobot competition table is 3m (Y) x 2m (X).
Based on crate positions, the arena likely spans:
  X: -1.0 to 1.0    (2.0m)
  Y: -1.5 to 1.5    (3.0m)

This aligns well with the crate distribution being symmetric about Y=0.
"""

import numpy as np
import struct
import os

# ============================================================
# Arena parameters (Eurobot 2026: 2m x 3m)
# ============================================================
ARENA_X_MIN = -1.0    # meters
ARENA_X_MAX =  1.0    # meters  (2m total along X)
ARENA_Y_MIN = -1.5    # meters
ARENA_Y_MAX =  1.5    # meters  (3m total along Y)

WALL_THICKNESS = 0.025  # 2.5 cm thick walls (in meters)

# Map resolution: 0.02 m/pixel = 2cm per pixel (balance detail vs performance)
RESOLUTION = 0.02  # meters per pixel

# Large margin so planner never goes out of bounds
MARGIN = 1.5  # 1.5m margin around the arena

# ============================================================
# Compute map dimensions
# ============================================================
map_x_min = ARENA_X_MIN - MARGIN
map_x_max = ARENA_X_MAX + MARGIN
map_y_min = ARENA_Y_MIN - MARGIN
map_y_max = ARENA_Y_MAX + MARGIN

width  = int(round((map_x_max - map_x_min) / RESOLUTION))
height = int(round((map_y_max - map_y_min) / RESOLUTION))

print(f"Map size: {width} x {height} pixels")
print(f"Map covers: X[{map_x_min}, {map_x_max}] Y[{map_y_min}, {map_y_max}]")
print(f"Resolution: {RESOLUTION} m/px")

# ============================================================
# Create the image (white = free, black = occupied, gray = unknown)
# ============================================================
# PGM values: 254 = free (white), 0 = occupied (black), 205 = unknown
img = np.full((height, width), 254, dtype=np.uint8)  # Start all free


def world_to_pixel(wx, wy):
    """Convert world coords to pixel coords."""
    px = int(round((wx - map_x_min) / RESOLUTION))
    py = int(round((wy - map_y_min) / RESOLUTION))
    # PGM row 0 = top of image, but ROS map row 0 = bottom
    # We'll flip at the end
    return px, py


def draw_wall(x_start, y_start, x_end, y_end, thickness):
    """Draw a wall (filled rectangle) on the map."""
    # Expand by thickness/2 perpendicular to the wall
    x_lo = min(x_start, x_end)
    x_hi = max(x_start, x_end)
    y_lo = min(y_start, y_end)
    y_hi = max(y_start, y_end)

    # If wall is mostly horizontal (along X), expand in Y
    if (x_hi - x_lo) > (y_hi - y_lo):
        y_lo -= thickness / 2
        y_hi += thickness / 2
    else:  # vertical wall, expand in X
        x_lo -= thickness / 2
        x_hi += thickness / 2

    px_lo, py_lo = world_to_pixel(x_lo, y_lo)
    px_hi, py_hi = world_to_pixel(x_hi, y_hi)

    px_lo = max(0, min(px_lo, width - 1))
    px_hi = max(0, min(px_hi, width - 1))
    py_lo = max(0, min(py_lo, height - 1))
    py_hi = max(0, min(py_hi, height - 1))

    img[py_lo:py_hi+1, px_lo:px_hi+1] = 0  # black = occupied


# ============================================================
# Draw the 4 outer walls
# ============================================================
# Bottom wall (Y = ARENA_Y_MIN, along X)
draw_wall(ARENA_X_MIN, ARENA_Y_MIN, ARENA_X_MAX, ARENA_Y_MIN, WALL_THICKNESS)
# Top wall (Y = ARENA_Y_MAX, along X)
draw_wall(ARENA_X_MIN, ARENA_Y_MAX, ARENA_X_MAX, ARENA_Y_MAX, WALL_THICKNESS)
# Left wall (X = ARENA_X_MIN, along Y)
draw_wall(ARENA_X_MIN, ARENA_Y_MIN, ARENA_X_MIN, ARENA_Y_MAX, WALL_THICKNESS)
# Right wall (X = ARENA_X_MAX, along Y)
draw_wall(ARENA_X_MAX, ARENA_Y_MIN, ARENA_X_MAX, ARENA_Y_MAX, WALL_THICKNESS)

# ============================================================
# Mark outside arena as unknown (gray = 205)
# ============================================================
for py in range(height):
    for px in range(width):
        wx = map_x_min + px * RESOLUTION
        wy = map_y_min + py * RESOLUTION
        if wx < ARENA_X_MIN or wx > ARENA_X_MAX or wy < ARENA_Y_MIN or wy > ARENA_Y_MAX:
            if img[py, px] != 0:  # Don't overwrite walls
                img[py, px] = 205  # unknown

# ============================================================
# Flip vertically: PGM row 0 is top, but ROS map origin is bottom-left
# ============================================================
img = np.flipud(img)

# ============================================================
# Write PGM (P5 binary format)
# ============================================================
output_dir = os.path.dirname(os.path.abspath(__file__))
pgm_path = os.path.join(output_dir, "eurobot_2026.pgm")
yaml_path = os.path.join(output_dir, "eurobot_2026.yaml")

with open(pgm_path, 'wb') as f:
    header = f"P5\n{width} {height}\n255\n"
    f.write(header.encode('ascii'))
    f.write(img.tobytes())

print(f"Wrote PGM: {pgm_path} ({width}x{height})")

# ============================================================
# Write the YAML map metadata
# ============================================================
yaml_content = f"""image: eurobot_2026.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{map_x_min}, {map_y_min}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open(yaml_path, 'w') as f:
    f.write(yaml_content)

print(f"Wrote YAML: {yaml_path}")
print(f"Origin: [{map_x_min}, {map_y_min}, 0]")
print("Done!")
