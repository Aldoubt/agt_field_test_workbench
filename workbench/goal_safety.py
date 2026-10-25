"""Conservative static-map endpoint checks; Nav2 still owns path collision checking.

Map pixels come from a versioned PGM (top row first), while Nav2 map origin
refers to the *bottom-left* cell and can have a nonzero yaw.  This check never
marks Unknown as free. It is independent of dynamic obstacle avoidance.
"""
import math
from pathlib import Path

from .maps import read_pgm


class NavigableGrid:
    def __init__(self, record):
        width, height, pixels = read_pgm(Path(record['pgm']))
        if (width, height) != (record['width'], record['height']):
            raise ValueError('map dimensions differ from manifest')
        self.width, self.height, self.pixels = width, height, pixels
        self.resolution = float(record['resolution'])
        self.origin = tuple(float(v) for v in record['origin'])
        if (not math.isfinite(self.resolution) or self.resolution <= 0 or
                len(self.origin) != 3 or not all(map(math.isfinite, self.origin))):
            raise ValueError('invalid map geometry')

    def cell(self, x, y):
        if not all(map(math.isfinite, (x, y))):
            raise ValueError('non-finite target position')
        dx, dy = x - self.origin[0], y - self.origin[1]
        yaw = self.origin[2]
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return math.floor(local_x / self.resolution), self.height - 1 - math.floor(local_y / self.resolution)

    def require_clear(self, point, clearance_m=0.0):
        if not math.isfinite(clearance_m) or clearance_m < 0:
            raise ValueError('invalid target clearance')
        column, row = self.cell(float(point['x']), float(point['y']))
        radius_cells = math.ceil(clearance_m / self.resolution)
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                # A conservative collision disk: include cells whose *square*
                # could intersect the radius, not just their center.
                if clearance_m and math.hypot(max(abs(dx) - .5, 0),
                                               max(abs(dy) - .5, 0)) * self.resolution > clearance_m:
                    continue
                c, r = column + dx, row + dy
                if not (0 <= c < self.width and 0 <= r < self.height):
                    raise ValueError('target or footprint falls outside known map')
                if self.pixels[r * self.width + c] < 250:
                    raise ValueError('target or footprint overlaps occupied/unknown map; choose a verified free cell')
        return True


def footprint_clearance(profile, margin_m=0.10):
    vertices = profile['footprint_m']
    if not isinstance(vertices, list) or len(vertices) < 3:
        raise ValueError('robot footprint polygon required')
    radius = 0.0
    for item in vertices:
        if not isinstance(item, (tuple, list)) or len(item) != 2:
            raise ValueError('invalid robot footprint')
        x, y = item
        if (isinstance(x, bool) or isinstance(y, bool) or
                not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or
                not math.isfinite(x) or not math.isfinite(y)):
            raise ValueError('invalid robot footprint')
        radius = max(radius, math.hypot(x, y))
    return radius + margin_m


def validate_endpoints(record, points, profile=None):
    grid = NavigableGrid(record)
    margin = footprint_clearance(profile) if profile is not None else 0.0
    for index, point in enumerate(points, 1):
        try:
            grid.require_clear(point, margin)
        except ValueError as exc:
            raise ValueError(f'waypoint {index}: {exc}') from exc
    return True
