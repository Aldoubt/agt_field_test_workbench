"""Real-library synthetic smoke test; not a replay of the field failure.

Usage: python tests/bbs_geometry_smoke.py --assets-builder BIN
    --candidate-builder BIN --localizer BIN
All inputs and BBS assets are generated in a temporary directory.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile


def write_pcd(path, points):
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA ascii\n')
    path.write_text(header + ''.join(f'{x:.8f} {y:.8f} {z:.8f}\n' for x, y, z in points))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('assets-builder', 'candidate-builder', 'localizer'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='agt_bbs_geometry_') as directory:
        root = Path(directory)
        patches = root / 'patches'
        patches.mkdir()
        # Three non-parallel surfaces and unequal tree-like cylinders constrain
        # the six-dimensional registration; no robot or ROS graph is involved.
        points = [(x * .35, y * .35, 0.) for x in range(-20, 26) for y in range(-14, 18)]
        points += [(-7., y * .35, z * .35) for y in range(-14, 18) for z in range(1, 14)]
        points += [(x * .35, 5.95, z * .35) for x in range(-20, 12) for z in range(1, 9)]
        for cx, cy, height in ((2., 1., 13), (4., -2., 8)):
            points += [(cx + .4 * math.cos(a * .3), cy + .4 * math.sin(a * .3), z * .25)
                       for a in range(21) for z in range(height)]
        yaw = .2
        c, s = math.cos(yaw), math.sin(yaw)
        query = [(c * (x - 2.) + s * (y + 1.), -s * (x - 2.) + c * (y + 1.), z - .3)
                 for x, y, z in points]
        write_pcd(root / 'map.pcd', points)
        write_pcd(patches / '000000.pcd', query)
        (root / 'poses.txt').write_text(
            f'000000.pcd 2 -1 .3 {math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}\n')
        assets = root / 'assets'
        subprocess.run([args.assets_builder, '--map', str(root / 'map.pcd'), '--output', str(assets),
                        '--map-leaf', '.35'], check=True, capture_output=True, text=True, timeout=30)
        subprocess.run([args.candidate_builder, '--map-dir', str(root), '--output', str(assets)],
                       check=True, capture_output=True, text=True, timeout=30)
        proc = subprocess.run([args.localizer, '--map', str(root / 'map.pcd'),
                               '--scan', str(patches / '000000.pcd'), '--assets-dir', str(assets),
                               '--timeout', '18'], capture_output=True, text=True, timeout=22)
        result = json.loads(proc.stdout.strip().splitlines()[-1])
        assert proc.returncode == 0 and result['success'], (proc.stdout, proc.stderr)
        translation_error = math.sqrt((result['x'] - 2.) ** 2 + (result['y'] + 1.) ** 2 +
                                      (result['z'] - .3) ** 2)
        q = [result[k] for k in ('qx', 'qy', 'qz', 'qw')]
        estimate_yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                  1 - 2 * (q[1] ** 2 + q[2] ** 2))
        yaw_error = abs(math.atan2(math.sin(estimate_yaw - yaw), math.cos(estimate_yaw - yaw)))
        assert translation_error < .15, result
        assert yaw_error < .03, result
        assert result['overlap'] > .5, result
        print(json.dumps({'test': 'asymmetric_synthetic_real_libraries',
                          'translation_error_m': translation_error, 'yaw_error_rad': yaw_error,
                          'result': result}, indent=2))


if __name__ == '__main__':
    main()
