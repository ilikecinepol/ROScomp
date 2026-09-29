"""Сравнение двух установленных декодеров на одном сохранённом пакете."""
import itertools
import json
from pathlib import Path
import runpy
import sys

import numpy as np


def main():
    folder = Path(sys.argv[1])
    sdk = Path('/home/ubuntu/ai-robot/venv/lib/python3.14/site-packages/unitree_webrtc_connect/lidar')
    metadata = json.loads((folder/'raw-lidar.json').read_text())
    payload = (folder/'raw-lidar.bin').read_bytes()
    native = runpy.run_path(str(sdk/'lidar_decoder_native.py'))['LidarDecoder']()
    wasm = runpy.run_path(str(sdk/'lidar_decoder_libvoxel.py'))['LidarDecoder']()
    points = native.decode(payload, metadata)['points']
    mesh = wasm.decode(payload, metadata)
    resolution = float(metadata['resolution'])
    cells = set(map(tuple, np.rint((points-np.array(metadata['origin']))/resolution).astype(int)))
    vertices = np.unique(mesh['positions'].reshape(-1, 3), axis=0).astype(int)
    offsets = list(itertools.product((0, 1), repeat=3))
    unexplained = [v.tolist() for v in vertices
                   if not any(tuple(v-np.array(d)) in cells for d in offsets)]
    result = {'native_cells': len(cells), 'mesh_vertices': len(vertices),
              'mesh_vertices_without_native_neighbour': len(unexplained),
              'unexplained_examples': unexplained[:20],
              'resolution': resolution, 'origin': metadata['origin'],
              'note': 'Согласие декодеров не проверяет привязку к корпусу или свежесть измерения.',
              'motion_authorized': False}
    (folder/'decoder-comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
