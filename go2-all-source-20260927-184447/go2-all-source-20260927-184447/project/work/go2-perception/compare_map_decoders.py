"""Сравнение двух штатных декодеров одного сохранённого пакета карты.

Не соединяется с роботом. Проверяет согласованность поверхности mesh
и занятых ячеек, а не физическую точность карты или сырых измерений.
"""
import argparse
import hashlib
import importlib.util
import json
import sysconfig
import time
from pathlib import Path

import numpy as np
import lz4.block

def installed_decoder(name):
    # Декодер — самостоятельный файл. Не запускаем __init__ всего пакета:
    # тот открывает PortAudio даже при обработке записи без аудио/связи.
    path = Path(sysconfig.get_paths()['purelib']) / 'unitree_webrtc_connect/lidar' / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LidarDecoder


def boundary_match(quads, occupied):
    lower = quads.min(axis=1).astype(int)
    upper = quads.max(axis=1).astype(int)
    shape_xyz = np.array(occupied.shape[::-1])
    axis = np.argmin(upper - lower, axis=1)
    rows = np.arange(len(lower))
    before = lower.copy()
    before[rows, axis] -= 1

    def sample(xyz):
        inside = ((xyz >= 0) & (xyz < shape_xyz)).all(axis=1)
        result = np.zeros(len(xyz), bool)
        x, y, z = xyz[inside].T
        result[inside] = occupied[z, y, x]
        return result

    a, b = sample(before), sample(lower)
    boundary_count = sum(int(np.count_nonzero(np.diff(np.pad(occupied.astype(np.int8), 1), axis=i))) for i in range(3))
    return {'mesh_faces': len(quads), 'xor_boundary_faces': int(np.count_nonzero(a ^ b)),
            'fraction_supported': float(np.mean(a ^ b)),
            'both_cells_empty_faces': int(np.count_nonzero(~a & ~b)),
            'both_cells_occupied_faces': int(np.count_nonzero(a & b)),
            'expected_full_voxel_boundary_faces': boundary_count}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    binary = (args.capture / 'map_payload.bin').read_bytes()
    saved = json.loads((args.capture / 'map_payload.json').read_text())
    metadata = saved['metadata']
    width = np.asarray(metadata['width'], int)
    decoded = lz4.block.decompress(binary, uncompressed_size=metadata['src_size'])
    if width[0] != 128 or width[1] != 128 or len(decoded) * 8 != int(np.prod(width)):
        raise ValueError('Формат карты не соответствует проверяемому штатному декодеру 128×128')
    start = time.perf_counter()
    native = installed_decoder('lidar_decoder_native.py')().decode(binary, metadata)
    native_ms = (time.perf_counter() - start) * 1000
    mesh_decoder = installed_decoder('lidar_decoder_libvoxel.py')()
    start = time.perf_counter()
    mesh = mesh_decoder.decode(binary, metadata)
    mesh_ms = (time.perf_counter() - start) * 1000
    quads = mesh['positions'].reshape(-1, 4, 3)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output / 'same_packet_native.npz', **native)
    np.savez_compressed(args.output / 'same_packet_mesh.npz', **{k: v for k, v in mesh.items() if isinstance(v, np.ndarray)})
    cases = {}
    for order in ['big', 'little']:
        occupied = np.unpackbits(np.frombuffer(decoded, np.uint8), bitorder=order).reshape(tuple(width[::-1])).astype(bool)
        cases[order] = boundary_match(quads, occupied)
    result = {'payload_sha256': hashlib.sha256(binary).hexdigest(),
              'metadata': metadata, 'mesh_point_count': int(mesh['point_count']),
              'native_point_count': len(native['points']), 'mesh_face_count': int(mesh['face_count']),
              'decode_ms_single_call': {'native': native_ms, 'mesh_excludes_initialization': mesh_ms},
              'boundary_comparison': cases,
              'native_uses_bitorder': 'big',
              'physical_accuracy_verified': False,
              'note': 'Это сравнение двух представлений одной обработанной карты; согласие не подтверждает положение реальных препятствий.'}
    (args.output / 'decoder-comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
