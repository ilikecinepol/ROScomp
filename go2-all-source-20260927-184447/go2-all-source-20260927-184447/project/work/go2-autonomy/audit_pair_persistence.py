"""Повторная проверка устойчивости соответствий на сохранённой серии."""
import argparse
import json
from pathlib import Path
from wolf_go2.crossmodal_tracks import PairPersistence


def evaluate(frames):
    tracker=PairPersistence()
    results=[]
    for frame in sorted(frames,key=lambda f:f['frame']):
        if 'error' in frame:continue
        path=Path(frame['frame'])
        metadata=json.loads(path.read_text(encoding='utf-8-sig'))
        if metadata.get('camera_received_at') is None:continue
        context=(str(path.parent),metadata.get('pi_boot_id'),metadata.get('frame_epoch'))
        geometry={p['lidar_index']:p for p in frame['lidar_geometry']}
        accepted=[geometry[m['lidar_index']]['center_xy'] for m in frame['matches'] if m['accepted']]
        tracks=tracker.update(context,metadata['received_at'],metadata['camera_received_at'],accepted)
        results.append({'frame':str(path),'tracks':tracks,
                        'persistent_pairs':sum(t['consecutive_frames']>=3 for t in tracks)})
    return {'frames':results,'summary':{'processed':len(results),
            'with_three_persistent_pairs':sum(f['persistent_pairs']>=3 for f in results),
            'maximum_persistent_pairs':max((f['persistent_pairs'] for f in results),default=0),
            'calibration_verified':False,'motion_authorized':False,
            'limitation':'Повторяемость пары не доказывает правильность калибровки, возраст измерения или полноту ряда.'}}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('series',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=evaluate(json.loads(args.series.read_text(encoding='utf-8'))['frames'])
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result['summary'],ensure_ascii=False))
