"""Проверка семейства поправок без изменения калибровки и движения."""
import argparse
import json
from pathlib import Path
import numpy as np
from fit_projection_offset import score


def scan(frames):
    groups={}
    for frame in frames:
        if 'error' not in frame and frame['camera_candidates']>=3:
            groups.setdefault(str(Path(frame['frame']).parent),[]).append(frame)
    candidates=[]
    for focal in np.linspace(.8,1.2,9):
        for offset in range(-40,41,10):
            sessions={}
            for name,rows in groups.items():
                values=[score(f,float(offset),float(focal)) for f in rows]
                sessions[name]={'frames':len(rows),'three_matches':sum(v>=3 for v in values),
                                'total_matches':sum(values)}
            candidates.append({'focal_scale':float(focal),'offset_px':offset,'sessions':sessions})
    best={}
    for name in groups:
        maximum=max(c['sessions'][name]['three_matches'] for c in candidates)
        ties=[{'focal_scale':c['focal_scale'],'offset_px':c['offset_px']}
              for c in candidates if c['sessions'][name]['three_matches']==maximum]
        best[name]={'frames':len(groups[name]),'best_three_matches':maximum,
                    'equally_scoring_models':ties}
    return {'per_session_best':best,'candidates':candidates,
            'calibration_verified':False,'profile_modified':False,'motion_authorized':False,
            'limitation':'Поиск использует все сеансы для диагностики; это не независимая проверка выбранной калибровки.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('series',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=scan(json.loads(args.series.read_text(encoding='utf-8'))['frames'])
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({Path(k).name:{'frames':v['frames'],'best_three_matches':v['best_three_matches'],
                      'ties':len(v['equally_scoring_models'])} for k,v in result['per_session_best'].items()}))
