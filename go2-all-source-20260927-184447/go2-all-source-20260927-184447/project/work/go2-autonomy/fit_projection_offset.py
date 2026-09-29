"""Оценка одного смещения на обучающем сеансе, проверка на остальных.

Не калибрует автоматически боевой профиль. Порог назначения не изменяется.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from wolf_go2.association import gated_assignment
from wolf_go2.projection_match import curve_cost


def score(frame, offset, focal_scale=1.):
    features=frame['projection_features']
    scale=features['image_size'][0]/1280
    centers=features['centers_px']
    curves=features['curves']
    if 'boxes_xyxy' not in features:
        raise ValueError('Старый набор проекций без областей стоек: пересоздайте replay_camera_series')
    costs=np.full((len(centers),len(curves)),1e6)
    for i,center in enumerate(centers):
        for j,curve in enumerate(curves):
            pixels=np.asarray(curve['pixels']).copy()
            pixels[:,0]=(pixels[:,0]-640*scale)*focal_scale+640*scale+offset*scale
            costs[i,j]=curve_cost(pixels,features['boxes_xyxy'][i])[0]
    matches=gated_assignment(costs)
    return sum(m['accepted'] for m in matches)


def session_validation(frames, training_session, offset, focal_scale):
    """Не позволяем общей сумме скрыть ухудшение отдельного сеанса."""
    sessions={}
    for frame in frames:
        name=Path(frame['frame']).parent.name
        row=sessions.setdefault(name,{'frames':0,'visible':0,'baseline':0,'corrected':0})
        row['frames']+=1
        if frame['camera_candidates']<3:
            continue
        row['visible']+=1
        row['baseline']+=int(score(frame,0.)>=3)
        row['corrected']+=int(score(frame,offset,focal_scale)>=3)
    regressions=[name for name,row in sessions.items()
                 if name!=training_session and row['corrected']<row['baseline']]
    return {'per_session':sessions,'held_out_regressions':regressions,
            'candidate_accepted':False,
            'decision':('rejected_session_regression' if regressions else
                        'requires_independent_pose_validation')}


def evaluate(frames,training_session):
    train=[f for f in frames if Path(f['frame']).parent.name==training_session]
    validation=[f for f in frames if Path(f['frame']).parent.name!=training_session]
    # Один вклад на кадр, а не завышенный вес кадра с большим числом стоек.
    samples=[]
    for frame in train:
        values=[m['observed_minus_projected_xy'][0]*1280/frame['projection_features']['image_size'][0]
                for m in frame['matches'] if m['accepted']]
        if values:samples.append(float(np.median(values)))
    if len(samples)<5:
        raise ValueError('Недостаточно кадров с исходными соответствиями для оценки')
    offset=float(np.median(samples))
    if abs(offset)>40:
        raise ValueError('Смещение слишком велико для проверяемой модели')
    groups={}
    for name,rows in [('training',train),('held_out',validation)]:
        visible=[f for f in rows if f['camera_candidates']>=3]
        groups[name]={'frames':len(rows),'with_three_camera_candidates':len(visible),
                      'baseline_three_matches':sum(score(f,0.)>=3 for f in visible),
                      'corrected_three_matches':sum(score(f,offset)>=3 for f in visible)}
    return {'training_session':training_session,'candidate_offset_px_at_1280':offset,
            'training_frame_offsets':samples,'evaluation':groups,
            'calibration_verified':False,'profile_modified':False,'motion_authorized':False,
            'limitation':'Сеансы разделены, но независимость положений робота и физическая калибровка не подтверждены.'}


def evaluate_affine(frames,training_session):
    train=[f for f in frames if Path(f['frame']).parent.name==training_session]
    validation=[f for f in frames if Path(f['frame']).parent.name!=training_session]
    pairs=[]
    for f in train:
        scale=1280/f['projection_features']['image_size'][0]
        selected=[m for m in f['matches'] if m['accepted']]
        for m in selected:
            observed=f['projection_features']['centers_px'][m['image_index']][0]*scale
            predicted=observed-m['observed_minus_projected_xy'][0]*scale
            pairs.append((predicted-640,observed-640,1/np.sqrt(len(selected))))
    if len(pairs)<12:
        raise ValueError('Недостаточно исходных пар для оценки масштаба')
    samples=np.asarray(pairs)
    if np.ptp(samples[:,0])<150:
        raise ValueError('Недостаточный диапазон положений в изображении')
    fit=least_squares(lambda p:(samples[:,0]*p[0]+p[1]-samples[:,1])*samples[:,2],
                      [1.,0.],bounds=([.8,-50],[1.2,50]),loss='soft_l1',f_scale=5.)
    focal,offset=map(float,fit.x)
    results={}
    for label,rows in [('training',train),('held_out',validation)]:
        visible=[f for f in rows if f['camera_candidates']>=3]
        results[label]={'frames':len(rows),'with_three_camera_candidates':len(visible),
                        'baseline_three_matches':sum(score(f,0.)>=3 for f in visible),
                        'corrected_three_matches':sum(score(f,offset,focal)>=3 for f in visible)}
    return {'training_session':training_session,'candidate_focal_scale_x':focal,
            'candidate_offset_px_at_1280':offset,'evaluation':results,
            **session_validation(frames,training_session,offset,focal),
            'on_parameter_boundary':bool(focal<.801 or focal>1.199 or abs(offset)>49.9),
            'calibration_verified':False,'profile_modified':False,'motion_authorized':False,
            'limitation':'Уточнена только горизонтальная проекция. Физическая калибровка, вертикальная геометрия и время не подтверждены.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('series',type=Path)
    p.add_argument('--training-session',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--affine',action='store_true')
    args=p.parse_args()
    frames=json.loads(args.series.read_text(encoding='utf-8'))['frames']
    result=(evaluate_affine if args.affine else evaluate)([f for f in frames if 'error' not in f],args.training_session)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
