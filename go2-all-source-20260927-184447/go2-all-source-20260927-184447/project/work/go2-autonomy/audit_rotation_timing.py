"""Сравнение движения изображения и yaw в записи короткого поворота.

Оценивает относительное совмещение сигналов; не подтверждает задержку
съёмки, параметры камеры или физический успех автономного маршрута.
"""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np


def _shift_from_pairs(reference,a,b,method):
    if len(a)<40:return None
    h,mask=cv2.findHomography(a,b,cv2.RANSAC,2.)
    if h is None or mask is None or mask.sum()<40 or mask.mean()<.5:return None
    center=np.array([reference.shape[1]/2,reference.shape[0]/2,1.])
    projected=h@center
    if abs(projected[2])<1e-8:return None
    shift=projected[:2]/projected[2]-center[:2]
    return {'shift_x_px':float(shift[0]),'shift_y_px':float(shift[1]),'inliers':int(mask.sum()),'method':method}


def feature_shift(reference,image):
    detector=cv2.ORB_create(nfeatures=2000)
    ka,da=detector.detectAndCompute(reference,None)
    kb,db=detector.detectAndCompute(image,None)
    if da is None or db is None:return None
    pairs=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True).match(da,db)
    pairs=[m for m in pairs if m.distance<=40]
    a=np.asarray([ka[m.queryIdx].pt for m in pairs],dtype=np.float32)
    b=np.asarray([kb[m.trainIdx].pt for m in pairs],dtype=np.float32)
    return _shift_from_pairs(reference,a,b,'descriptor_homography')


def image_shift(reference,image,features):
    tracked,status,_=cv2.calcOpticalFlowPyrLK(reference,image,features,None,winSize=(31,31),maxLevel=4)
    if tracked is None:return None
    back,reverse,_=cv2.calcOpticalFlowPyrLK(image,reference,tracked,None,winSize=(31,31),maxLevel=4)
    if back is None:return None
    valid=(status.ravel()>0)&(reverse.ravel()>0)&(np.linalg.norm(back-features,axis=2).ravel()<1.)
    a=features[valid].reshape(-1,2);b=tracked[valid].reshape(-1,2)
    return _shift_from_pairs(reference,a,b,'optical_flow_homography')


def fit_alignment(frames, poses):
    """Не выдаёт численную калибровку при слишком слабом повороте."""
    poses=np.asarray(poses,dtype=float).copy()
    if len(frames)<20 or len(poses)<20:
        return {'status':'insufficient_observations','relative_alignment_candidates':[]}
    if poses.ndim!=2 or poses.shape[1]!=2 or not np.isfinite(poses).all() or np.any(np.diff(poses[:,0])<=0):
        raise ValueError('Время одометрии должно строго возрастать; значения должны быть конечными')
    poses[:,1]=np.unwrap(poses[:,1])
    times=np.array([r['t'] for r in frames]);flow=np.array([r['shift_x_px'] for r in frames])
    candidates=[]
    for lag in np.linspace(-1.,1.,41):
        inside=(times-lag>=poses[0,0])&(times-lag<=poses[-1,0])
        if inside.sum()<20:continue
        yaw=np.interp(times[inside]-lag,poses[:,0],poses[:,1])
        if np.ptp(yaw)<math.radians(1.):continue
        design=np.column_stack((yaw-yaw[0],np.ones(len(yaw))))
        fit=np.linalg.lstsq(design,flow[inside],rcond=None)[0]
        residual=flow[inside]-design@fit
        candidates.append({'relative_shift_s':float(lag),'pixel_per_yaw_radian':float(fit[0]),
                           'rmse_px':float(np.sqrt(np.mean(residual**2))),'samples':int(inside.sum())})
    return {'status':'relative_estimate_only' if candidates else 'insufficient_rotation',
            'odometry_yaw_span_deg':float(np.degrees(np.ptp(poses[:,1]))),
            'minimum_yaw_span_deg':1.,
            'relative_alignment_candidates':sorted(candidates,key=lambda x:x['rmse_px'])}


def audit(folder):
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    records=[r for r in summary['camera_records'] if r['phase'] in ('before','move','after')]
    baseline=next(r for r in records if r['phase']=='before')
    reference=cv2.imread(str(folder/baseline['file']),0)
    if reference is None:raise ValueError('Нет исходного кадра')
    features=cv2.goodFeaturesToTrack(reference,600,.02,10)
    if features is None or len(features)<40:raise ValueError('Недостаточно деталей изображения')
    frames=[]
    for record in records:
        image=cv2.imread(str(folder/record['file']),0)
        if image is None or image.shape!=reference.shape:continue
        shift=image_shift(reference,image,features)
        if shift is None:shift=feature_shift(reference,image)
        if shift:frames.append({'t':record['elapsed_s'],'file':record['file'],'phase':record['phase'],**shift})
    poses=[]
    with (folder/'samples.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']!='ROBOTODOM':continue
            q=row['message']['data']['pose']['orientation']
            yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
            poses.append((row['elapsed_s'],yaw))
    return {'frames':frames,**fit_alignment(frames,poses),
            'source':'received_camera_and_received_odometry',
            'absolute_latency_verified':False,'calibration_verified':False,'correction_applied':False,
            'limitation':'Один короткий поворот не разделяет задержки источников, погрешность одометрии и движение корпуса.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path)
    args=p.parse_args()
    result=audit(args.folder)
    (args.folder/'rotation-timing-analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'status':result['status'],'frames':len(result['frames']),'best_candidates':result['relative_alignment_candidates'][:5]},indent=2))
