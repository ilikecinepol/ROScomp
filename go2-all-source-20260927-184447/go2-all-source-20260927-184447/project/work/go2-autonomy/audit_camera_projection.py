"""Проверка гипотезы калибровки SDK на синхронно сохранённых кадрах.

Параметры другого экземпляра — отправная точка, а не готовая калибровка.
Этот инструмент не меняет профиль и не разрешает движение.
"""
import argparse
import json
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from wolf_go2.association import gated_assignment
from wolf_go2.projection_match import curve_cost

from wolf_go2.models import Pose
from wolf_go2.scene_audit import inspect_scene
from wolf_go2.striped_poles import detect_striped_poles


@lru_cache(maxsize=4)
def recorded_orientations(folder):
    source=folder/'samples.jsonl'
    result={}
    if source.is_file():
        with source.open(encoding='utf-8-sig') as stream:
            for line in stream:
                row=json.loads(line)
                if row.get('topic')=='ROBOTODOM':
                    data=row.get('message',{}).get('data',{})
                    orientation=data.get('pose',{}).get('orientation')
                    if orientation is not None:
                        result[row['t']]=orientation
    return result


def audit(path, camera_offset=(.33,0.,.04), camera_yaw_deg=0., camera_focal_scale=1., distortion_coefficients=None):
    camera_offset=np.asarray(camera_offset,dtype=float)
    if camera_offset.shape!=(3,) or not np.isfinite(camera_offset).all() or not np.isfinite(camera_yaw_deg):
        raise ValueError('Некорректная гипотеза положения камеры')
    metadata=json.loads(path.read_text(encoding='utf-8-sig'))
    if not metadata.get('camera_file') or not metadata.get('pose'):
        raise ValueError('Нет парных изображения и позы')
    image=cv2.imread(str(path.parent/metadata['camera_file']))
    if image is None:
        raise ValueError('Не прочитано парное изображение')
    with np.load(path.with_suffix('.npz'),allow_pickle=False) as npz:
        points=npz['points']
    pose=Pose(**metadata['pose'])
    scene=inspect_scene(points,pose,{'resolution':metadata['resolution'],'frame_id':metadata['point_frame']})
    detections=detect_striped_poles(image)
    h,w=image.shape[:2]
    camera=np.array([[864.39938,0,639.19798],[0,863.73849,373.28118],[0,0,1.]])
    if not np.isfinite(camera_focal_scale) or camera_focal_scale<=0:
        raise ValueError('Некорректный масштаб фокусного расстояния')
    camera[0,0]*=camera_focal_scale;camera[1,1]*=camera_focal_scale
    camera[0]*=w/1280;camera[1]*=h/720
    distortion=np.array([-.354630,.102054,-.001614,-.001249,0.] if distortion_coefficients is None else distortion_coefficients,dtype=float)
    if distortion.shape!=(5,) or not np.isfinite(distortion).all():
        raise ValueError('Нужны пять конечных коэффициентов дисторсии')
    orientation=metadata.get('pose_orientation_xyzw')
    if orientation is None:
        orientation=recorded_orientations(path.parent).get(metadata['pose_received_at'])
    sidecar=path.with_suffix('.orientation.json')
    if orientation is None and sidecar.is_file():
        extra=json.loads(sidecar.read_text(encoding='utf-8'))
        if abs(extra['pose_received_at']-metadata['pose_received_at'])>1e-6:
            raise ValueError('Ориентация не относится к сохранённой паре')
        orientation=extra['orientation']
    if orientation:
        quat=np.array([orientation[k] for k in 'xyzw'],dtype=float)
        if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.02:
            raise ValueError('Некорректный кватернион позы')
        rotation=Rotation.from_quat(quat).as_matrix()
    else:
        rotation=Rotation.from_euler('z',pose.yaw).as_matrix()
    projected=[]
    object_geometry=[]
    floor=scene.get('floor_coefficients')
    if floor is None:
        raise ValueError('Нет оценки плоскости пола')
    for i,pole in enumerate(scene.get('slalom_pole_candidates',[])):
        if pole.get('row_eligible') is False:
            continue
        x,y=pole['center_xy']
        base=float(np.dot(floor,[x,y,1]))
        heights=np.linspace(.1,1.3,121)+base
        world=np.column_stack((np.full(len(heights),x),np.full(len(heights),y),heights))
        body=(world-[pose.x,pose.y,pose.z+.07])@rotation-camera_offset
        body=body@Rotation.from_euler('z',camera_yaw_deg,degrees=True).as_matrix()
        optical=np.column_stack((-body[:,1],-body[:,2],body[:,0]))
        geometry={'lidar_index':i,'center_xy':[x,y],
                  'partially_observed':bool(pole.get('partially_observed')),
                  'depth_range_m':[float(optical[:,2].min()),float(optical[:,2].max())],
                  'in_image_samples':0,'projected_samples':0}
        object_geometry.append(geometry)
        optical=optical[optical[:,2]>.1]
        if not len(optical):
            continue
        pixels=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),camera,distortion)[0].reshape(-1,2)
        geometry['projected_samples']=len(pixels)
        geometry['in_image_samples']=int(((pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)).sum())
        projected.append((i,pixels))
    costs=np.full((len(detections),len(projected)),1e6)
    centers=[]
    residuals={}
    for i,detection in enumerate(detections):
        x0,y0,x1,y1=detection['bbox_xyxy']
        center=np.array([(x0+x1)/2,(y0+y1)/2])
        centers.append(center.tolist())
        for j,(_,pixels) in enumerate(projected):
            costs[i,j],residuals[i,j]=curve_cost(pixels,detection['bbox_xyxy'])
    matches=gated_assignment(costs)
    for match in matches:
        match['observed_minus_projected_xy']=residuals[match['image_index'],match['projected_index']]
        match['lidar_index']=projected[match.pop('projected_index')][0]
    return {'frame':str(path),'matches':matches,'camera_candidates':len(detections),
            'camera_detections':detections,'lidar_geometry':object_geometry,
            'row_hypotheses':scene.get('slalom_row_candidates',[]),
            'pose':metadata['pose'],
            'camera_extrinsic_hypothesis':{'offset_xyz':camera_offset.tolist(),'yaw_deg':float(camera_yaw_deg),'verified':False},
            'camera_intrinsic_hypothesis':{'matrix':camera.tolist(),'distortion':distortion.tolist(),'verified':False},
            'accepted_matches':sum(m['accepted'] for m in matches),
            'projection_features':{'image_size':[w,h],'centers_px':centers,
                'boxes_xyxy':[d['bbox_xyxy'] for d in detections],
                'curves':[{'lidar_index':i,'pixels':pixels.tolist()} for i,pixels in projected]},
            'calibration_source':'go2_ros2_sdk b440609591a249e7bdd4bbc88e056a3660575447',
            'assumptions':['Калибровка другого экземпляра',
                           'Ориентация из ROBOTODOM' if orientation else 'Нулевые roll/pitch: старый файл не содержит кватернион',
                           'Согласование по времени приёма, не съёмки'],
            'calibration_verified':False,'profile_modified':False,'motion_authorized':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    reports=[audit(path) for path in args.frames]
    args.output.write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(reports,ensure_ascii=False,indent=2))
