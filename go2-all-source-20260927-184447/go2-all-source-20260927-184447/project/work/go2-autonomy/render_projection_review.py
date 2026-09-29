"""Наглядная проверка геометрии одного записанного кадра, без робота."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from audit_camera_projection import audit


def render(path, output):
    report=audit(path)
    metadata=json.loads(path.read_text(encoding='utf-8-sig'))
    image=cv2.imread(str(path.parent/metadata['camera_file']))
    h,w=image.shape[:2]
    panel=np.full((h,w,3),245,np.uint8)
    with np.load(path.with_suffix('.npz'),allow_pickle=False) as data:
        cloud=data['points']
    lo=cloud[:,:2].min(0)-.2
    hi=cloud[:,:2].max(0)+.2
    scale=min((w-80)/(hi[0]-lo[0]),(h-100)/(hi[1]-lo[1]))
    def xy(point):
        value=(np.asarray(point)-lo)*scale
        return int(40+value[0]),int(h-40-value[1])
    for point in cloud[::max(1,len(cloud)//18000)]:
        cv2.circle(panel,xy(point[:2]),1,(205,205,205),-1)
    colors=[(0,150,0),(200,90,0),(180,0,180),(0,130,220),(140,140,0)]
    indices={i for row in report['row_hypotheses'] for i in row['pole_indices']}
    curves={c['lidar_index']:np.asarray(c['pixels']) for c in report['projection_features']['curves']}
    for index in sorted(indices):
        geometry=next((g for g in report['lidar_geometry'] if g['lidar_index']==index),None)
        if geometry is None:continue
        color=colors[index%len(colors)]
        point=xy(geometry['center_xy'])
        cv2.circle(panel,point,7,color,-1)
        label=f'L{index}'+(' partial' if geometry['partially_observed'] else '')
        cv2.putText(panel,label,(point[0]+9,point[1]),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
        pixels=curves.get(index,np.empty((0,2)))
        visible=pixels[(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)]
        for pixel in visible:
            cv2.circle(image,tuple(np.rint(pixel).astype(int)),2,color,-1)
        if len(visible):
            pixel=tuple(np.rint(visible[len(visible)//2]).astype(int))
            cv2.putText(image,f'L{index}',pixel,cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
    for index,detection in enumerate(report['camera_detections']):
        x0,y0,x1,y1=detection['bbox_xyxy']
        cv2.rectangle(image,(x0,y0),(x1,y1),(255,255,0),2)
        cv2.putText(image,f'C{index}',(x0,max(20,y0)),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,0),2)
    pose=report['pose'];point=xy([pose['x'],pose['y']])
    end=xy([pose['x']+.5*np.cos(pose['yaw']),pose['y']+.5*np.sin(pose['yaw'])])
    cv2.arrowedLine(panel,point,end,(0,0,0),3)
    cv2.putText(panel,'Robot pose / candidate objects (not verified)',(25,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),1)
    if not cv2.imwrite(str(output),np.hstack((image,panel))):
        raise OSError('Не удалось сохранить разбор')
    report['review_image']=str(output)
    output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frame',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=render(args.frame,args.output)
    print(json.dumps({'image':str(args.output),'camera_candidates':result['camera_candidates'],
                      'accepted_matches':result['accepted_matches']},ensure_ascii=False))
