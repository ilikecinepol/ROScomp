"""Полосатые красно-белые стойки в изображении. Расстояние не определяется."""
import cv2
import numpy as np


def detect_striped_poles(image):
    if not isinstance(image,np.ndarray) or image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
        raise ValueError('Нужно изображение BGR uint8')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    red=((hsv[:,:,0]<=12)|(hsv[:,:,0]>=165))&(hsv[:,:,1]>=90)&(hsv[:,:,2]>=55)
    white=(hsv[:,:,1]<80)&(hsv[:,:,2]>100)
    count,labels,stats,centers=cv2.connectedComponentsWithStats(red.astype(np.uint8),8)
    parts=[]
    minimum=max(20,image.shape[0]*image.shape[1]*.00002)
    for i in range(1,count):
        x,y,w,h,area=stats[i]
        if area<minimum or w<3 or h<4 or w>image.shape[1]*.18:continue
        parts.append((int(x),int(y),int(w),int(h)))
    links={i:set() for i in range(len(parts))}
    bands={}
    for i,(x,y,w,h) in enumerate(parts):
        for j,(xx,yy,ww,hh) in enumerate(parts):
            if yy<y+h:continue
            gap=yy-y-h
            if not 2<=gap<=max(w,ww)*3 or max(w,ww)>min(w,ww)*3:continue
            lo=max(x,xx);hi=min(x+w,xx+ww)
            if hi-lo<min(w,ww)*.45:continue
            # Проверяем светлую перемычку, а не только красный цвет.
            margin=max(1,(hi-lo)//5)
            if hi-lo<=2*margin:continue
            ratio=float(white[y+h:yy,lo+margin:hi-margin].mean())
            if ratio<.5:
                # Белая полоса в тени может быть темнее фиксированного порога.
                # Требуем нейтральный цвет и контраст с ОБЕИМИ красными полосами.
                gap_hsv=hsv[y+h:yy,lo+margin:hi-margin]
                neutral=(gap_hsv[:,:,1]<80)&(gap_hsv[:,:,2]>55)
                if float(neutral.mean())<.5:continue
                upper=hsv[y:y+h,x:x+w,2][red[y:y+h,x:x+w]]
                lower=hsv[yy:yy+hh,xx:xx+ww,2][red[yy:yy+hh,xx:xx+ww]]
                if not len(upper) or not len(lower):continue
                brightness=float(np.median(gap_hsv[:,:,2][neutral]))
                if brightness<max(float(np.median(upper)),float(np.median(lower)))+8:continue
                ratio=float(neutral.mean())
            links[i].add(j);links[j].add(i);bands[i,j]=ratio
    result=[];seen=set()
    for first in links:
        if first in seen:continue
        todo=[first];group=set()
        while todo:
            i=todo.pop()
            if i in group:continue
            group.add(i);todo.extend(links[i]-group)
        seen.update(group)
        # Двух пятен недостаточно: вывеска и обрезанное основание дают ложную пару.
        if len(group)<3:continue
        boxes=[parts[i] for i in group]
        x=min(b[0] for b in boxes);y=min(b[1] for b in boxes)
        right=max(b[0]+b[2] for b in boxes);bottom=max(b[1]+b[3] for b in boxes)
        if bottom-y<2.5*(right-x):continue
        result.append({'bbox_xyxy':[x,y,right,bottom],'red_sections':len(group),
                       'white_gaps':sum(i in group and j in group for i,j in bands),
                       'touches_edge':x<=1 or right>=image.shape[1]-1 or y<=1 or bottom>=image.shape[0]-1,
                       'distance_verified':False,'route_verified':False})
    return sorted(result,key=lambda item:item['bbox_xyxy'][0])
