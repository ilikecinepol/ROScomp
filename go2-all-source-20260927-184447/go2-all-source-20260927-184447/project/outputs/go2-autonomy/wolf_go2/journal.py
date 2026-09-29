"""Ограниченная очередь записи не задерживает цикл остановки."""
import json
import queue
import threading
import time
import math
from pathlib import Path

def jsonable(value):
    if isinstance(value,dict): return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [jsonable(v) for v in value]
    if getattr(value,'ndim',None)==0 and hasattr(value,'item'): return jsonable(value.item())
    if hasattr(value,'shape'): return {'shape':list(value.shape),'dtype':str(value.dtype)}
    if hasattr(value,'item'): return jsonable(value.item())
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,bytes): return {'bytes':len(value)}
    return value

class Journal:
    def __init__(self,path):
        self.path=Path(path)
        self.path.mkdir(parents=True,exist_ok=False)
        self.queue=queue.Queue(maxsize=4000)
        self.failed=threading.Event()
        self.dropped=0
        self.media_bytes=0
        self.media_lock=threading.Lock()
        self.thread=threading.Thread(target=self._worker,daemon=True)
        self.thread.start()

    def event(self,event,**data):
        self.put('events.jsonl',{'event':event,'t':time.monotonic(),**data})

    def put(self,filename,data):
        if Path(filename).name != filename:
            raise ValueError('Журнал принимает только имя файла внутри сессии')
        size=int(getattr(data,'nbytes',0)) if filename.endswith(('.jpg','.npz')) else 0
        with self.media_lock:
            if self.media_bytes+size>32*1024*1024:
                self.dropped+=1
                self.failed.set()
                return
            self.media_bytes+=size
        try:
            self.queue.put_nowait((filename,data,size))
        except queue.Full:
            with self.media_lock: self.media_bytes-=size
            self.dropped+=1
            self.failed.set()

    def _worker(self):
        files={}
        try:
            while True:
                job=self.queue.get()
                try:
                    if job is None: break
                    name,data,size=job
                    if name.endswith('.jpg'):
                        import cv2
                        if not cv2.imwrite(str(self.path/name),data): raise OSError('JPEG write failed')
                    elif name.endswith('.npz'):
                        import numpy as np
                        np.savez_compressed(self.path/name,points=data)
                    else:
                        f=files.setdefault(name,None)
                        if f is None:
                            files[name]=f=(self.path/name).open('a',encoding='utf-8')
                        f.write(json.dumps(jsonable(data),ensure_ascii=False,allow_nan=False)+'\n')
                        f.flush()
                finally:
                    if job is not None:
                        with self.media_lock: self.media_bytes-=job[2]
                    self.queue.task_done()
        except Exception:
            self.failed.set()
        finally:
            for f in files.values():
                if f: f.close()

    def close(self):
        if self.thread.is_alive():
            try: self.queue.put(None,timeout=1)
            except queue.Full: self.failed.set()
            self.thread.join(timeout=10)
            if self.thread.is_alive(): self.failed.set()
