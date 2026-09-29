"""Проверка восстановления видео на Pi без команд движения."""
import asyncio
from pathlib import Path
import sys

TEAM=Path('/home/ubuntu/ai-robot/team_wolf_setup')
sys.path[:0]=[str(TEAM/'autonomy-current'),str(TEAM/'perception_deps'),'/home/ubuntu/ai-robot']
import go2
import training_entry

original=go2.connect


async def connect(*args,**kwargs):
    conn=await original(*args,**kwargs)
    async def interrupt_video_once():
        await asyncio.sleep(7.)
        conn.video.switchVideoChannel(False)
        print('TEST: видеопоток отключён для проверки восстановления',flush=True)
    asyncio.create_task(interrupt_video_once())
    return conn


if __name__=='__main__':
    go2.connect=connect
    sys.argv=[sys.argv[0],'--observe']
    raise SystemExit(training_entry.main())
