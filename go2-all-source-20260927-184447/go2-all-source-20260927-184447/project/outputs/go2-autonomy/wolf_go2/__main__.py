"""Команды офлайн-проверки и явного бортового запуска; по умолчанию справка."""
import argparse
import asyncio
from dataclasses import asdict
import json
import math
from pathlib import Path

def positive(value):
    number=float(value)
    if not math.isfinite(number) or not 0<number<=900:
        raise argparse.ArgumentTypeError('Длительность должна быть в пределах 0–900 секунд')
    return number

def write_new(path,value):
    target=Path(path)
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('x',encoding='utf-8') as out:
        json.dump(value,out,ensure_ascii=False,indent=2,allow_nan=False)
    return str(target.resolve())

def parser():
    p=argparse.ArgumentParser(description='WOLF Go2: общий контроллер и проверка данных. По умолчанию соединений нет.')
    modes=p.add_subparsers(dest='mode')
    demo=modes.add_parser('demo',help='Замкнутая кинематическая проверка, без физики и робота')
    demo.add_argument('--seed',type=int,default=0)
    demo.add_argument('--output',required=True,help='Новый JSON-файл результата')
    replay=modes.add_parser('replay',help='Разбор локальной записи, без команд роботу')
    replay.add_argument('--capture',required=True)
    replay.add_argument('--output',required=True,help='Новый JSON-файл результата')
    profile=modes.add_parser('profile-template',help='Пустой профиль: все проверки неподтверждены')
    profile.add_argument('--robot-id',required=True)
    profile.add_argument('--output',required=True)
    for mode in ('observe','live'):
        child=modes.add_parser(mode,help='Бортовое чтение датчиков' if mode=='observe' else 'Бортовой учебный автономный запуск')
        child.add_argument('--connect' if mode=='observe' else '--execute',action='store_true',required=True)
        child.add_argument('--robot-id',required=True,help='Идентификатор назначенного робота')
        child.add_argument('--profile',required=mode=='live')
        child.add_argument('--sdk-dir',default='~/ai-robot',help='Каталог штатного go2.py на Pi')
        child.add_argument('--output',required=True,help='Новый каталог журнала на Pi')
        child.add_argument('--duration',type=positive,default=45. if mode=='observe' else 600.)
        if mode=='live': child.add_argument('--start-file',required=True,help='Новый файл локального сигнала; вне журнала')
    return p

def main(argv=None):
    p=parser()
    args=p.parse_args(argv)
    if not args.mode:
        p.print_help()
        return 0
    if args.mode=='profile-template':
        from .profile import RobotProfile
        profile=RobotProfile(args.robot_id)
        profile.verified={k:False for k in ('identity','motion_response','geometry_frames','localization','support_geometry')}
        profile.evidence={k:'' for k in profile.verified}
        profile.expected_identity={'serial_number':''}
        profile.geometry={'units':None,'up_axis':None,'point_frame':None,'pose_frame':None,
            'pose_child_frame':None,'source_kind':None,'voxel_size_m':None,'voxel_reference':None,
            'localization_error_m':None}
        print(write_new(args.output,asdict(profile)))
    elif args.mode=='demo':
        from .simulation import run_simulation
        result=run_simulation(seed=args.seed)
        print(write_new(args.output,result))
        print(json.dumps({k:v for k,v in result.items() if k not in ('trajectory','transitions','scene')},ensure_ascii=False))
        return 0 if result.get('phase',result.get('final_phase'))=='FINISHED' else 2
    elif args.mode=='replay':
        from .replay import replay_capture
        result=replay_capture(args.capture)
        print(write_new(args.output,result))
        print('Запись разобрана. Подключений и команд движения не было.')
    else:
        from .runtime import run_session
        result=asyncio.run(run_session(args))
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 2 if result.get('fault') else 0
    return 0

if __name__=='__main__':
    raise SystemExit(main())
