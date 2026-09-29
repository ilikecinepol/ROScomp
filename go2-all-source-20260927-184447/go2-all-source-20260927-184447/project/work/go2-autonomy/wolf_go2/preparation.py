"""Автоматический разбор измерений; не подменяет аппаратную калибровку."""
import math
import statistics


def timing_estimate(pairs):
    """Оцениваем масштаб часов и разброс приёма, а не абсолютную задержку съёмки."""
    rows=[(float(r),float(s)) for r,s in pairs if isinstance(r,(int,float)) and isinstance(s,(int,float))
          and not isinstance(r,bool) and not isinstance(s,bool) and math.isfinite(r) and math.isfinite(s)]
    result={'samples':len(rows),'verified':False,'absolute_latency_s':None}
    if len(rows)<10:return {**result,'status':'insufficient_samples'}
    if any(b[0]<=a[0] or b[1]<a[1] for a,b in zip(rows,rows[1:])):
        return {**result,'status':'non_monotonic'}
    if rows[-1][1]==rows[0][1]:return {**result,'status':'constant_source_stamp'}
    # Длинные интервалы уменьшают влияние джиттера доставки на масштаб.
    half=len(rows)//2
    slopes=[(b[0]-a[0])/(b[1]-a[1]) for a,b in zip(rows,rows[half:]) if b[1]>a[1] and b[0]-a[0]>=1.]
    if not slopes:return {**result,'status':'insufficient_time_span'}
    scale=statistics.median(slopes)
    r0,s0=rows[0]
    residuals=sorted((r-r0)-(s-s0)*scale for r,s in rows)
    low=residuals[int((len(rows)-1)*.05)]; high=residuals[int((len(rows)-1)*.95)]
    return {**result,'status':'scale_estimate_only','source_to_receive_scale':scale,
            'receive_residual_p90_s':high-low,
            'reason':'Смещение часов и задержка доставки неразличимы по пассивной записи; контракт времени не создан'}


def prepare_report(report,monitor=None,profile=None):
    """Каждый запуск пересчитывает причины; файл боевого профиля не изменяет."""
    estimates={}
    if monitor is not None:
        estimates={name:timing_estimate(stream.source_pairs) for name,stream in monitor.streams.items()}
    blockers=[]
    if report.get('fault'):blockers.append('Ошибка сеанса: '+str(report['fault']))
    if profile is None:blockers.append('Бортовой профиль не настроен: нужны измерения систем координат и времени')
    else:blockers.extend(profile.readiness(profile.robot_id))
    audit=report.get('packet_audit',{})
    if not audit.get('acquisition_age_verified'):
        blockers.append('Не установлена задержка от измерения лидара до использования облака')
    if audit.get('transport_status')=='content_changes_source_time_unavailable':
        blockers.append('Содержимое лидара обновляется, но метка времени не продвигается')
    wire=report.get('wire_audit',{})
    first=wire.get('first') or {}
    coarse_wire=False
    try:
        coarse_wire=(wire.get('packets',0)>=10 and wire.get('distinct_exact_stamps')==1
                     and first.get('stamp_form')=='number'
                     and float(first.get('represented_decimal_quantum',0))>=1.)
    except (ValueError,TypeError):pass
    if coarse_wire:
        blockers.append('Время лидара уже в переданном JSON имеет грубую разрядность: '+str(first['represented_decimal_quantum'])+
                        ' единиц. Перевод в float на Pi не восстанавливает время отдельных сканов')
    scene=report.get('scene_audit',{})
    body=scene.get('body_map_audit',{})
    if body.get('status') and body['status']!='body_clear':
        blockers.insert(0,body['reason'])
    if not scene.get('route_verified'):blockers.append('Стойки камеры и лидара ещё не сопоставлены; свободный маршрут не подтверждён')
    # Даже отсутствие диагностических ошибок не является физическим испытанием.
    blockers.append('Автоматическая подготовка не подтверждает прохождение навыков на роботе')
    return {'mode':'automatic_measurement_review','ready_for_motion':False,
            'profile_modified':False,'timing_estimates':estimates,
            'lidar_wire_time_status':'coarse_constant_number_on_wire' if coarse_wire else 'not_established',
            'camera_poles':len(scene.get('camera_striped_poles',[])),
            'lidar_row_hypotheses':len(scene.get('slalom_row_candidates',[])),
            'blockers':list(dict.fromkeys(blockers)),
            'next_step':'Привязать камеру и лидар к корпусу по измерениям и установить временной контракт; затем испытать короткий сегмент'}
