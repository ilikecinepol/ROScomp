"""Единственный отправитель Move; срок команд и остановка независимы от зрения."""
import asyncio
import math
import time
from dataclasses import replace
from .models import Decision

def acknowledged(response):
    try:
        code=response['data']['header']['status']['code']
        return type(code) is int and code == 0
    except (TypeError,KeyError):
        return False

class CommandGate:
    def __init__(self, policy, ttl=.35):
        self.policy=policy.validate()
        self.ttl=ttl
        self.armed=False
        self.fault=None
        self.intent=Decision()
        self.issued=-math.inf
        self.last_tick=None
        self.previous=(0.,0.,0.)
        self.generation=0

    def arm(self):
        if self.fault:
            raise RuntimeError('Остановка зафиксирована; необходима новая сессия: '+self.fault)
        self.armed=True
        self.generation+=1

    def trip(self, reason):
        self.fault=self.fault or reason
        self.armed=False
        self.previous=(0.,0.,0.)
        self.generation+=1

    def submit(self, decision, now):
        if not math.isfinite(now) or not all(type(v) in (int,float) and math.isfinite(v) for v in (decision.vx,decision.vy,decision.wz)):
            self.trip('Некорректная команда планировщика')
            return
        if now < self.issued:
            self.trip('Часы планировщика пошли назад')
            return
        self.intent,self.issued=decision,now

    def command(self, now, errors=()):
        if not math.isfinite(now) or (self.last_tick is not None and now < self.last_tick):
            self.trip('Часы управляющего цикла пошли назад или некорректны')
        if errors:
            self.trip('; '.join(errors))
        if not self.armed or self.fault:
            self.previous=(0.,0.,0.)
            return Decision(phase='STOPPED',reason=self.fault or 'Ожидание сигнала старта')
        if not 0 <= now-self.issued <= self.ttl:
            self.trip('Планировщик не обновил команду вовремя')
            return Decision(phase='STOPPED',reason=self.fault)
        desired=self.intent
        if not desired.moving:
            self.previous=(0.,0.,0.)
            self.last_tick=now
            return desired
        if desired.vx < 0:
            self.trip('Задний ход не включён в проверенный набор навыков')
            return Decision(phase='STOPPED',reason=self.fault)
        dt=min(.1,max(0.,now-self.last_tick)) if self.last_tick is not None else .05
        self.last_tick=now
        limits=(self.policy.max_vx,self.policy.max_vy,self.policy.max_wz)
        accels=(self.policy.max_accel,self.policy.max_accel,self.policy.max_yaw_accel)
        goal=(desired.vx,desired.vy,desired.wz)
        values=[]
        for previous,target,limit,accel in zip(self.previous,goal,limits,accels):
            target=max(-limit,min(limit,target))
            values.append(previous+max(-accel*dt,min(accel*dt,target-previous)))
        # Сокращаем поступательную скорость в тесном повороте.
        if abs(values[2])>.2:
            values[0]=min(values[0],.16)
        speed=math.hypot(values[0],values[1])
        if speed>self.policy.max_vx:
            values[0]*=self.policy.max_vx/speed
            values[1]*=self.policy.max_vx/speed
        self.previous=tuple(values)
        return replace(desired,vx=values[0],vy=values[1],wz=values[2])

class CommandPump:
    def __init__(self, send, gate, health, log=lambda *a,**k:None, clock=time.monotonic):
        self.send,self.gate,self.health,self.log,self.clock=send,gate,health,log,clock
        self.lock=asyncio.Lock()
        self.stopped_ack=False
        self.sent_move=False
        self.stop_attempts=0

    def event(self,event,**data):
        try:
            self.log(event,**data)
        except Exception:
            self.gate.trip('Недоступен журнал')

    async def stop(self,reason,trip=True):
        self.stopped_ack=False
        if trip:
            self.gate.trip(reason)
        self.event('stop_requested',reason=reason)
        async with self.lock:
            for attempt in range(3):
                self.stop_attempts+=1
                try:
                    response=await asyncio.wait_for(self.send('StopMove',None),.4)
                    self.event('response',command='StopMove',response=response)
                    if acknowledged(response):
                        self.stopped_ack=True
                        self.sent_move=False
                        return True
                except Exception as exc:
                    self.event('command_error',command='StopMove',error=type(exc).__name__)
        self.gate.trip('Остановка не подтверждена; требуется проверка оператором')
        return False

    async def run(self,ending):
        try:
            while not ending.is_set():
                now=self.clock()
                try:
                    errors=self.health()
                except Exception as exc:
                    errors=['Ошибка проверки состояния: '+type(exc).__name__]
                cmd=self.gate.command(now,errors)
                if self.gate.fault:
                    await self.stop(self.gate.fault)
                    ending.set()
                    return
                if not cmd.moving:
                    if self.sent_move:
                        await self.stop('Планировщик запросил остановку',trip=False)
                else:
                    epoch=self.gate.generation
                    async with self.lock:
                        # Проверка повторяется после ожидания блокировки, перед API.
                        if epoch!=self.gate.generation or not self.gate.armed or self.clock()-self.gate.issued>self.gate.ttl:
                            continue
                        try:
                            parameter={'x':cmd.vx,'y':cmd.vy,'z':cmd.wz}
                            self.event('command',command='Move',parameter=parameter,phase=cmd.phase)
                            if self.gate.fault:
                                continue
                            self.sent_move=True
                            self.stopped_ack=False
                            response=await asyncio.wait_for(self.send('Move',parameter),.2)
                            self.event('response',command='Move',response=response)
                            if not acknowledged(response):
                                self.gate.trip('Move не подтверждён')
                        except Exception as exc:
                            self.gate.trip('Ошибка Move: '+type(exc).__name__)
                await asyncio.sleep(.025)
        finally:
            # Завершаем остановку даже при отмене планировщика/самой задачи.
            task=asyncio.create_task(self.stop('Завершение управляющего цикла'))
            try:
                await asyncio.shield(task)
            except asyncio.CancelledError:
                await task
                raise
