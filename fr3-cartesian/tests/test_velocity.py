"""Physical units, vector integration and worker-owned servo lifecycle."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from fr3_cartesian.geometry import rotation
from fr3_cartesian.joystick import StrokeGate, default_profile, throttle_fraction, validate_profile
from fr3_cartesian.messages import MoveRejected, OperatorError
from fr3_cartesian.velocity import INPUT_TIMEOUT, SERVO_PERIOD, TrackingGovernor, VelocityIntegrator
from fr3_cartesian.worker import Session


def test_velocity_scales_in_mm_per_second_and_caps_total_path_with_reversals():
    motion = VelocityIntegrator(10, 1., acceleration=250)
    for _ in range(1000):
        delta = motion.step([6.,8.,0.])
        assert np.linalg.norm(delta) <= 10*SERVO_PERIOD+1e-10
        if motion.exhausted: break
    assert motion.distance == pytest.approx(1)
    np.testing.assert_allclose(motion.position,[.6,.8,0],atol=1e-9)
    assert np.linalg.norm(motion.step([-6,-8,0])) == pytest.approx(0)


def test_zero_has_no_tail_and_speed_change_is_acceleration_limited():
    motion=VelocityIntegrator(25,10,100)
    np.testing.assert_allclose(motion.step([25,0,0]), [.8*.008,0,0])
    np.testing.assert_allclose(motion.step([0,0,0]), [0,0,0])
    assert np.linalg.norm(motion.velocity)==0
    with pytest.raises(ValueError,match='maximum'): motion.step([25,25,0])
    with pytest.raises(ValueError): motion.step([0,0,float('nan')])


def test_lowered_throttle_caps_speed_on_the_next_increment():
    motion=VelocityIntegrator(25,10,100)
    for _ in range(40): motion.step([25,0,0])
    assert np.linalg.norm(motion.velocity)==pytest.approx(25)
    increment=motion.step([2,0,0])
    assert np.linalg.norm(increment)==pytest.approx(2*SERVO_PERIOD)
    np.testing.assert_allclose(motion.velocity,[2,0,0])


def test_vector_gate_updates_three_components_proportionally_and_throttle_live():
    profile=validate_profile(default_profile() | dict(axes=[0,1,4],centers={'0':128,'1':128,'4':128},
        invert=[False,False,False], tcp_speed=10., throttle_axis=3))
    ranges={a:dict(min=0,max=255) for a in (0,1,3,4)}
    center={0:128,1:128,3:255,4:128}
    gate=StrokeGate()
    gate.update(center,set(),ranges,profile,True,now=0,vector_mode=True)
    def sample(values,now):
        return gate.update(center|values,{288},ranges,profile,True,now=now,vector_mode=True)[0]
    first=sample({0:255,1:255,4:255},.02)
    np.testing.assert_allclose(first.velocity,[10/np.sqrt(3)]*3)
    second=sample({0:255,1:128,4:128},.04)
    np.testing.assert_allclose(second.velocity,[10,0,0])
    half=sample({0:255,3:128},.06)
    assert 4.9 < half.velocity[0] < 5.1
    zero=sample({0:255,3:0},.08)
    assert zero.velocity==(0.,0.,0.)
    reverse=sample({0:0,3:255},.10)
    np.testing.assert_allclose(reverse.velocity,[-10,0,0])
    # It uses a linear response regardless of the joint-mode exponent.
    part=sample({0:200,3:255},.12)
    assert part.velocity[0] == pytest.approx(((200-128)/127-.15)/.85*10)
    demand=gate.update(center|{0:255},set(),ranges,profile,True,now=.14,vector_mode=True)[0]
    assert demand is None
    assert gate.update(center|{0:255},{288},ranges,profile,True,now=.16,vector_mode=True)[0] is None


def test_throttle_is_independent_of_center_inversion_and_axis_assignment():
    profile=default_profile()|dict(throttle_axis=3)
    ranges={3:dict(min=10,max=110)}
    assert throttle_fraction({3:10},ranges,profile)==0
    assert throttle_fraction({3:110},ranges,profile)==1
    assert throttle_fraction({3:60},ranges,profile)==pytest.approx(.5)
    assert throttle_fraction({3:10},ranges,profile|dict(throttle_invert=True))==1
    with pytest.raises(ValueError): validate_profile(profile|dict(axes=[0,3,None]))
    with pytest.raises(ValueError): validate_profile(profile|dict(throttle_axis=True))
    with pytest.raises(ValueError): throttle_fraction({},ranges,profile)


def test_vector_center_keeps_zero_demand_in_the_same_held_trigger_stroke():
    profile=validate_profile(default_profile() | dict(centers={'0':128,'1':128},invert=[False,False,False]))
    ranges={a:dict(min=0,max=255) for a in (0,1)}
    gate=StrokeGate()
    gate.update({0:128,1:128},set(),ranges,profile,True,now=0,vector_mode=True)
    first=gate.update({0:255,1:128},{288},ranges,profile,True,now=.02,vector_mode=True)[0]
    assert first.velocity[0]>0
    center=gate.update({0:128,1:128},{288},ranges,profile,True,now=.04,vector_mode=True)[0]
    assert center.velocity==(0.,0.,0.) and gate.active is not None
    other=gate.update({0:128,1:255},{288},ranges,profile,True,now=.06,vector_mode=True)[0]
    assert other.velocity[1]>0
    assert gate.update({0:128,1:255},set(),ranges,profile,True,now=.08,vector_mode=True)[0] is None


def start_sim(travel=2, speed=10):
    s=Session()
    result=s.servo_start('stroke',travel,speed,time.monotonic())
    assert result['servo_active'] and s.servo
    return s


def test_sim_servo_is_simultaneous_and_updates_without_restarting():
    s=start_sim()
    origin=np.array(s.flange[:3])
    orientation=s.flange[3:]
    now=time.monotonic()
    s.servo_update('stroke',[3,4,0],now)
    s.servo_tick(now)
    assert s.flange[0]>origin[0] and s.flange[1]>origin[1]
    for _ in range(5):
        s.servo_update('stroke',[3,0,4],time.monotonic())
        s.servo_tick()
    assert s.flange[2]>origin[2]
    assert s.flange[3:]==orientation
    ended=s.servo_end('stroke')
    assert not ended['servo_active'] and ended['stopped'] and s.servo is None
    s.servo_update('stroke',[10,0,0],time.monotonic())
    assert s.servo is None


@pytest.mark.parametrize('reason', ['input_timeout','timing_delay','travel_limit'])
def test_stream_ends_without_replay_on_expiry_delayed_cycle_or_shared_distance(reason):
    s=start_sim(travel=.1)
    if reason=='input_timeout': s.servo['input_at']=time.monotonic()-INPUT_TIMEOUT-.01
    elif reason=='timing_delay': s.servo['tick_at']=time.monotonic()-.05
    else:
        for i in range(100):
            s.servo_update('stroke',[6,8,0],time.monotonic())
            ended=s.servo_tick()
            if ended: break
    if reason!='travel_limit': ended=s.servo_tick()
    assert ended['servo_reason']==reason and not ended['servo_active']
    if reason=='travel_limit': assert ended['servo_distance']==pytest.approx(.1)
    assert s.servo is None


def test_wrong_token_or_old_packets_never_refresh_motion():
    s=start_sim()
    previous=s.servo['input_at']
    s.servo_update('other',[10,0,0],time.monotonic())
    assert s.servo['velocity']==[0,0,0] and s.servo['input_at']==previous
    s.servo_update('stroke',[10,0,0],time.monotonic()-1)
    assert s.servo['velocity']==[0,0,0] and s.servo['input_at']==previous
    with pytest.raises(ValueError): s.servo_update('stroke',[20,0,0],time.monotonic())
    s.servo_end('stroke')


def test_servo_start_rejects_wrong_base_or_expired_input_without_writing():
    s=Session();s.workpiece_slot=1
    with pytest.raises(MoveRejected,match='base workpiece'): s.servo_start('stroke',2,10,time.monotonic())
    s.workpiece_slot=0;s.workpiece_pose=[1,0,0,0,0,0]
    with pytest.raises(MoveRejected): s.servo_start('stroke',2,10,time.monotonic())
    s.workpiece_pose=[0]*6
    with pytest.raises(MoveRejected,match='expired'): s.servo_start('stroke',2,10,time.monotonic()-1)
    assert s.servo is None


def fake_real(response=1.):
    s=Session();initial=s.read();s.sim=False
    calls=[]
    pkg=SimpleNamespace(frame_cnt=1, tool=0, user=0,tl_cur_pos=initial['tcp'],jt_cur_pos=[0]*6,
        main_code=0,sub_code=0, EmergencyStop=0,collisionState=0,safety_stop0_state=0,safety_stop1_state=0,
        rbtEnableState=1,robot_mode=1,program_state=1,robot_state=1)
    class Robot:
        reconnect_flag=False
        robot_state_pkg=pkg
        def ServoMoveStart(self): calls.append(('start',));return 0
        def ServoCart(self,mode,pose,cmdT):
            calls.append(('cart',mode,pose,cmdT))
            actual=np.array(pkg.tl_cur_pos[:3])
            target=np.array(pose[:3]) if mode==0 else actual+pose[:3]
            attitude=pose[3:] if mode==0 else (np.array(pkg.tl_cur_pos[3:])+pose[3:]).tolist()
            pkg.tl_cur_pos=[*(actual+response*(target-actual)),*attitude]
            pkg.frame_cnt+=1
            return 0
        def StopMotion(self): calls.append(('stop',));return 0
        def ServoMoveEnd(self): calls.append(('end',));return 0
    s.robot=Robot()
    s.ready=lambda motion=False:initial
    original=s.read
    s.read=lambda: s.servo_state() if s.servo else initial
    return s,calls,pkg


def test_real_servo_api_lifecycle_uses_absolute_base_tcp_with_fixed_orientation():
    s,calls,pkg=fake_real()
    origin=list(pkg.tl_cur_pos)
    s.servo_start('stroke',5,10,time.monotonic())
    s.servo_update('stroke',[3,4,5],time.monotonic())
    s.servo_tick()
    cart=calls[1]
    assert cart[0:2]==('cart',0) and cart[3]==.008
    assert all(v>start for v,start in zip(cart[2][:3],origin[:3]))
    assert np.linalg.norm(np.array(cart[2][:3])-origin[:3])<=10*.008
    assert cart[2][3:]==origin[3:]
    assert s.servo['target']==cart[2]
    s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','cart','stop','end']


def test_lagging_plant_follows_full_three_axis_stroke_without_incremental_drift():
    # A plant moves partway toward each target. Relative-to-actual increments
    # lose distance every cycle and fail the old accumulated-position check;
    # absolute encoder targets retain bounded tracking lag over the full path.
    s,calls,pkg=fake_real(response=.2)
    s.servo_start('stroke',15,10,time.monotonic())
    largest_error=0.
    for _ in range(1000):
        s.servo_update('stroke',[3.,4.,5.],time.monotonic())
        ended=s.servo_tick()
        if ended: break
        error=np.linalg.norm(np.array(pkg.tl_cur_pos[:3])-s.servo['target'][:3])
        largest_error=max(largest_error,error)
    assert ended['servo_reason']=='travel_limit' and ended['servo_distance']==pytest.approx(15.)
    assert 0.<largest_error<1.
    targets=[call[2] for call in calls if call[0]=='cart']
    assert len(targets)>200 and all(call[1]==0 for call in calls if call[0]=='cart')
    assert all(target[3:]==[180.,0.,0.] for target in targets)
    assert all(np.linalg.norm(np.array(b[:3])-a[:3])<=10*.008+1e-9
               for a,b in zip(targets,targets[1:]))


def test_absolute_targets_hold_at_neutral_and_reverse_without_reference_reset():
    s,calls,pkg=fake_real(response=.25)
    s.servo_start('stroke',15,10,time.monotonic())
    for demand,count in (([3,4,5],40),([0,0,0],12),([-3,-4,-5],80)):
        before=s.servo['integrator'].distance
        held_target=list(s.servo['target'])
        for _ in range(count):
            s.servo_update('stroke',demand,time.monotonic())
            assert s.servo_tick() is None
        if demand==[0,0,0]:
            assert s.servo['integrator'].distance==before
            assert s.servo['target']==held_target
    s.servo_end('stroke')
    assert calls[-2:]==[('stop',),('end',)]


def test_one_mm_hard_guard_still_reports_complete_diagnostics():
    s,calls,pkg=fake_real(response=0.)
    s.servo_start('stroke',15,10,time.monotonic())
    pkg.tl_cur_pos=list(pkg.tl_cur_pos)
    pkg.tl_cur_pos[1]+=1.1
    with pytest.raises(OperatorError,match='following error') as caught:
        s.servo_tick()
    message=str(caught.value)
    for detail in ('exceeds 1 mm','absolute ServoCart writes','actual XYZ','target XYZ',
                   'delta XYZ','feedback age','last RPC','commanded speed','travel'):
        assert detail in message
    assert s.servo['integrator'].distance==0
    s.servo_end('stroke')
    assert calls[-2:]==[('stop',),('end',)]


@pytest.mark.parametrize('requested_speed', [15.,25.])
def test_higher_speed_with_80ms_controller_delay_completes_without_tracking_disconnect(monkeypatch,requested_speed):
    clock=[100.]
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic',lambda:clock[0])
    s,calls,pkg=fake_real(response=0.)
    queued=[]
    original=s.robot.ServoCart
    def delayed(mode,pose,cmdT):
        original(mode,pose,cmdT)
        queued.append((clock[0]+.08,list(pose)))
        return 0
    s.robot.ServoCart=delayed
    s.servo_start('stroke',35,requested_speed,clock[0])
    max_error=0.;min_scale=1.
    for _ in range(6000):
        clock[0]+=.008
        while queued and queued[0][0]<=clock[0]:
            pkg.tl_cur_pos=queued.pop(0)[1]
        pkg.frame_cnt+=1
        s.robot._fr3_feedback_at=clock[0]
        direction=np.array([1.,2.,3.])/np.sqrt(14.)
        s.servo_update('stroke',(direction*requested_speed).tolist(),clock[0])
        ended=s.servo_tick()
        if ended: break
        live=s.servo_state()
        max_error=max(max_error,live['servo_tracking_error'])
        min_scale=min(min_scale,live['servo_tracking_scale'])
        assert np.linalg.norm(live['servo_velocity'])<=requested_speed+1e-7
    assert ended['servo_reason']=='travel_limit' and ended['servo_distance']==pytest.approx(35.)
    assert max_error<.6+1e-9 and min_scale<.8
    assert all(c[1]==0 for c in calls if c[0]=='cart')


def test_stalled_plant_stops_with_bounded_target_lead_even_below_hard_guard(monkeypatch):
    clock=[100.]
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic',lambda:clock[0])
    s,calls,pkg=fake_real(response=0.)
    s.servo_start('stroke',35,25,clock[0])
    with pytest.raises(OperatorError,match='less than 0.05 mm progress'):
        for _ in range(200):
            clock[0]+=.008
            pkg.frame_cnt+=1;s.robot._fr3_feedback_at=clock[0]
            s.servo_update('stroke',[0,25.,0],clock[0])
            s.servo_tick()
    assert clock[0]<101.2 and s.servo['integrator'].distance<=.6
    s.servo_end('stroke')
    assert calls[-2:]==[('stop',),('end',)]


def test_delayed_plant_reverses_crosses_center_and_changes_throttle_without_backlog(monkeypatch):
    clock=[100.]
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic',lambda:clock[0])
    s,calls,pkg=fake_real(response=0.)
    queue=[];original=s.robot.ServoCart
    def delayed(mode,pose,cmdT):
        original(mode,pose,cmdT);queue.append((clock[0]+.08,list(pose)));return 0
    s.robot.ServoCart=delayed
    s.servo_start('stroke',50,25,clock[0])
    for demand in ([0,25,0],[0,-25,0],[0,0,0],[1,1,1],[0,0,0],[10,10,10]):
        distance=s.servo['integrator'].distance
        held_target=list(s.servo['target'])
        for _ in range(100):
            clock[0]+=.008
            while queue and queue[0][0]<=clock[0]: pkg.tl_cur_pos=queue.pop(0)[1]
            pkg.frame_cnt+=1;s.robot._fr3_feedback_at=clock[0]
            s.servo_update('stroke',demand,clock[0])
            assert s.servo_tick() is None
            assert np.linalg.norm(s.servo['integrator'].velocity)<=np.linalg.norm(demand)+1e-7
        if demand==[0,0,0]:
            assert s.servo['target']==held_target
            assert s.servo['integrator'].distance==distance
    s.servo_end('stroke')
    assert calls[-2:]==[('stop',),('end',)]


def test_governor_neutral_and_throttle_zero_hold_without_stall_or_catchup():
    governor=TrackingGovernor([0,0,0],0.)
    np.testing.assert_allclose(governor.limit([0,20,0],[0,0,0],[0,.4,0],.1),[0,10,0])
    for now in (.2,1.,3.):
        np.testing.assert_allclose(governor.limit([0,0,0],[0,0,0],[0,.4,0],now),[0,0,0])
    # Resuming has the same scaled demand, no accumulated velocity debt.
    np.testing.assert_allclose(governor.limit([0,20,0],[0,0,0],[0,.4,0],3.1),[0,10,0])
    np.testing.assert_allclose(governor.limit([0,20,0],[0,0,0],[0,.6,0],3.2),[0,0,0])


def test_feedback_socket_failure_blocks_next_increment_but_allows_stop_commands():
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    s.robot._fr3_feedback_error='Robot feedback socket closed'
    with pytest.raises(OperatorError,match='feedback is stale'): s.servo_tick()
    s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','stop','end']


def test_servo_end_waits_out_delayed_running_feedback_before_returning(monkeypatch):
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    initial=s.servo['state']
    states=iter([dict(initial,program_stopped=False,stopped=False),
                 dict(initial,program_stopped=False,stopped=True), initial])
    monkeypatch.setattr(s,'read',lambda:next(states))
    monkeypatch.setattr('fr3_cartesian.worker.time.sleep',lambda duration:None)
    result=s.servo_end('stroke')
    assert result['program_stopped'] and result['stopped']
    assert [c[0] for c in calls]==['start','stop','end']


def test_servo_stop_settling_has_a_deadline_and_notifies_before_stop_calls(monkeypatch):
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    initial=s.servo['state']
    s.event_sink=lambda event:calls.append(('notification',event))
    monkeypatch.setattr(s,'read',lambda:dict(initial,stopped=False,program_stopped=False))
    clock=iter([0.,1.01])
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic',lambda:next(clock))
    with pytest.raises(OperatorError,match='did not settle'): s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','notification','stop','end']
    assert calls[1][1]==dict(event='servo_stopping',token='stroke')
    assert s.servo is None


@pytest.mark.parametrize('field,value', [('user',1),('tool',2),('EmergencyStop',1),('main_code',74),('rbtEnableState',0),('robot_mode',0),('program_state',2)])
def test_controller_changes_block_before_next_servo_increment(field,value):
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    setattr(pkg,field,value)
    with pytest.raises(OperatorError) as error: s.servo_tick()
    if field not in ('user', 'tool'):
        assert 'after 0 ServoCart writes' in str(error.value)
        assert f'{field}={value} (start=' in str(error.value)
    assert [c[0] for c in calls]==['start']
    s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','stop','end']


def test_servo_start_transition_is_reported_before_any_position_increment():
    s,calls,pkg=fake_real()
    def start():
        calls.append(('start',))
        pkg.program_state=2
        pkg.robot_state=2
        return 0
    s.robot.ServoMoveStart=start
    with pytest.raises(OperatorError, match='program_state=2 \\(start=1\\)') as error:
        s.servo_start('stroke',5,10,time.monotonic())
    assert 'after 0 ServoCart writes' in str(error.value)
    assert 'robot_mode=1 (start=1)' in str(error.value)
    assert 'main_code=0 (start=0)' in str(error.value)
    assert calls==[('start',)]
    s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','stop','end']


def test_already_running_program_is_rejected_before_servo_start_even_if_preflight_was_older():
    s,calls,pkg=fake_real()
    pkg.program_state=2
    with pytest.raises(MoveRejected,match='program stopped'):
        s.servo_start('stroke',5,10,time.monotonic())
    assert calls==[] and s.servo is None


def test_qx_program_running_transition_after_three_servo_writes_keeps_stream_alive():
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    s.servo_update('stroke',[3,4,0],time.monotonic())
    for _ in range(3): s.servo_tick()
    assert s.servo['writes']==3
    # Replay the user's FR3 trace: manual, enabled, no faults; both running
    # markers turn to 2 after the third accepted ServoCart command.
    pkg.program_state=2
    pkg.robot_state=2
    for _ in range(6):
        s.servo_update('stroke',[3,0,4],time.monotonic())
        assert s.servo_tick() is None
        state=s.servo_state()
        assert state['servo_active'] and state['servo_program_running']
        assert not state['program_stopped']
    assert len([c for c in calls if c[0]=='cart'])==9
    ended=s.servo_end('stroke')
    assert ended['stopped'] and not ended['servo_active']
    assert [c[0] for c in calls[-2:]]==['stop','end']


@pytest.mark.parametrize('field,value', [('robot_mode',0),('rbtEnableState',0),('EmergencyStop',1),
    ('collisionState',1),('safety_stop0_state',1),('safety_stop1_state',1),('main_code',74),
    ('sub_code',2),('program_state',3),('program_state',0),('robot_state',3),('robot_state',4)])
def test_actual_faults_and_pause_still_block_during_owned_running_servo(field,value):
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    s.servo_tick()
    pkg.program_state=2
    pkg.robot_state=2
    setattr(pkg,field,value)
    with pytest.raises(OperatorError,match='readiness changed'):
        s.servo_tick()
    assert len([c for c in calls if c[0]=='cart'])==1
    s.servo_end('stroke')
    assert [c[0] for c in calls[-2:]]==['stop','end']


@pytest.mark.parametrize('change', ['stale','following_error','orientation','travel'])
def test_bad_servo_feedback_blocks_before_the_next_increment(change):
    s,calls,pkg=fake_real()
    s.servo_start('stroke',.5,10,time.monotonic())
    if change=='stale':
        s.servo['frame']=pkg.frame_cnt
        s.servo['frame_at']=time.monotonic()-.11
    elif change=='orientation': pkg.tl_cur_pos=list(pkg.tl_cur_pos[:3])+[170,0,0]
    else:
        pkg.tl_cur_pos=list(pkg.tl_cur_pos)
        pkg.tl_cur_pos[0]+=1.1 if change=='following_error' else .8
    with pytest.raises(OperatorError,match={'stale':'stale','following_error':'following error',
                                          'orientation':'orientation','travel':'travel limit'}[change]):
        s.servo_tick()
    assert [c[0] for c in calls]==['start']
    s.servo_end('stroke')


def test_failed_servo_write_is_never_replayed_and_both_stop_calls_are_attempted():
    s,calls,pkg=fake_real()
    s.servo_start('stroke',5,10,time.monotonic())
    def failed(*args,**kwargs): calls.append(('failed_cart',));raise OperatorError('lost')
    s.robot.ServoCart=failed
    with pytest.raises(OperatorError): s.servo_tick()
    def stop_failed(): calls.append(('stop',));raise OperatorError('stop lost')
    s.robot.StopMotion=stop_failed
    with pytest.raises(OperatorError,match='not confirmed'): s.servo_end('stroke')
    assert [c[0] for c in calls]==['start','failed_cart','stop','end']
    assert s.servo is None


def test_worker_process_stops_itself_when_input_heartbeat_disappears():
    root=Path(__file__).resolve().parents[1]
    # Use pytest's interpreter, just as RobotClient does, so the checkout is
    # testable with any environment location rather than a pre-existing .venv.
    process=subprocess.Popen([sys.executable,'-m','fr3_cartesian.worker'],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0,cwd=root)
    buffer=b''
    def send(value): process.stdin.write((json.dumps(value)+'\n').encode());process.stdin.flush()
    def receive():
        nonlocal buffer
        deadline=time.monotonic()+5
        while b'\n' not in buffer:
            assert select.select([process.stdout],[],[],max(0,deadline-time.monotonic()))[0], 'Worker reply timed out'
            data=os.read(process.stdout.fileno(),65536)
            assert data, process.stderr.read().decode()
            buffer+=data
        line,buffer=buffer.split(b'\n',1)
        return json.loads(line)
    try:
        send(dict(id=1,method='connect',args=dict(simulation=True)))
        assert receive()['result']['stopped']
        send(dict(id=2,method='servo_start',args=dict(token='process',travel=5,speed=10,sent_at=time.monotonic())))
        assert receive()['result']['servo_active']
        send(dict(method='servo_update',args=dict(token='process',velocity=[3,4,0],sent_at=time.monotonic())))
        reports=[]
        while not reports or reports[-1].get('event')!='servo_end': reports.append(receive())
        ended=reports[-1]['result']
        assert ended['servo_reason']=='input_timeout' and ended['stopped']
        assert ended['servo_distance']>0
        assert ended['flange'][0]>300 and ended['flange'][1]>0
        send(dict(id=3,method='read',args={}))
        final=receive()['result']
        assert final['stopped'] and final['flange']==ended['flange']
    finally:
        process.stdin.close()
        process.wait(timeout=5)
        process.stdout.close();process.stderr.close()
