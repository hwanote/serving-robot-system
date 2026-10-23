from pathlib import Path
import importlib.util,math,yaml,json,hashlib
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('team_policy',root/'ros_ws/src/cafe_autonomy/cafe_autonomy/policy.py')
import sys
policy=importlib.util.module_from_spec(spec);sys.modules[spec.name]=policy;spec.loader.exec_module(policy)

def test_nonfinite_and_reverse_commands_stop():
    for value in (float('nan'),float('inf'),-float('inf')):
        assert policy.velocity_limit(value,.1)==(0.,0.)
        assert policy.velocity_limit(.1,value)==(0.,0.)
    assert policy.velocity_limit(-.2,-3.)==(0.,-.8)

def test_distance_boundaries():
    assert policy.person_factor(.8)==0.
    assert policy.person_factor(1.5)==1.
    assert 0.<policy.person_factor(1.1)<1.

def test_terminal_failure_does_not_block_other_robot():
    core=policy.FleetCore();core.submit('one','table_01','robot_01');core.submit('two','table_02','robot_02')
    assert core.dispatch_next({'robot_01','robot_02'}).request_id=='one'
    core.update('robot_01','ERROR','one')
    assert core.tasks['one'].state=='FAILED'
    assert core.dispatch_next({'robot_02'}).request_id=='two'

def test_pickup_and_handover_require_separate_confirmation():
    assert policy.next_after_nav('GOING_PICKUP','KITCHEN_PICKUP')=='WAIT_LOAD'
    assert policy.next_after_nav('GOING_TABLE','KITCHEN_PICKUP')=='WAIT_HANDOVER'
    assert not policy.may_confirm('COMPLETE','GOING_TABLE','KITCHEN_PICKUP')
    assert policy.next_after_nav('RETURNING_HOME','KITCHEN_PICKUP')=='COMPLETED'

def test_actual_configuration_and_command_chain():
    for r in policy.ROBOT_IDS:
        path=root/'config/integration'/('nav2_'+r+'.yaml');text=path.read_text();assert '&id' not in text and '*id' not in text
        data=yaml.safe_load(text);c=data[f'/{r}/controller_server']['ros__parameters']
        assert c['FollowPath']['plugin']=='nav2_mppi_controller::MPPIController'
        assert c['FollowPath']['vx_min']==0.;assert c['FollowPath']['vx_max']==.25;assert c['FollowPath']['wz_max']==.8
        assert c['general_goal_checker']['xy_goal_tolerance']==.1
        cm=data[f'/{r}/collision_monitor']['ros__parameters'];assert cm['cmd_vel_in_topic']=='cmd_vel_semantic';assert cm['cmd_vel_out_topic']=='cmd_vel'
        assert cm['base_frame_id']==r+'/base_footprint'
        assert data[f'/{r}/global_costmap']['global_costmap']['ros__parameters']['keepout_filter']['enabled']

def test_world_and_original_maps_preserved():
    records=json.loads((root/'tests/integration/protected_assets.json').read_text())
    assert records
    for path,sha in records.items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==sha,path

def test_table_headings_face_actual_table_positions():
    wp=yaml.safe_load((root/'config/integration/waypoints.yaml').read_text())['destinations']
    for i,(x,y) in enumerate([(-1.6,.65),(1.55,.65),(-1.6,-1.45),(1.55,-1.45)],1):
        for p in wp[f'table_0{i}']['approaches'].values():
            expected=math.atan2(y+2.25-p['y'],x+3.2-p['x'])
            assert abs(policy.norm_angle(expected-p['yaw']))<1e-9
