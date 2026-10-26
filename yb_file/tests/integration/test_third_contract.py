import math,sys
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.rotation_guard import RotationGuard
from cafe_autonomy.policy import FleetCore,next_after_nav,may_confirm

def cfg():return yaml.safe_load((root/'config/integration/runtime_limits.yaml').read_text())['navigation']
def test_cumulative_rotation_survives_direction_reversal_until_eight_cm():
 g=RotationGuard(cfg());g.observe(0.,0.,0.,.5,0.)
 for i in range(1,17):g.observe(0.,0.,math.radians(i*10),.5,i*.2)
 assert g.permits(.5)
 for i in range(1,3):g.observe(0.,0.,math.radians(160-i*10),-.5,4+i*.2)
 assert not g.permits(-.5) and g.reason=='CUMULATIVE_ROTATION_LIMIT'
 g.observe(.081,0.,math.radians(140),0.,5.)
 assert g.total==0. and g.reason is None

def test_continuous_limit_survives_translation_reset_of_cumulative():
 g=RotationGuard(cfg());g.observe(0.,0.,0.,.5,0.)
 for i in range(1,19):g.observe(i*.09,0.,math.radians(i*10),.5,i*.2)
 assert g.total<math.radians(20)
 assert not g.permits(.5) and g.reason=='CONTINUOUS_ROTATION_LIMIT'
 g.observe(1.7,0.,math.pi,0.,4.);g.observe(1.7,0.,math.pi,0.,4.4)
 assert g.continuous==0. and g.reason is None

def test_explicit_followup_modes_and_collection_confirmation():
 core=FleetCore();home,_=core.submit('home',None,'robot_01','GO_HOME');kitchen,_=core.submit('kitchen',None,'robot_02','GO_KITCHEN')
 assert core.dispatch_next({'robot_01','robot_02'}).mode=='GO_HOME'
 assert next_after_nav('GOING_KITCHEN','GO_KITCHEN')=='COMPLETED'
 assert may_confirm('COLLECTED','WAIT_HANDOVER','TABLE_COLLECT')
 assert not may_confirm('COMPLETE','WAIT_HANDOVER','TABLE_COLLECT')

def test_nav2_progress_checker_is_forty_seconds():
 for r in ('robot_01','robot_02'):
  c=yaml.safe_load((root/'config/integration'/('nav2_'+r+'.yaml')).read_text())
  assert c['/'+r+'/controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']==40.


def test_recovery_refreshes_costmaps_without_reverse_or_spin():
 for name in ('navigate_no_reverse.xml','navigate_forward_curves.xml'):
  text=(root/'ros_ws/src/cafe_autonomy/config'/name).read_text()
  assert 'ClearEntireCostmap' in text and '<Wait ' in text
  assert '<BackUp ' not in text and '<Spin ' not in text
