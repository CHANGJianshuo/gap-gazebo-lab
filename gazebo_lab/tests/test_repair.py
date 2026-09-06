"""Behavioral safety tests for the independently implemented repair boundary."""
import copy,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from server import initial_graph,validate_patch,propose

def patch():return {'reason':'clear barrier','insert_before':'carry','nodes':[{'id':'lift','xyz':[.5,-.24,.49],'mode':'cartesian'},{'id':'across','xyz':[.5,.24,.49],'mode':'cartesian'}]}

def test_repair_preserves_goal_and_all_existing_actions():
 g=initial_graph();before=copy.deepcopy(g);r=validate_patch(g,patch(),'carry')
 assert g==before
 assert all(r['nodes'][n]==v for n,v in g['nodes'].items())
 assert ['low_lift','lift'] in r['edges'] and ['lift','across'] in r['edges'] and ['across','carry'] in r['edges']
 assert ['retreat','verify'] in r['edges'] and ['verify','done'] in r['edges']

@pytest.mark.parametrize('mutate',[
 lambda p:p.update(insert_before='verify'),
 lambda p:p.update(disable_collisions=True),
 lambda p:p['nodes'][0].update(xyz=[.5,0,float('nan')]),
 lambda p:p['nodes'][0].update(xyz=[.5,0,4]),
 lambda p:p['nodes'][0].update(id='verify'),
 lambda p:p['nodes'][0].update(mode='teleport'),
 lambda p:p['nodes'][0].update(tool='eval'),
 lambda p:p.update(nodes=[]),
])
def test_reject_unsafe_or_goal_changing_patches(mutate):
 p=patch();mutate(p)
 with pytest.raises(ValueError):validate_patch(initial_graph(),p,'carry')

def test_rules_do_not_fabricate_repairs_for_unrecognized_failures():
 with pytest.raises(RuntimeError,match='no repair'):
  propose(initial_graph(),{'node':'close','error':'GRASP_FAILED','world':{}},'rules')

def test_quintic_peak_bounds_include_between_waypoint_extrema():
 from trajectory_math import segment,required_scale
 # Rest-to-rest one-radian move: maxima are interior, not at endpoints.
 c=segment([0],[0],[0],[1],[0],[0],1.)
 factor,peaks=required_scale([(c,1.)])
 assert factor>1
 assert peaks['velocity']<=.6 and peaks['acceleration']<=1.2 and peaks['jerk']<=8
 assert abs(peaks['velocity']*factor-1.875)<1e-9
 assert abs(peaks['jerk']*factor**3-60)<1e-8
