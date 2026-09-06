#!/usr/bin/env python3
"""GaP runtime + independently implemented execution-feedback graph repair harness."""
from __future__ import annotations
import os,json,time,copy,threading,math,hashlib,re,traceback
from pathlib import Path
import httpx
from fastapi import FastAPI,HTTPException
from fastapi.responses import FileResponse,Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from gap_core.tools import ToolRegistry
from gap.runtime.executor import WorkflowExecutor
from gap.runtime.workflow import load_workflow
from gap.runtime.validate import validate_workflow
R=Path(__file__).resolve().parents[1];OUT=R/'outputs/runs';OUT.mkdir(parents=True,exist_ok=True)
app=FastAPI(title='GaP Gazebo Lab');lock=threading.RLock();stop_event=threading.Event()
state={'running':False,'phase':'idle','round':0,'provider':'rules','versions':[],'events':[],'error':None,'run_id':None,'node_status':{},'latest_world':{}}

def ros(op,args=None):
 with httpx.Client(timeout=90,trust_env=False) as c:
  res=c.post('http://127.0.0.1:9431/act',json={'op':op,'args':args or {}})
 if res.status_code!=200:raise RuntimeError(res.json().get('error',res.text))
 return res.json()

def event(kind,**kw):
 with lock:
  e={'time':time.time(),'kind':kind,**kw};state['events'].append(e)
  if state.get('run_id'):
   with (OUT/state['run_id']/'events.jsonl').open('a') as f:f.write(json.dumps(e,ensure_ascii=False)+'\n')

def initial_graph():
 def tool(name,**args):return {'type':'tool','tool':'gazebo.'+name,'inputs':args}
 nodes={
 'observe':tool('observe'),
 'pregrasp':tool('move',xyz=[.5,-.24,.36],mode='free'),
 'descend':tool('move',xyz=[.5,-.24,.17],mode='cartesian'),
 'close':tool('gripper',close=True),
 'low_lift':tool('move',xyz=[.5,-.24,.25],mode='cartesian'),
 'carry':tool('move',xyz=[.5,.24,.25],mode='cartesian'),
 'place':tool('move',xyz=[.5,.24,.178],mode='cartesian'),
 'release':tool('gripper',close=False),
 'retreat':tool('move',xyz=[.5,.24,.42],mode='cartesian'),
 'verify':tool('verify'),
 'done':{'type':'end','status':'success'}}
 names=['START',*nodes]
 return {'version':3,'meta':{'name':'Panda transfer across a barrier','source':'independent Gazebo adaptation; official GaP v3 runtime'},'nodes':nodes,'edges':[[a,b] for a,b in zip(names,names[1:])]}

def validate_patch(graph,patch,failed):
 # Deliberately narrow patch language: only insert motion nodes BEFORE the failed
 # motion. The task, evaluator, collision checker and all existing nodes stay fixed.
 if not isinstance(patch,dict) or set(patch)-{'reason','insert_before','nodes'}:raise ValueError('Invalid patch fields')
 if patch.get('insert_before')!=failed or failed!='carry':raise ValueError('Only the failed carry boundary is repairable in this first experiment')
 if not isinstance(patch.get('reason'),str) or not patch['reason'].strip():raise ValueError('Repair reason required')
 ns=patch.get('nodes');seen=set(graph['nodes'])
 if not isinstance(ns,list) or not 1<=len(ns)<=6:raise ValueError('Require 1–6 inserted motions')
 for n in ns:
  if not isinstance(n,dict) or set(n)!={'id','xyz','mode'}:raise ValueError('Motion fields must be id, xyz, mode')
  if not isinstance(n['id'],str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,39}',n['id']) or n['id'] in seen:raise ValueError('Invalid/duplicate node id')
  seen.add(n['id']);v=n['xyz']
  if not isinstance(v,list) or len(v)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in v):raise ValueError('Finite xyz required')
  if not (.2<=v[0]<=.75 and -.45<=v[1]<=.45 and .20<=v[2]<=.65):raise ValueError('Waypoint outside lab bounds')
  if n['mode'] not in ('free','cartesian'):raise ValueError('Invalid motion mode')
 result=copy.deepcopy(graph)
 for n in ns:result['nodes'][n['id']]={'type':'tool','tool':'gazebo.move','inputs':{'xyz':n['xyz'],'mode':n['mode']}}
 predecessors=[a for a,b in result['edges'] if b==failed]
 if len(predecessors)!=1:raise ValueError('Expected one repair boundary')
 result['edges'].remove([predecessors[0],failed]);chain=[predecessors[0],*[n['id'] for n in ns],failed]
 result['edges'] += [[a,b] for a,b in zip(chain,chain[1:])]
 return result

def propose(graph,feedback,provider):
 if provider=='rules':
  if feedback['node']!='carry' or 'CARTESIAN_BLOCKED' not in feedback['error']:raise RuntimeError('Offline rule has no repair for this failure; no scripted success fallback')
  tcp=feedback['world']['tcp'];h=.49
  return {'reason':'直线搬运被隔板阻挡；先升到 0.49 m，再横移，最后沿原 carry 节点下降。此建议由显式规则生成，不是大模型。','insert_before':'carry','nodes':[{'id':'lift_clearance','xyz':[tcp[0],tcp[1],h],'mode':'cartesian'},{'id':'transfer_clearance','xyz':[.5,.24,h],'mode':'cartesian'}]}
 key=os.environ.get('GAP_LLM_API_KEY');model=os.environ.get('GAP_LLM_MODEL');base=os.environ.get('GAP_LLM_BASE_URL','https://openrouter.ai/api/v1')
 if not key or not model:raise RuntimeError('LLM provider not configured: set GAP_LLM_API_KEY and GAP_LLM_MODEL in the server environment')
 prompt='You repair a robot policy graph from real execution feedback. Return ONLY a JSON object with reason, insert_before="carry", nodes=[{id,xyz:[x,y,z],mode:"cartesian" or "free"}]. Insert 1-6 motions before carry. Do not change existing nodes or goal evaluation. World frame meters. Table top z=.15. Barrier center [.52,0,.265], size [.32,.065,.23], top z=.38. Cube size .04. Downward Panda gripper. Respect whole-arm collision checks; choose clearance for carried object. Allowed xyz: x .2-.75, y -.45-.45, z .20-.65. Graph and real feedback follow.'
 with httpx.Client(timeout=90) as c:
  res=c.post(base.rstrip('/')+'/chat/completions',headers={'Authorization':'Bearer '+key},json={'model':model,'messages':[{'role':'system','content':prompt},{'role':'user','content':json.dumps({'graph':graph,'feedback':feedback})}],'temperature':.2,'response_format':{'type':'json_object'}})
 if res.status_code!=200:raise RuntimeError(f'LLM request failed: HTTP {res.status_code}')
 raw=res.json();event('llm_response',model=model,usage=raw.get('usage'));return json.loads(raw['choices'][0]['message']['content'])

def run(provider,max_rounds):
 run_id=time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns()%1000000);directory=OUT/run_id;directory.mkdir()
 with lock:state.update(running=True,phase='executing',round=0,provider=provider,versions=[],events=[],error=None,run_id=run_id,node_status={})
 graph=initial_graph();cache={};last_node=None;executor=None
 try:
  with httpx.Client(trust_env=False) as c:c.post('http://127.0.0.1:9431/resume',json={})
  observed=ros('observe')
  if observed['attached'] or math.dist(observed['cube'],[.5,-.24,.17])>.04:raise RuntimeError('请先重置实验场景：方块需要位于起始区且未被夹持。可停止并重新启动 launch.py。')
  event('run_started',provider=provider,resume_semantics='successful prefix cached; same physical episode, no hidden reset')
  for iteration in range(max_rounds):
   if stop_event.is_set():raise RuntimeError('CANCELLED')
   d=directory/f'v{iteration}';d.mkdir();(d/'workflow.json').write_text(json.dumps(graph,ensure_ascii=False,indent=2))
   version={'index':iteration,'hash':hashlib.sha256(json.dumps(graph,sort_keys=True).encode()).hexdigest()[:12],'graph':copy.deepcopy(graph),'status':'running','feedback':None}
   with lock:state['round']=iteration;state['versions'].append(version);state['phase']='executing';state['node_status']={}
   registry=ToolRegistry()
   def dispatch(op,**args):
    nonlocal last_node
    if stop_event.is_set():raise RuntimeError('CANCELLED')
    name=last_node;signature=json.dumps({'op':op,'args':args},sort_keys=True)
    if name in cache and cache[name][0]==signature:
     with lock:state['node_status'][name]='cached'
     event('node_cached',node=name);return cache[name][1]
    event('skill_input',node=name,op=op,inputs=args)
    value=ros(op,args);cache[name]=(signature,value);event('skill_output',node=name,outputs=value);return value
   # Explicit signatures let official GaP validate tool inputs.
   def observe()->dict:return dispatch('observe')
   def move(xyz:list[float],mode:str='cartesian')->dict:return dispatch('move',xyz=xyz,mode=mode)
   def gripper(close:bool)->dict:return dispatch('gripper',close=close)
   def verify()->dict:return dispatch('verify')
   for name,fn in [('observe',observe),('move',move),('gripper',gripper),('verify',verify)]:registry.register_callable('gazebo.'+name,fn,summary='Gazebo '+name)
   wf=load_workflow(d/'workflow.json');issues=validate_workflow(wf,tool_registry=registry)
   errs=[str(i) for i in issues if i.severity=='error']
   if errs:raise RuntimeError('Graph validation failed: '+'; '.join(errs))
   executor=WorkflowExecutor(d,tool_registry=registry,trace_dir=d/'trace',max_node_workers=1,node_visit_cap=80)
   def on_start(name,_):
    nonlocal last_node
    last_node=name
    with lock:state['node_status'][name]='running'
    event('node_start',node=name)
   def on_end(name,success):
    with lock:
     if state['node_status'].get(name)!='cached':state['node_status'][name]='success' if success else 'failure'
    event('node_end',node=name,success=success)
   executor.trace.on_node_start=on_start;executor.trace.on_node_end=on_end
   try:
    executor.execute()
    if executor.exit_status!='success':raise RuntimeError('Graph ended without success')
    # Independent, immutable ground-truth judge is also invoked outside the graph.
    final=ros('verify');version['status']='success';version['node_status']=dict(state['node_status']);event('verified_success',result=final)
    with lock:state['phase']='success'
    break
   except Exception as e:
    if stop_event.is_set():raise RuntimeError('CANCELLED') from e
    version['status']='failure';version['node_status']=dict(state['node_status']);feedback={'node':last_node,'error':str(e),'world':ros('observe')};version['feedback']=feedback
    (d/'feedback.json').write_text(json.dumps(feedback,ensure_ascii=False,indent=2));event('execution_failed',**feedback)
    if iteration+1>=max_rounds:raise RuntimeError('Repair budget exhausted: '+str(e))
    with lock:state['phase']='repairing'
    patch=propose(graph,feedback,provider);candidate=validate_patch(graph,patch,last_node)
    (d/'repair.json').write_text(json.dumps(patch,ensure_ascii=False,indent=2));version['patch']=patch;event('graph_repaired',patch=patch,source=provider)
    graph=candidate;time.sleep(1.)
   finally:executor.close()
 except Exception as e:
  traceback.print_exc();event('run_failed',error=str(e))
  with lock:state['phase']='stopped' if stop_event.is_set() else 'failure';state['error']=str(e)
 finally:
  with lock:state['running']=False;(directory/'summary.json').write_text(json.dumps(state,ensure_ascii=False,indent=2))

class RunRequest(BaseModel):
 provider:str='rules'
 max_rounds:int=3

@app.get('/api/state')
def get_state():
 with lock:s=copy.deepcopy(state)
 s['llm_configured']=bool(os.environ.get('GAP_LLM_API_KEY') and os.environ.get('GAP_LLM_MODEL'))
 s['llm_model']=os.environ.get('GAP_LLM_MODEL','')
 try:
  with httpx.Client(timeout=2,trust_env=False) as c:s['simulation']=c.get('http://127.0.0.1:9431/state').json()
 except Exception:s['simulation']={'fresh':False}
 return s

@app.post('/api/run')
def start(req:RunRequest):
 with lock:
  if state['running']:raise HTTPException(409,'Experiment already running')
  if req.provider not in ('rules','llm') or not 1<=req.max_rounds<=5:raise HTTPException(422,'Invalid provider or budget')
  if req.provider=='llm' and not (os.environ.get('GAP_LLM_API_KEY') and os.environ.get('GAP_LLM_MODEL')):raise HTTPException(422,'LLM is not configured')
  stop_event.clear();state['running']=True
  threading.Thread(target=run,args=(req.provider,req.max_rounds),daemon=True).start()
 return {'started':True}

@app.post('/api/stop')
def stop():
 stop_event.set()
 try:
  with httpx.Client(timeout=3,trust_env=False) as c:c.post('http://127.0.0.1:9431/stop',json={})
 except Exception:pass
 return {'stopping':True}

@app.post('/api/reset')
def reset():
 with lock:
  if state['running']:raise HTTPException(409,'Stop the experiment before resetting')
  state['running']=True;state['phase']='resetting'
 try:
  value=ros('reset')
  with lock:state.update(phase='idle',error=None)
  return value
 except Exception as e:
  with lock:state.update(phase='failure',error=str(e))
  raise HTTPException(422,str(e))
 finally:
  with lock:state['running']=False

@app.get('/camera.jpg')
def camera():
 try:
  with httpx.Client(timeout=2,trust_env=False) as c:r=c.get('http://127.0.0.1:9431/camera.jpg')
  return Response(r.content,media_type='image/jpeg',headers={'Cache-Control':'no-store'})
 except Exception:raise HTTPException(503,'Camera unavailable')

@app.get('/api/initial-graph')
def graph():return initial_graph()
@app.get('/')
def index():return FileResponse(R/'web/index.html')
app.mount('/assets',StaticFiles(directory=R/'web'),name='assets')
if __name__=='__main__':
 import uvicorn
 uvicorn.run(app,host='127.0.0.1',port=9433,log_level='warning')
