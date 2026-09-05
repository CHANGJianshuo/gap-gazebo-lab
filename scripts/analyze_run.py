#!/usr/bin/env python3
"""Create exportable plots from the actual controller samples of one run."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

JOINTS=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
def main():
    p=argparse.ArgumentParser();p.add_argument('run');args=p.parse_args();run=Path(args.run)
    rows=list(csv.DictReader((run/'tracking.csv').open()));summary=json.loads((run/'summary.json').read_text());out=run/'figures';out.mkdir(exist_ok=True)
    t=np.array([float(r['sim_time']) for r in rows]);t-=t[0]
    q=np.array([[float(r['q_'+j]) for j in JOINTS] for r in rows]);ref=np.array([[float(r['reference_'+j]) for j in JOINTS] for r in rows]);error=np.array([[float(r['error_'+j]) for j in JOINTS] for r in rows]);effort=np.array([[float(r['command_effort_'+j]) for j in JOINTS] for r in rows])
    colors=['#245cff','#15977d','#e69f00','#ad55c4','#e05760','#64748b']
    fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True,gridspec_kw={'height_ratios':[1.2,1,1]})
    cuts=np.where(np.diff(t)>.06)[0]+1;segments=np.split(np.arange(len(t)),cuts)
    for j in range(6):
        for k,indices in enumerate(segments):
            label=f'J{j+1}' if k==0 else None
            axes[0].plot(t[indices],q[indices,j],color=colors[j],lw=1.3,label=label)
            axes[0].plot(t[indices],ref[indices,j],color=colors[j],lw=.7,ls='--',alpha=.65)
            axes[1].plot(t[indices],error[indices,j]*1000,color=colors[j],lw=1.)
            axes[2].plot(t[indices],np.abs(effort[indices,j])/[49,49,39,9.8,9.8,9.8][j],color=colors[j],lw=1.)
    axes[0].set_ylabel('Joint angle [rad]');axes[1].set_ylabel('Tracking error [mrad]');axes[2].set_ylabel('|command torque| / limit');axes[2].set_xlabel('Simulation time since first motion sample [s]');axes[2].axhline(1,color='#c33',ls='--',lw=1);axes[2].set_ylim(0,1.06)
    axes[0].legend(ncol=6,loc='upper center',bbox_to_anchor=(.5,1.30),frameon=False)
    for ax in axes:ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False)
    fig.suptitle(f"AUBO S3 / {summary['task']} / actual Gazebo controller feedback",x=.07,ha='left',fontsize=14,y=.995)
    fig.text(.07,.006,'Solid: measured joints. Dashed: controller reference. Gaps: planning / dwell periods, not sampled by the tracking logger.',fontsize=9,color='#657286')
    fig.tight_layout(rect=(0,.025,1,.97));fig.savefig(out/'tracking.svg');fig.savefig(out/'tracking.png',dpi=150);plt.close(fig)
    checks={'source':str(run/'tracking.csv'),'sample_count':len(rows),'max_tracking_error_rad':float(np.abs(error).max()),'rms_tracking_error_rad':float(np.sqrt(np.mean(error**2))),'maximum_command_torque_ratio':float(np.nanmax(np.abs(effort)/np.array([49,49,39,9.8,9.8,9.8]))),'matches_summary':bool(abs(float(np.abs(error).max())-summary['max_tracking_error_rad'])<1e-12)}
    trace=json.loads((run/'ground_truth_trace.json').read_text());cartesian=[]
    for plan in summary['plans']:
        if plan['mode']!='cartesian':continue
        points=np.array([r['tcp'] for r in trace if r['phase']==plan['label']+' · 执行'])
        if len(points)<2:continue
        target=np.array([plan['target']['pose']['position'][k] for k in ['x','y','z']]);start=points[0];direction=target-start;denominator=float(direction@direction)
        progress=np.clip((points-start)@direction/denominator,0,1) if denominator>1e-12 else np.zeros(len(points));nearest=start+progress[:,None]*direction
        cartesian.append({'label':plan['label'],'max_actual_line_deviation_m':float(np.linalg.norm(points-nearest,axis=1).max()),'actual_endpoint_error_m':float(np.linalg.norm(points[-1]-target))})
    checks['actual_cartesian_segments']=cartesian
    holds=[r for r in trace if r['phase']=='保持举起 2 秒']
    if holds:
        letter=summary['demo_order'][0];positions=np.array([r['objects']['battery_'+letter] for r in holds]);events=json.loads((run/'events.json').read_text());event=next(i for i,e in enumerate(events) if e['phase']=='保持举起 2 秒')
        checks['lift_hold']={'commanded_duration_sim_s':events[event+1]['sim_time']-events[event]['sim_time'],'min_battery_bottom_height_above_table_m':float(positions[:,2].min()-.65),'max_position_drift_m':float(np.linalg.norm(positions-positions[0],axis=1).max())}
    (out/'checks.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks))
if __name__=='__main__':main()
