#!/usr/bin/env python3
"""Build the final, source-backed local handoff from successful run artifacts."""
from collections import Counter
import json
import math
from pathlib import Path
from update_progress import update
ROOT=Path(__file__).resolve().parents[1]

def main():
    runs={}
    for task,name in [('sequence','sequence_demo'),('basic','basic_demo_final')]:
        run=ROOT/'data/runs'/name;s=json.loads((run/'summary.json').read_text());reject=json.loads((run/'rejections.json').read_text());video=json.loads((ROOT/f'data/videos/{name}.json').read_text());probe=json.loads((run/'video.json').read_text())
        assert s['success'] and reject['pass'] and s['contact_monitor_present'] and s['unexpected_contact_pairs']==0
        assert probe['probe']['decode_check']['pass'] and probe['probe']['decode_check']['frames']==video['frames']
        assert abs(probe['probe']['decode_check']['duration_s']-video['frames']/video['fps'])<.002
        assert json.loads((run/'figures/checks.json').read_text())['matches_summary']
        runs[task]={'name':name,'summary':s,'recording':video,'video':probe,'rejections':reject}
    seq=runs['sequence']['summary'];basic=runs['basic']['summary'];max_error=max(seq['max_tracking_error_rad'],basic['max_tracking_error_rad'])
    rows=[]
    for task,label in [('sequence','序列 CAB → P1/P2/P3'),('basic','基础 C → T0')]:
        s=runs[task]['summary'];rows.append({'task':label,'success':f"{len(s['placements'])} / {len(s['placements'])}",'times':f"{s['total_planned_motion_s']:.2f} / {s['simulation_duration_s']:.2f} s",'compute':f"{s['total_planning_wall_s']:.2f} s",'placement':f"{max(x['xy_error_m'] for x in s['placements'])*1000:.3f} mm"})
    update('最终版本交付完成',f"基础 1 次与序列 3 次抓放全部通过；两个场景的 14 项接口检查通过。视频、CSV、物理接触、候选轨迹和独立验证记录已归档。",stage=5,status='已完成 · 仿真验收通过',metrics={'success':'4 / 4','execution_time':f"{seq['simulation_duration_s']:.2f} s",'tracking_error':f"{math.degrees(max_error):.3f}°",'validation':'零非预期碰撞'},artifacts={'video':'../../data/videos/sequence_demo.mp4','poster':'../../data/videos/sequence_demo_poster.png','caption':f"1280×720 / 25 fps，按仿真时间播放，序列视频 {runs['sequence']['recording']['video_duration_s']:.2f} 秒。右上角为腕部相机；本机完成序列实际耗时 {seq['wall_duration_s']:.1f} 秒。",'links':[{'url':'../../data/videos/sequence_demo.mp4','label':'下载序列视频'},{'url':'../../data/videos/basic_demo_final.mp4','label':'查看基础任务视频'},{'url':'../VALIDATION.md','label':'验收记录'},{'url':'../../data/runs/sequence_demo/summary.json','label':'完整数据'}],'results':rows,'plot':'../../data/runs/sequence_demo/figures/tracking.svg'},assumptions='理想输入；电池质量 0.4 kg。半圆锁扣闭合并对准后建立刚性机械连接，不瞬移物体。相机噪声、实机控制和锁扣强度尚未验证；本页结果仅覆盖所记录的仿真条件。')
    lines=['# 最终仿真验收 · 常建烁运动规划','', '两张原尺寸底图分别运行，基础任务 1 次、序列任务 3 次抓放全部通过。使用 ROS 2 Humble、MoveIt 2、Gazebo Harmonic、官方 AUBO S3、腕部 D435i 与用户 STEP 电池。','', '| 指标 | 序列 CAB → P1/P2/P3 | 基础 C → T0 |','| --- | --- | --- |']
    measures=[('完整流程（仿真秒）','simulation_duration_s',1),('纯机械臂运动（仿真秒）','total_planned_motion_s',1),('规划计算累计（墙钟秒）','total_planning_wall_s',1),('本机运行耗时（墙钟秒）','wall_duration_s',1),('最大关节跟踪误差（rad）','max_tracking_error_rad',1),('关节跟踪 RMS（rad）','rms_tracking_error_rad',1),('最大输出力矩 / 电机限值','max_command_effort_ratio',1),('非预期接触对','unexpected_contact_pairs',1)]
    for label,key,factor in measures:lines.append(f"| {label} | {seq[key]*factor:.6g} | {basic[key]*factor:.6g} |")
    lines+=['','视频保持正常仿真速度，墙钟耗时较长是本机仿真推进速度造成的。完整流程包含开合锁扣、规划期间仿真推进和规定等待；不能把纯运动时长当成整场完成时间。','', '## 落点与稳定性','', '| 场景 / 电池 | 目标 | 平面误差（mm） | 静置 3 秒漂移（mm） |','| --- | --- | --- |']
    for task in ['sequence','basic']:
        for row in runs[task]['summary']['placements']:lines.append(f"| {task} / {row['object']} | {row['target']} | {row['xy_error_m']*1000:.5f} | {row['drift_over_3s_m']*1000:.5f} |")
    tests=json.loads((ROOT/'data/validation/controller_interpolation.json').read_text());dyn=json.loads((ROOT/'data/validation/dynamics.json').read_text());short=json.loads((ROOT/'data/validation/short_moves.txt').read_text().splitlines()[-1]);sensors=json.loads((ROOT/'data/validation/sensors.json').read_text());assert tests['pass'] and dyn['pass'] and short['bad_samples']==0 and sensors['pass']
    actual=json.loads((ROOT/'data/runs/sequence_demo/figures/checks.json').read_text());line_error=max(x['max_actual_line_deviation_m'] for x in actual['actual_cartesian_segments'])
    hold=json.loads((ROOT/'data/runs/basic_demo_final/figures/checks.json').read_text()).get('lift_hold',{})
    lines+=['','## 验证覆盖','',f"- 实际控制器插值：{tests['samples']} 个点，最大位置表示误差 {tests['max_q_error_rad']:.3g} rad。",f"- 短动作数值检查：{short['cases']} 组，0 个超限样本；合并小于 0.5 ms 的相邻时间点后重新约束控制器插值。",f"- 独立动力学：{dyn['cases']} 组，与 PyBullet 最大力矩差 {dyn['max_torque_difference_Nm']:.3g} N·m。",'- 传感器：640×480 RGB、有效 32FC1 深度、内参、IMU 均通过读取检查。', '- 两个场景各 7 项接口检查通过：可达短目标、不可达目标、无效四元数、未知坐标系、越界起点、碰撞起点、未闭合锁扣。',f"- 序列中实际 TCP 的笛卡尔动作最大直线偏差 {line_error*1000:.3f} mm，直接根据 Gazebo 真值计算。",f"- 基础任务举起保持 {hold.get('commanded_duration_sim_s',0):.3f} 仿真秒；电池底面最低高出台面 {hold.get('min_battery_bottom_height_above_table_m',0)*1000:.2f} mm。",'- 物理接触每 1 ms 监测。正常桌面支撑、锁扣与把手接触单列；锁扣接触电池主体和其他非预期接触会导致失败。', '- 两个 MP4 均完成全帧解码，帧数与录制记录一致；网页在 Chromium 桌面/手机下无溢出、无脚本异常且可播放视频。','', '## 文件与复现','']
    for task in ['sequence','basic']:
        name=runs[task]['name'];rec=runs[task]['recording'];lines.append(f"- [{task} 视频](../data/videos/{name}.mp4)：{rec['frames']} 帧，{rec['video_duration_s']:.2f} 秒，补持帧 {rec['duplicated_frames']}；[运行记录](../data/runs/{name}/summary.json)，同目录包含 CSV、全部候选、输入快照、源码/二进制哈希与接触记录。")
    counts=Counter(x['selected'] for x in seq['plans']);lines+=['', '序列选中的算法段数：'+', '.join(f'{k}: {v}' for k,v in counts.items())+'。RRT-Connect 后备已接入，本次原场景录像未触发它；这些记录不能替代复杂障碍基准。','', '```bash','bash scripts/build.sh','bash scripts/run_demo.sh --task sequence --order CAB --verify','bash scripts/run_demo.sh --task basic --order C --verify','```','', '## 结果边界','', '这是理想输入的规划仿真。电池 0.4 kg、摩擦 0.8、相机和锁扣质量/安装、加速度/jerk 均按记录的仿真假设。锁扣是受位置条件约束的刚性连接，不瞬移物体，也没有验证实际材料强度和间隙冲击。理想 RGB-D/IMU 没有模拟真实相机噪声。尚无实机验证。','', '选择的是候选中最快的通过验证轨迹，不宣称带障碍动力学问题的全局最优。FCL 采用最大 4 ms、自适应目标关节增量 0.003 rad 的有限分辨率检查；物理接触记录是另一项独立的实际运行证据。','', '原图内圈直径 80 mm，70×80 mm 电池平放不能完全被其包含。本演示检查中心落点，不声称内圈满分。示例颜色顺序由测试指定，正式调度及状态机仍由 SJM 对接。','', '历史调试发现并修复：控制库版本混用、DDS 发现、腕部振荡、缺失碰撞网格、Humble effort 支持和短动作插值数值问题。早期失败日志保留在其他运行目录；最终验收以本页链接的两次运行为准。','']
    (ROOT/'docs/VALIDATION.md').write_text('\n'.join(lines));(ROOT/'data/validation/final_acceptance.json').write_text(json.dumps({'pass':True,'runs':{k:{'run_id':v['name'],'success':v['summary']['success'],'video_sha256':v['video']['sha256'],'rejections_pass':v['rejections']['pass']} for k,v in runs.items()},'controller_interpolation':tests,'dynamics':dyn,'short_moves':short,'sensors_pass':sensors['pass']},indent=2));print('Final handoff generated from successful artifacts.')

if __name__=='__main__':main()
