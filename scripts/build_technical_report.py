#!/usr/bin/env python3
"""Render the Chinese engineering report from its Markdown and final run evidence.

Uses the documentation Python environment, not the ROS launch environment.
Dependencies: markdown, matplotlib; --pdf also uses Playwright/Chromium, PyMuPDF.
Does not run the robot or modify existing experiment records.
"""
import argparse
import base64
import collections
import hashlib
import html
import json
import math
import mimetypes
import re
import unicodedata
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'docs/reports'
ASSETS = REPORT / 'assets'
MD = REPORT / 'motion_planning_report.md'


def read_json(path):
    return json.loads(path.read_text())


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |',
                      '| ' + ' | '.join(['---'] * len(headers)) + ' |'] +
                     ['| ' + ' | '.join(map(str, row)) + ' |' for row in rows])


def inject(text, name, value):
    pattern = rf'(<!-- BEGIN {name} -->).*?(<!-- END {name} -->)'
    result, count = re.subn(pattern, lambda m: m[1] + '\n\n' + value + '\n\n' + m[2],
                            text, flags=re.S)
    if count != 1:
        raise ValueError(f'Expected one generated block {name}, got {count}')
    return result


def planning_diagram():
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="550" viewBox="0 0 1080 550" role="img" aria-labelledby="title desc">
<title id="title">AUBO S3 候选生成与验证</title><desc id="desc">目标与场景输入后，自由运动比较逆解和直接轨迹，直线运动通过连续 IK 生成样条。自由运动候选全失败时启用 RRT。所有候选经过统一检查，由执行层发送成功轨迹。</desc>
<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8" fill="none" stroke="#76849a" stroke-width="1.5"/></marker></defs>
<style>text{font-family:system-ui,'Noto Sans CJK SC','Microsoft YaHei',sans-serif;fill:#1a2940;text-anchor:middle}.title{font-size:19px;font-weight:700}.body{font-size:16px}.small{font-size:14px;fill:#62738a}.line{fill:none;stroke:#76849a;stroke-width:2;marker-end:url(#arrow)}</style>
<rect width="1080" height="550" rx="14" fill="#f6f8fc"/>
<rect x="300" y="16" width="480" height="58" rx="12" fill="#e6edff" stroke="#91aafb"/>
<text x="540" y="40" class="title">目标 TCP + 当前关节 + 场景 + 负载</text><text x="540" y="62" class="small">统一坐标、检查数据和起点有效性</text>
<path d="M390 74 V92 H180 V118" class="line"/><path d="M690 74 V92 H900 V118" class="line"/>
<rect x="22" y="123" width="316" height="126" rx="12" fill="white" stroke="#ccd6e7"/>
<text x="180" y="152" class="title">free：自由运动</text><text x="180" y="180" class="body">多个目标关节姿态 → Ruckig</text><text x="180" y="207" class="body">带载或全失败时另试直线样条</text><text x="180" y="232" class="small">比较整条轨迹，不能只看 IK 终点</text>
<rect x="742" y="123" width="316" height="126" rx="12" fill="white" stroke="#ccd6e7"/>
<text x="900" y="152" class="title">cartesian：末端直线</text><text x="900" y="180" class="body">位姿插值 → 连续 IK</text><text x="900" y="207" class="body">五次 B 样条：10 / 16 / 24 点</text><text x="900" y="232" class="small">任一路点失败或关节跳变则拒绝</text>
<path d="M338 177 H410" class="line"/><text x="376" y="152" class="small">全失败</text>
<rect x="420" y="130" width="240" height="108" rx="12" fill="#fff5df" stroke="#dbc08a"/>
<text x="540" y="159" class="title">RRT-Connect 后备</text><text x="540" y="186" class="body">绕障路径 → 样条候选</text><text x="540" y="215" class="small">最终录像没有触发此分支</text>
<path d="M180 249 V287 H310 V317" class="line"/><path d="M900 249 V287 H770 V317" class="line"/><path d="M540 238 V317" class="line"/>
<rect x="132" y="322" width="816" height="95" rx="12" fill="#eaf6f1" stroke="#8ec9b2"/>
<text x="540" y="350" class="title">统一验证：控制器插值、碰撞、导数、力矩、负载与直线约束</text>
<text x="540" y="379" class="body">候选粗检查 → 按时长排序 → 更密的最终检查</text>
<text x="540" y="403" class="small">选最短通过者；否则返回失败原因</text>
<path d="M540 417 V447" class="line"/>
<rect x="160" y="452" width="760" height="66" rx="12" fill="white" stroke="#9fb4d8"/>
<text x="540" y="479" class="title">输出带时间的关节轨迹 + 状态 + 验证指标</text><text x="540" y="505" class="body">SJM / 演示客户端发起执行 → 控制器 → Gazebo → 反馈与验收</text>
</svg>'''
    (ASSETS / 'planning_flow.svg').write_text(svg)


def duration_plot(seq):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font_path = ROOT / 'third_party/fonts/NotoSansCJKsc-Regular.otf'
    font = FontProperties(fname=str(font_path))
    labels = ['到把手上方', '垂直接近', '带载抬升', '水平转移', '垂直放下', '打开后撤离']
    colors = ['#315ce8', '#7d9bff', '#147f70', '#74b8a5', '#c08b2b', '#e1c383']
    fig, ax = plt.subplots(figsize=(10.5, 3.9))
    for i, letter in enumerate('CAB'):
        start = 0
        for j in range(6):
            duration = seq['plans'][i * 6 + j]['duration']
            ax.barh(i, duration, left=start, color=colors[j], height=.5,
                    label=labels[j] if i == 0 else None, edgecolor='white', linewidth=1)
            ax.text(start + duration / 2, i, f'{duration:.2f}', ha='center', va='center',
                    color='white' if j in [0, 2] else '#132638', fontsize=10)
            start += duration
        ax.text(start + .08, i, f'{start:.3f} s', va='center', fontsize=10, color='#26354a')
    ax.set_yticks([0, 1, 2], ['C → P1', 'A → P2', 'B → P3'])
    ax.invert_yaxis()
    ax.set_xlim(0, 7.15)
    ax.set_xlabel('机械臂轨迹累计时间 / 仿真秒（不含锁扣与等待）', fontproperties=font, labelpad=12)
    ax.set_title('序列 CAB：六段动作的时间组成', fontproperties=font, fontsize=14, loc='left', pad=18)
    ax.legend(ncol=3, prop=font, frameon=False, loc='upper left', bbox_to_anchor=(0, -.35))
    ax.spines[['top', 'right', 'left']].set_visible(False)
    ax.grid(axis='x', alpha=.15)
    ax.set_axisbelow(True)
    ax.tick_params(axis='y', length=0, pad=10)
    fig.tight_layout()
    fig.savefig(ASSETS / 'motion_duration.svg', bbox_inches='tight')
    fig.savefig(ASSETS / 'motion_duration.png', dpi=170, bbox_inches='tight')
    plt.close(fig)


def extract_tables():
    runs = {name: read_json(ROOT / 'data/runs' / name / 'summary.json')
            for name in ['sequence_demo', 'basic_demo_final']}
    seq, basic = runs.values()
    if not (seq['success'] and basic['success']):
        raise ValueError('The report expects the explicitly named final successful runs')
    rows = []
    metrics = [
        ('完成抓放 / 机械臂轨迹段数', lambda s: f"{len(s['placements'])} / {len(s['plans'])}"),
        ('完整流程 / 仿真秒', lambda s: f"{s['simulation_duration_s']:.3f}"),
        ('机械臂纯运动 / 仿真秒', lambda s: f"{s['total_planned_motion_s']:.3f}"),
        ('规划计算累计 / 墙钟秒', lambda s: f"{s['total_planning_wall_s']:.3f}"),
        ('demo 记录耗时 / 墙钟秒', lambda s: f"{s['wall_duration_s']:.3f}"),
        ('最大关节跟踪误差 / rad', lambda s: f"{s['max_tracking_error_rad']:.8f}"),
        ('最大关节跟踪误差 / °', lambda s: f"{math.degrees(s['max_tracking_error_rad']):.6f}"),
        ('关节跟踪 RMS / rad', lambda s: f"{s['rms_tracking_error_rad']:.9f}"),
        ('记录的控制反馈样本数', lambda s: str(s['controller_samples'])),
        ('最大输出力矩命令 / URDF 限值', lambda s: f"{s['max_command_effort_ratio'] * 100:.3f}%"),
        ('非预期接触对数', lambda s: str(s['unexpected_contact_pairs'])),
    ]
    for label, formatter in metrics:
        rows.append([label, formatter(seq), formatter(basic)])
    result = {'METRICS_TABLE': table(['指标', '序列 CAB → P1 / P2 / P3', '基础 C → T0'], rows)}
    placements = []
    for name, s in runs.items():
        for p in s['placements']:
            placements.append(['序列' if name == 'sequence_demo' else '基础',
                               p['object'].replace('battery_', ''), p['target'],
                               f"{1000 * p['xy_error_m']:.5f}",
                               f"{1000 * p['drift_over_3s_m']:.5f}"])
    result['PLACEMENTS_TABLE'] = table(['场景', '电池', '目标', 'XY 误差 / mm', '3 秒漂移 / mm'], placements)
    phase_rows = []
    for i, name in enumerate(['到把手上方', '垂直接近把手', '带载垂直抬升', '搬运到目标上方', '垂直放下', '打开后垂直撤离']):
        values = [seq['plans'][i + 6 * j]['duration'] for j in range(3)]
        phase_rows.append([str(i + 1) + '. ' + name] + [f'{v:.3f}' for v in values])
    phase_rows.append(['纯运动合计'] + [f"{sum(p['duration'] for p in seq['plans'][j * 6:(j + 1) * 6]):.3f}" for j in range(3)])
    result['PHASES_TABLE'] = table(['动作', 'C / 仿真秒', 'A / 仿真秒', 'B / 仿真秒'], phase_rows)
    candidates = seq['plans'][0]['candidates']
    result['EXAMPLE_TABLE'] = table(['候选', 'IK 终点', '轨迹生成', '候选时长 / 仿真秒', '粗检查'],
                                    [[str(i + 1), f'有效姿态 {i + 1}', 'Ruckig', f"{c['duration']:.6f}",
                                      '通过' if c['valid'] else c['reason']] for i, c in enumerate(candidates)])
    facts = {}
    for name, s in runs.items():
        facts[name] = {
            'source': f'data/runs/{name}/summary.json',
            'source_sha256': hashlib.sha256((ROOT / 'data/runs' / name / 'summary.json').read_bytes()).hexdigest(),
            'transfers': len(s['placements']), 'segments': len(s['plans']),
            'candidate_count': sum(len(p['candidates']) for p in s['plans']),
            'candidate_reasons': dict(collections.Counter(c['reason'] for p in s['plans'] for c in p['candidates'])),
            'selected_algorithms': dict(collections.Counter(p['selected'] for p in s['plans'])),
            'simulation_duration_s': s['simulation_duration_s'],
            'total_planned_motion_s': s['total_planned_motion_s'],
            'total_planning_wall_s': s['total_planning_wall_s'],
            'wall_duration_s': s['wall_duration_s'],
            'max_tracking_error_deg': math.degrees(s['max_tracking_error_rad']),
            'max_placement_error_mm': max(p['xy_error_m'] * 1000 for p in s['placements']),
        }
    facts['scope'] = 'Two final runs only; candidate counts use coarse validation outcomes.'
    (ASSETS / 'report_facts.json').write_text(json.dumps(facts, ensure_ascii=False, indent=2) + '\n')
    return result, seq


def inline_images(body):
    def replace(match):
        path = (REPORT / html.unescape(match[1])).resolve()
        if not path.is_file() or not path.is_relative_to(ROOT):
            raise ValueError(f'Missing report image: {path}')
        mime = mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        data = base64.b64encode(path.read_bytes()).decode('ascii')
        return f'src="data:{mime};base64,{data}"'
    return re.sub(r'src="([^"]+)"', replace, body)


TIME_DEMO = '''<div class="interactive" aria-label="时间缩放知识示意">
<p><strong>动手看：同一条路径改变时长</strong><span class="chip">教学示意 · 不是本次实验曲线</span></p>
<label for="time-scale">运动时间 T：<input id="time-scale" type="range" min="0.5" max="4" step="0.1" value="2"><output id="time-value">2.0 秒</output></label>
<div class="factor-grid"><div>速度倍率<strong id="v-factor">1.00×</strong></div><div>加速度倍率<strong id="a-factor">1.00×</strong></div><div>jerk 倍率<strong id="j-factor">1.00×</strong></div></div>
<svg id="scale-chart" viewBox="0 0 660 185" role="img" aria-label="关节角度随时间的示意曲线"><path d="M40 16V149H639" stroke="#ccd6e4" fill="none"/><path id="q-path" fill="none" stroke="#315ce8" stroke-width="3"/><path id="q-baseline" fill="none" stroke="#91a1b9" stroke-width="2" stroke-dasharray="6 5"/><text x="45" y="179" fill="#69798d" font-size="12">0 秒</text><text x="600" y="179" fill="#69798d" font-size="12">4 秒</text><text x="49" y="27" fill="#69798d" font-size="12">1 rad</text></svg>
<p class="small">倍率以 T = 2 秒为基准。蓝线为当前关节角度，灰色虚线为 2 秒基准；到达终点后保持。将 T 调为 1 秒，速度 / 加速度 / jerk 分别为 2 / 4 / 8 倍。</p>
</div>'''

JS = '''function updateScale(){const input=document.getElementById('time-scale');if(!input)return;const T=Number(input.value),k=2/T;document.getElementById('time-value').textContent=T.toFixed(1)+' 秒';for(const [id,n] of [['v-factor',1],['a-factor',2],['j-factor',3]])document.getElementById(id).textContent=(k**n).toFixed(2)+'×';function curve(D){let path='';for(let i=0;i<=240;i++){const t=4*i/240,u=Math.min(1,t/D),q=10*u**3-15*u**4+6*u**5;path+=(i?'L':'M')+(40+599*i/240).toFixed(2)+','+(149-120*q).toFixed(2);}return path;}document.getElementById('q-path').setAttribute('d',curve(T));document.getElementById('q-baseline').setAttribute('d',curve(2));}document.getElementById('time-scale')?.addEventListener('input',updateScale);updateScale();
const observer=new IntersectionObserver(entries=>{for(const entry of entries)if(entry.isIntersecting){document.querySelectorAll('.contents a').forEach(a=>a.classList.toggle('active',a.hash==='#'+entry.target.id));}},{rootMargin:'-5% 0px -80% 0px'});document.querySelectorAll('article h2').forEach(h=>observer.observe(h));'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pdf', action='store_true')
    args = parser.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    blocks, seq = extract_tables()
    text = MD.read_text()
    for name, value in blocks.items():
        text = inject(text, name, value)
    MD.write_text(text)
    planning_diagram()
    duration_plot(seq)
    converter = markdown.Markdown(extensions=['tables', 'fenced_code', 'attr_list', 'toc'],
                                  extension_configs={'toc': {'toc_depth': '2-2'}})
    body = converter.convert(text).replace('<!-- TIME_SCALING_DEMO -->', TIME_DEMO)
    body = re.sub(r'<table>(.*?)</table>', r'<div class="table-wrap"><table>\1</table></div>', body, flags=re.S)
    body = inline_images(body)
    css = (REPORT / 'report.css').read_text()
    page = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="description" content="常建烁的 AUBO S3 运动规划技术报告：IK、B 样条、Ruckig、动力学、Gazebo 验证与团队接口。"><title>AUBO S3 运动规划技术报告 · 常建烁</title><style>{css}</style></head><body>
<header class="toolbar"><a href="../progress/index.html">← 制作记录与演示</a><span>技术报告 · 2026.09.05</span><div><a href="motion_planning_report.md">Markdown</a><a class="download" href="motion_planning_report.pdf">下载 PDF</a></div></header>
<div class="layout"><aside class="contents" aria-label="章节导航"><p>阅读目录</p>{converter.toc}<a class="back-top" href="#">回到顶部 ↑</a></aside><main><article>{body}</article><footer>常建烁运动规划模块 · 数据来自最终两次 Gazebo 运行 · 图片已嵌入，网页无需联网；视频与源码链接指向项目文件。</footer></main></div><script>{JS}</script></body></html>'''
    target = REPORT / 'motion_planning_report.html'
    target.write_text(page)
    if args.pdf:
        from playwright.sync_api import sync_playwright
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            tab = browser.new_page()
            tab.goto(target.as_uri(), wait_until='networkidle')
            tab.evaluate('document.fonts.ready')
            tab.emulate_media(media='print')
            tab.pdf(path=str(REPORT / 'motion_planning_report.pdf'), format='A4', print_background=True,
                    margin={'top': '15mm', 'right': '15mm', 'bottom': '17mm', 'left': '15mm'},
                    display_header_footer=True, header_template='<div></div>',
                    footer_template='<div style="font-size:8px;color:#758298;width:100%;margin:0 55px;display:flex;justify-content:space-between"><span>AUBO S3 · Motion Planning · 2026-09-05</span><span><span class="pageNumber"></span> / <span class="totalPages"></span></span></div>',
                    outline=True)
            browser.close()
        # Chromium's outline generation varies by version. Add explicit, checked
        # section bookmarks using the actual rendered page text.
        import fitz
        pdf_path = REPORT / 'motion_planning_report.pdf'
        pdf = fitz.open(pdf_path)
        normalized = lambda value: re.sub(r'\s+', '', unicodedata.normalize('NFKC', value))
        pages = [normalized(page.get_text()) for page in pdf]
        outline = []
        for heading in re.findall(r'^## (.+?) \{#[^}]+\}$', text, re.M):
            needle = normalized(heading)
            matches = [i + 1 for i, content in enumerate(pages) if needle in content]
            if not matches:
                raise ValueError(f'Cannot locate PDF section: {heading}')
            outline.append([1, heading, matches[0]])
        pdf.set_toc(outline)
        metadata = pdf.metadata
        metadata.update(title='AUBO S3 电池搬运运动规划技术报告',
                        subject='常建烁运动规划模块：原理、实现、Gazebo 实测与团队接口')
        pdf.set_metadata(metadata)
        temporary = pdf_path.with_suffix('.tmp.pdf')
        pdf.save(temporary, garbage=3, deflate=True)
        pdf.close()
        temporary.replace(pdf_path)
    print(json.dumps({'html': str(target.relative_to(ROOT)), 'markdown_chars': len(text),
                      'pdf': args.pdf, 'sections': len(re.findall(r'^## ', text, re.M))}, ensure_ascii=False))


if __name__ == '__main__':
    main()
