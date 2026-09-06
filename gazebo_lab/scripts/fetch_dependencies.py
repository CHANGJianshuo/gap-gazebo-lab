#!/usr/bin/env python3
"""Fetch exact source commits from config/dependencies.json (no package upgrades)."""
import json,subprocess
from pathlib import Path
R=Path(__file__).resolve().parents[1]
for name,item in json.loads((R/'config/dependencies.json').read_text()).items():
 p=(R/item['path']).resolve()
 if not (p/'.git').exists():
  p.parent.mkdir(parents=True,exist_ok=True)
  subprocess.run(['git','clone',item['url'],str(p)],check=True)
 current=subprocess.check_output(['git','rev-parse','HEAD'],cwd=p,text=True).strip()
 if current!=item['commit']:
  if subprocess.check_output(['git','status','--porcelain'],cwd=p,text=True).strip():raise SystemExit(f'{name}: local changes; refusing to switch checkout')
  subprocess.run(['git','fetch','origin',item['commit']],cwd=p,check=True)
  subprocess.run(['git','checkout','--detach',item['commit']],cwd=p,check=True)
 print(name,item['commit'])
