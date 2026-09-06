#!/usr/bin/env python3
"""Check the pinned public GaP core; this does not run robot simulation.

Run with .venv-audit/bin/python. Upstream tests use fixtures/mocked model
responses. No live code-generation or perception model is invoked here.
"""
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT / "graph-as-policy"
SKILLS = ROOT / "open-robot-skills"


def main():
    for folder in ("logs", "reports", "outputs"):
        (ROOT / folder).mkdir(exist_ok=True)
    env = dict(os.environ, PYTHONPATH=str(REPO),
               MPLCONFIGDIR=str(ROOT / ".cache/matplotlib"))
    checks = [
        ("core-tests", [sys.executable, "-m", "pytest", "tests/runtime",
                        "tests/builder", "tests/tools", "tests/skills", "tests/agent",
                        "-q", "--maxfail=8", "--junitxml=" + str(ROOT / "reports/core-tests.xml")]),
        ("build-graph", [sys.executable, "examples/build_a_graph/build_graph.py",
                         "--out", str(ROOT / "outputs/authored_graph"),
                         "--skills", str(SKILLS)]),
        ("skills-check", [sys.executable, "-c", "from gap.cli import main; main()",
                          "skills", "check", "--skills", str(SKILLS)]),
    ]
    results = []
    for name, command in checks:
        logfile = ROOT / "logs" / (name + ".log")
        with logfile.open("w") as stream:
            result = subprocess.run(command, cwd=REPO, env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
        results.append({"check": name, "command": command, "exit_code": result.returncode,
                        "log": str(logfile.relative_to(ROOT))})
        print(f"{name}: exit {result.returncode}; {logfile}", flush=True)
    xml = ROOT / "reports/core-tests.xml"
    junit = [dict(x.attrib) for x in ElementTree.parse(xml).getroot().iter("testsuite")] if xml.exists() else []
    report = {"checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "offline core tests, authored graph validation and bundle metadata/import checks",
              "excludes": ["live LLM generation", "model inference", "physical robot simulation",
                           "paper rehearsal/self-learning", "paper benchmark success rates"],
              "checks": results, "junit": junit}
    (ROOT / "reports/core-checks.json").write_text(json.dumps(report, indent=2) + "\n")
    return int(any(r["exit_code"] for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
