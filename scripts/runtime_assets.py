"""Resolve portable model templates into ignored runtime files; no CAD toolchain required."""
import os
from pathlib import Path
import re
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]


def asset_text(relative):
    return (ROOT / relative).read_text().replace('@PROJECT_ROOT@', str(ROOT))


def runtime_asset(relative, resolve_packages=False):
    text = asset_text(relative)
    if resolve_packages:
        def package_path(match):
            package = match.group(1)
            base = ROOT / 'src/mtc_description' if package == 'mtc_description' else Path(get_package_share_directory(package))
            return str(base) + '/'
        text = re.sub(r'package://([^/]+)/', package_path, text)
    output = ROOT / 'data/runtime' / relative
    if resolve_packages:
        output = output.with_name(output.stem + '_resolved' + output.suffix)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + f'.{os.getpid()}.tmp')
    temporary.write_text(text)
    temporary.replace(output)
    return output
