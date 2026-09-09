"""Render the real overlay helpers with ImGui into an offline visual QA PNG."""
from pathlib import Path
import os
import subprocess
import tempfile
import sys
import shutil
from PIL import Image

internal = Path(__file__).resolve().parents[1]
output = Path(sys.argv[1]).resolve()
output.parent.mkdir(parents=True, exist_ok=True)
vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
install = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-property', 'installationPath'], text=True).strip()
vcvars = Path(install) / 'VC/Auxiliary/Build/vcvars64.bat'
env = os.environ.copy()
for line in subprocess.check_output(f'cmd /d /s /c ""{vcvars}" >nul && set"', text=True).splitlines():
    if '=' in line:
        key, value = line.split('=', 1)
        env[key.upper()] = value
with tempfile.TemporaryDirectory(prefix='spacetime-overlay-preview-') as folder:
    folder = Path(folder)
    vendor = internal / 'vendor/imgui'
    binary = folder / 'preview.exe'
    sources = [internal / 'tests/spacetime_debug_preview.cpp'] + [vendor / f'{name}.cpp' for name in ('imgui', 'imgui_draw', 'imgui_tables', 'imgui_widgets')]
    compiler = shutil.which('cl.exe', path=env['PATH'])
    subprocess.run([compiler, '/nologo', '/std:c++17', '/O2', '/EHsc', '/D_CRT_SECURE_NO_WARNINGS', '/I'+str(vendor), *map(str,sources), '/Fe:'+str(binary)], cwd=folder, env=env, check=True)
    ppm = folder / 'preview.ppm'
    subprocess.run([str(binary), str(ppm)], cwd=folder, check=True)
    Image.open(ppm).save(output)
print(output)
