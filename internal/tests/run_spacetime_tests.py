"""Compile and run the production spacetime planner without the game or injector."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import sys
import re

internal = Path(__file__).resolve().parents[1]
# A registered group is useless unless the public dispatcher calls it. This
# also guards the Spacetime and Target Assist groups added by this port.
registry = (internal / "src/features/control/FeatureCommandRegistry.cpp").read_text(encoding="utf-8")
groups = set(re.findall(r"bool (Apply\w+Feature)\(const FeatureCommand& f\)", registry))
dispatch = registry[registry.index("bool Apply(const FeatureCommand& feature)"):]
unreachable = sorted(group for group in groups if not re.search(r"\b" + group + r"\(feature\)", dispatch))
if unreachable:
    raise AssertionError(f"Unreachable feature command groups: {unreachable}")
print(f"Feature dispatch: {len(groups)} handler groups reachable", flush=True)
sources = [internal / "src/features/movement/udodge/UDodgeCore.cpp",
           internal / "src/features/movement/spacetime/SpacetimeCore.cpp",
           internal / "src/features/movement/spacetime/SpacetimeMovement.cpp",
           internal / "tests/spacetime_tests.cpp"]
env = os.environ.copy()
if os.name == "nt" and not shutil.which("cl"):
    vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio/Installer/vswhere.exe"
    install = subprocess.check_output([str(vswhere), "-latest", "-products", "*",
                                       "-property", "installationPath"], text=True).strip()
    vcvars = Path(install) / "VC/Auxiliary/Build/vcvars64.bat"
    output = subprocess.check_output(f'cmd /d /s /c ""{vcvars}" >nul && set"', text=True)
    for line in output.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            env[key.upper()] = value

with tempfile.TemporaryDirectory(prefix="spacetime-tests-") as directory:
    build = Path(directory)
    (build / "pch-il2cpp.h").write_text("#define NOMINMAX\n#include <windows.h>\n" if os.name == "nt" else
        "#include <chrono>\nstruct LARGE_INTEGER { long long QuadPart; };\n"
        "inline void QueryPerformanceFrequency(LARGE_INTEGER* p) { p->QuadPart=1000000000; }\n"
        "inline void QueryPerformanceCounter(LARGE_INTEGER* p) { p->QuadPart=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }\n")
    binary = build / ("spacetime-tests.exe" if os.name == "nt" else "spacetime-tests")
    if os.name == "nt":
        compiler = shutil.which("cl.exe", path=env.get("PATH"))
        command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/O2", "/W4",
                   "/I" + str(build), *map(str, sources), "/Fe:" + str(binary)]
    else:
        command = [os.environ.get("CXX", "c++"), "-std=c++17", "-O2", "-Wall", "-Wextra",
                   "-I", str(build), *map(str, sources), "-o", str(binary)]
    subprocess.run(command, cwd=build, env=env, check=True)
    preview = os.environ.get("RE_SPACETIME_GRID_PREVIEW")
    if "--replay" in sys.argv:
        subprocess.run([str(binary), *sys.argv[sys.argv.index("--replay"):]], check=True)
    else:
        subprocess.run([str(binary), *([preview] if preview else [])], check=True)
    if "--all" in sys.argv:
        core=internal / "src/features/movement/udodge"
        for name in ("udodge_zone_tests", "udodge_temporal_tests", "udodge_admission_tests",
                     "udodge_speed_expiry_tests", "udodge_commitment_tests", "udodge_navigation_tests"):
            files=[core / "UDodgeCore.cpp",core / "UDodgeSolver.cpp",internal / f"tests/{name}.cpp"]
            if name=="udodge_commitment_tests": files.append(core / "UDodgeWorker.cpp")
            if name=="udodge_navigation_tests": files.append(core / "UDodgePathfinder.cpp")
            test_binary=build / (name+(".exe" if os.name=="nt" else ""))
            if os.name=="nt":
                test_command=[compiler,"/nologo","/std:c++17","/EHsc","/O2","/W3",
                    "/I"+str(build),"/I"+str(core),"/I"+str(internal / "src"),*map(str,files),"/Fe:"+str(test_binary)]
            else:
                test_command=[os.environ.get("CXX","c++"),"-std=c++17","-O2","-pthread",
                    "-I",str(build),"-I",str(core),"-I",str(internal / "src"),*map(str,files),"-o",str(test_binary)]
            subprocess.run(test_command,cwd=build,env=env,check=True)
            subprocess.run([str(test_binary)],check=True)
