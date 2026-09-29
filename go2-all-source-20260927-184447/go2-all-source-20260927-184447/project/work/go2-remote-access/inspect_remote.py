"""Проверка удалённого окружения без подключения к роботу и без команд движения."""
import ast
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
from pathlib import Path

root = Path.home() / "ai-robot"
report = {
    "hostname": socket.gethostname(),
    "platform": platform.platform(),
    "python": sys.version.split()[0],
    "workspace": str(root),
    "workspace_exists": root.is_dir(),
    "ros2_in_path": shutil.which("ros2"),
    "ros_installations": [p.name for p in Path("/opt/ros").glob("*")],
}
if root.is_dir():
    report["files"] = [
        {"name": p.name, "directory": p.is_dir(), "bytes": p.stat().st_size if p.is_file() else None}
        for p in sorted(root.iterdir())
        if not p.name.startswith(".")
    ]
    report["python_interfaces"] = {}
    for name in ("go2.py", "01_read_state.py", "02_battery.py", "03_command.py", "ros_bridge.py"):
        path = root / name
        if not path.is_file():
            continue
        try:
            tree = ast.parse(path.read_text())
            report["python_interfaces"][name] = {
                "imports": [ast.unparse(n) for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))],
                "functions": [
                    {"name": n.name, "line": n.lineno, "async": isinstance(n, ast.AsyncFunctionDef)}
                    for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                ],
            }
        except Exception as exc:
            report["python_interfaces"][name] = {"error": type(exc).__name__}
    result = subprocess.run([str(root / "venv/bin/python"), "-c",
        "import importlib.metadata,json; names=['unitree-webrtc-connect','go2-webrtc-connect','aiortc','numpy','opencv-python','av','rclpy']; d={}; "
        "exec('for n in names:\\n try: d[n]=importlib.metadata.version(n)\\n except importlib.metadata.PackageNotFoundError: pass'); print(json.dumps(d))"],
        capture_output=True, text=True, timeout=12)
    report["packages"] = result.stdout.strip()
    if result.returncode:
        report["package_check_error"] = result.stderr[-1200:]
active = []
for path in Path("/proc").glob("[0-9]*/cmdline"):
    try:
        args = path.read_bytes().split(b"\0")
        scripts = [Path(a.decode(errors="replace")).name for a in args[1:3] if a.endswith(b".py")]
        if scripts:
            active.append({"pid": int(path.parent.name), "scripts": scripts})
    except (OSError, ValueError):
        pass
report["active_python_scripts"] = active
print(json.dumps(report, ensure_ascii=False, indent=2))
