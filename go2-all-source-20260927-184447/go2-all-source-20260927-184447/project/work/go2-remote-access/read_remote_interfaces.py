"""Чтение разрешённых интерфейсов с удалением возможных секретов из вывода."""
import ast
import json
import re
from pathlib import Path

root = Path.home() / "ai-robot"

def redact(text):
    text = re.sub(r"-----BEGIN[\s\S]*?-----END[^\n]*", "[КЛЮЧ СКРЫТ]", text)
    text = re.sub(r"(?im)^([^\n]*(?:PASSWORD|TOKEN|SECRET|AES_KEY|PRIVATE_KEY)\s*=).*$", r"\1 '[СКРЫТО]'", text)
    text = re.sub(r"(?i)(https?://)[^\s/:]+:[^\s/@]+@", r"\1[СКРЫТО]@", text)
    text = re.sub(r"\b[A-Fa-f0-9]{48,}\b", "[ДЛИННОЕ ЗНАЧЕНИЕ СКРЫТО]", text)
    return text

for name in ["go2.py", "01_read_state.py", "02_battery.py", "ROS2.md", "ros_bridge.py", "sitecustomize.py"]:
    path = root / name
    if path.is_file():
        print("\nФАЙЛ", name)
        print(redact(path.read_text())[:16000])
path = root / "fleet-dog.py"
if path.is_file():
    tree = ast.parse(path.read_text())
    print("\nFLEET_DOG_OVERVIEW")
    print(redact(ast.get_docstring(tree) or ""))
    print(json.dumps([{"function": n.name, "line": n.lineno} for n in ast.walk(tree)
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))], ensure_ascii=False))
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in ("main", "run", "heartbeat"):
            print(redact(ast.unparse(node))[:7000])
