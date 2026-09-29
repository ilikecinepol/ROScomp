"""Offline structural checks for the arena when Webots is not installed."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
FILES = sorted(ROOT.glob("protos/*.proto")) + sorted(ROOT.glob("worlds/*.wbt"))


def without_comments_and_strings(text: str) -> str:
    text = re.sub(r"#.*", "", text)
    return re.sub(r'"(?:\\.|[^"\\])*"', '""', text)


def check_balanced(path: Path, text: str) -> list[str]:
    errors: list[str] = []
    pairs = {"}": "{", "]": "[", ")": "("}
    stack: list[tuple[str, int]] = []
    for line_no, line in enumerate(without_comments_and_strings(text).splitlines(), 1):
        for char in line:
            if char in "{[(":
                stack.append((char, line_no))
            elif char in "}])":
                if not stack or stack[-1][0] != pairs[char]:
                    errors.append(f"{path}: unmatched {char!r} at line {line_no}")
                    continue
                stack.pop()
    for char, line_no in stack:
        errors.append(f"{path}: unclosed {char!r} from line {line_no}")
    return errors


def main() -> int:
    errors: list[str] = []
    for path in FILES:
        text = path.read_text(encoding="utf-8")
        if not text.startswith("#VRML_SIM R2025a utf8"):
            errors.append(f"{path}: missing R2025a header")
        errors.extend(check_balanced(path, text))
        for relative in re.findall(r'EXTERNPROTO\s+"([^"]+)"', text):
            target = (path.parent / relative).resolve()
            if not target.exists():
                errors.append(f"{path}: missing EXTERNPROTO target {relative}")

    world = (ROOT / "worlds/truetech_arena.wbt").read_text(encoding="utf-8")
    arena = (ROOT / "protos/TrueTechArena.proto").read_text(encoding="utf-8")
    teeter = (ROOT / "protos/Teeter.proto").read_text(encoding="utf-8")
    bridge = (ROOT / "protos/SwingBridge.proto").read_text(encoding="utf-8")
    go2 = (ROOT / "protos/Go2.proto").read_text(encoding="utf-8")

    required = {
        "ENU coordinate system": 'coordinateSystem "ENU"' in world,
        "8 ms physics step": "basicTimeStep 8" in world,
        "six slalom poles": arena.count("SlalomPole {") == 6,
        "teeter hinge": "HingeJoint {" in teeter and "DEF TEETER_BOARD" in teeter,
        "bridge hinge": "HingeJoint {" in bridge and "DEF BRIDGE_DECK" in bridge,
        "teeter ±12 degree stops": "minStop -0.209440" in teeter and "maxStop 0.209440" in teeter,
        "teeter passive dynamic board": "mass 6" in teeter and "RotationalMotor" not in teeter,
        "upright slalom geometry": "rotation 1 0 0 1.570796" not in (ROOT / "protos/SlalomPole.proto").read_text(encoding="utf-8"),
        "bridge ±8 degree stops": "minStop -0.139626" in bridge and "maxStop 0.139626" in bridge,
        "Go2 present in arena": 'DEF GO2 Go2 {' in world,
        "Go2 has 12 motors": go2.count("RotationalMotor {") == 12,
        "Go2 mesh paths are portable": "C:/Users/" not in go2 and "../robots/go2_robot_sdk/dae/" in go2,
        "Go2 arena virtual sport": 'controller "go2_virtual_sport"' in world,
    }
    errors.extend(f"semantic check failed: {name}" for name, ok in required.items() if not ok)

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Validated {len(FILES)} VRML/Webots files and arena invariants.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
