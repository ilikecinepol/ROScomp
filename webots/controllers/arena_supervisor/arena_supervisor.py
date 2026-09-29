"""Minimal health monitor for the True Tech Arena Webots world."""

from controller import Supervisor


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

tracked = {
    "TEETER_BOARD": robot.getFromDef("TEETER_BOARD"),
    "BRIDGE_DECK": robot.getFromDef("BRIDGE_DECK"),
}

missing = [name for name, node in tracked.items() if node is None]
if missing:
    print(f"[arena] warning: missing DEF nodes: {', '.join(missing)}")
else:
    print("[arena] world loaded; teeter and bridge dynamic bodies are available")

counter = 0
while robot.step(timestep) != -1:
    counter += 1
    if counter % max(1, 2000 // timestep) == 0:
        states = []
        for name, node in tracked.items():
            if node is not None:
                z = node.getPosition()[2]
                states.append(f"{name}.z={z:.3f} m")
        if states:
            print("[arena] " + ", ".join(states))
