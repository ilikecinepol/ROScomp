"""Запись доступных датчиков Go2. Команд движения и смены походки здесь нет."""
import asyncio
import json
import signal
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path.home() / "ai-robot"
sys.path.insert(0, str(ROOT))
import go2
from unitree_webrtc_connect.constants import RTC_TOPIC

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
COUNTS = Counter()
SAMPLES = {}
ERRORS = []
POINT_FILES = []
CAMERA_FILES = []
FIRST = {}
LAST = {}
START = time.monotonic()


def compact(value, depth=0):
    """Крупные массивы заменяются описанием; полный облачный кадр хранится в NPZ."""
    if isinstance(value, np.ndarray):
        return {"array_shape": list(value.shape), "dtype": str(value.dtype)}
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, bytes):
        return {"bytes": len(value)}
    if isinstance(value, dict):
        return {str(k): compact(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        if len(value) > 80:
            return {"length": len(value), "first_items": [compact(v, depth + 1) for v in value[:3]]}
        return [compact(v, depth + 1) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return {"type": type(value).__name__}


def arrays(value, prefix="data"):
    result = {}
    if isinstance(value, np.ndarray):
        result[prefix] = value
    elif isinstance(value, dict):
        for key, child in value.items():
            result.update(arrays(child, prefix + "_" + str(key)))
    return result


def make_callback(name, logfile):
    def callback(message):
        try:
            now = time.monotonic()
            COUNTS[name] += 1
            FIRST.setdefault(name, now)
            LAST[name] = now
            if name not in SAMPLES:
                SAMPLES[name] = compact(message)
            record = {"topic": name, "elapsed_s": now - START, "message": compact(message)}
            if name == "ULIDAR_ARRAY" and len(POINT_FILES) < 3:
                extracted = arrays(message)
                if extracted:
                    filename = f"lidar_{len(POINT_FILES):02d}.npz"
                    np.savez_compressed(OUT / filename, **extracted)
                    POINT_FILES.append(filename)
                    record['npz_file'] = filename
            if COUNTS[name] <= 10 or 'npz_file' in record:
                logfile.write(json.dumps(record, ensure_ascii=False) + "\n")
                logfile.flush()
        except Exception as exc:
            if len(ERRORS) < 12:
                ERRORS.append({"topic": name, "error": str(exc)[:300]})
    return callback


async def main():
    conn = None
    traffic_changed = False
    camera_started = False
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    async def receive_video(track):
        last_saved = -100.0
        try:
            while not stop_event.is_set():
                frame = await track.recv()
                COUNTS["CAMERA"] += 1
                now = time.monotonic()
                FIRST.setdefault("CAMERA", now)
                LAST["CAMERA"] = now
                SAMPLES.setdefault("CAMERA", {"width": frame.width, "height": frame.height})
                if now - last_saved >= 2 and len(CAMERA_FILES) < 5:
                    filename = f"camera_{len(CAMERA_FILES):02d}.jpg"
                    frame.to_image().save(OUT / filename, quality=85)
                    CAMERA_FILES.append(filename)
                    last_saved = now
        except Exception as exc:
            if not stop_event.is_set():
                ERRORS.append({"topic": "CAMERA", "error": str(exc)[:300]})

    with (OUT / "samples.jsonl").open("w", encoding="utf-8") as logfile:
        try:
            conn = await asyncio.wait_for(go2.connect(retries=2, backoff=5), timeout=40)
            # Штатный декодер возвращает координаты занятых ячеек в метрах;
            # это обработанная воксельная карта, не сырые лучи лидара.
            conn.datachannel.set_decoder("native")
            conn.video.add_track_callback(receive_video)
            for name in ("LF_SPORT_MOD_STATE", "LOW_STATE", "ROBOTODOM", "ULIDAR_STATE", "ULIDAR_ARRAY"):
                conn.datachannel.pub_sub.subscribe(RTC_TOPIC[name], make_callback(name, logfile))
            conn.video.switchVideoChannel(True)
            camera_started = True
            try:
                traffic_changed = bool(await asyncio.wait_for(conn.datachannel.disableTrafficSaving(True), timeout=5))
            except Exception as exc:
                ERRORS.append({"topic": "LIDAR_STREAM", "error": str(exc)[:200]})
            print("DIAGNOSTICS_STARTED: чтение датчиков, без движения", flush=True)
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=18)
            except asyncio.TimeoutError:
                pass
        except Exception as exc:
            ERRORS.append({"topic": "CONNECTION", "error": type(exc).__name__ + ": " + str(exc)[:300]})
        finally:
            stop_event.set()
            if conn is not None:
                if camera_started:
                    conn.video.switchVideoChannel(False)
                if traffic_changed:
                    try:
                        await asyncio.wait_for(conn.datachannel.disableTrafficSaving(False), timeout=3)
                    except Exception:
                        pass
                await go2.disconnect(conn)
    report = {
        "time_utc": datetime.now(timezone.utc).isoformat(),
        "read_only_motion": True,
        "lidar_decoder": "native",
        "lidar_semantics": "occupied_voxel_points_in_odom_not_raw_rays",
        "counts": dict(COUNTS),
        "observed_hz": {k: round((n - 1) / (LAST[k] - FIRST[k]), 2)
            for k, n in COUNTS.items() if n > 1 and LAST[k] > FIRST[k]},
        "first_samples": SAMPLES,
        "camera_files": CAMERA_FILES,
        "lidar_files": POINT_FILES,
        "errors": ERRORS,
    }
    (OUT / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "first_samples"}, ensure_ascii=False, indent=2))
    return 0 if COUNTS["LOW_STATE"] and COUNTS["LF_SPORT_MOD_STATE"] else 2


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
