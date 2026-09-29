"""Пакетная проверка парных кадров без подключения к роботу."""
import argparse
import json
from pathlib import Path
from audit_camera_projection import audit


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('roots',type=Path,nargs='+')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    results=[];seen=set();duplicates=0
    for root in args.roots:
        for path in sorted(root.rglob('cloud_*.json')):
            if path.name.endswith('.orientation.json'):
                continue
            try:
                meta=json.loads(path.read_text(encoding='utf-8-sig'))
                if not meta.get('camera_file') or not meta.get('pose'):
                    continue
                if not (path.parent/meta['camera_file']).is_file():
                    continue
                key=(meta.get('pi_boot_id'),meta.get('frame_epoch'),meta.get('received_at'),meta.get('cloud_source_s'))
                if key[0] is not None and key[2] is not None:
                    if key in seen:
                        duplicates+=1
                        continue
                    seen.add(key)
                results.append(audit(path))
            except Exception as exc:
                results.append({'frame':str(path),'error':str(exc)})
    successful=[r for r in results if 'error' not in r]
    summary={'frames':len(results),'processed':len(successful),
             'duplicate_pairs_skipped':duplicates,
             'frames_with_three_camera_candidates':sum(r['camera_candidates']>=3 for r in successful),
             'frames_with_three_accepted':sum(r['accepted_matches']>=3 for r in successful),
             'errors':sum('error' in r for r in results),
             'calibration_verified':False,'motion_authorized':False}
    args.output.write_text(json.dumps({'summary':summary,'frames':results},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))
