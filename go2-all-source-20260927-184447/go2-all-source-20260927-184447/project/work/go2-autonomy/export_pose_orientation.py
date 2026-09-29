"""Извлекает точную записанную ориентацию пары, не изменяя исходные файлы."""
import json
from pathlib import Path
import sys

for name in sys.argv[1:]:
    folder=Path(name)
    metadata=json.loads((folder/'cloud_000010.json').read_text())
    target=metadata['pose_received_at']
    found=None
    with (folder/'samples.jsonl').open() as source:
        for line in source:
            row=json.loads(line)
            if row.get('topic')=='ROBOTODOM' and abs(row['t']-target)<1e-6:
                found=row['message']['data']['pose']['orientation']
                break
    if found is None:
        raise ValueError('Не найдена точная пара '+name)
    print(json.dumps({'session':folder.name,'pose_received_at':target,'orientation':found}))
