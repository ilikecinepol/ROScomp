"""Проверить переданные файлы без импорта кода и соединения с роботом."""
from pathlib import Path
import ast, hashlib, json, sys
root=Path(__file__).resolve().parent.parent
entries=json.loads((root/'FILE_MANIFEST.json').read_text(encoding='utf-8'))
errors=[]
for item in entries:
    p=root/item['path']
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:
        errors.append(item['path'])
sources=json.loads((root/'SOURCE_MANIFEST.json').read_text(encoding='utf-8'))['files']
count=0
for item in sources:
    if item['source'].startswith('work/go2-') and '/reference-go2-ros2/' not in item['source'] and item['source'].endswith('.py'):
        p=root/item['path']
        try: ast.parse(p.read_text(encoding='utf-8-sig'),filename=item['path']); count+=1
        except Exception as exc: errors.append(item['path']+': '+str(exc))
print('Files checked:',len(entries),'Python syntax checked:',count)
if errors:
    print('ERRORS:',json.dumps(errors,ensure_ascii=True,indent=2));sys.exit(1)
print('OK. No network access; no robot commands; no code imports.')
