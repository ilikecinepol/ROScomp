"""Собрать исходники физического Go2 и документацию без обращения к роботу."""
from pathlib import Path
from datetime import datetime
import ast
import hashlib
import html
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parent.parent
STAMP = datetime.now().strftime('%Y%m%d-%H%M%S')
DEST = ROOT / 'outputs' / ('go2-all-source-' + STAMP)
DEST.mkdir()
PROJECT = DEST / 'project'
SKIP = {'.git', '.venv', '__pycache__', '.pytest_cache', 'node_modules', 'recordings', 'client-logs'}
TEXT = {'.py', '.ps1', '.cmd', '.bat', '.sh', '.cjs', '.js', '.toml', '.json', '.yaml', '.yml', '.md', '.html', '.txt', '.example', '.cfg', '.xml', '.msg', '.urdf', '.xacro', '.cpp', '.hpp', '.h', '.c', '.rviz'}
ASSETS = {'.dae', '.stl', '.obj', '.mtl', '.png', '.jpg', '.jpeg'}
manifest = []

def copy_source(p):
    relative = p.relative_to(ROOT)
    dst = PROJECT / relative
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)
    original = hashlib.sha256(p.read_bytes()).hexdigest()
    assert original == hashlib.sha256(dst.read_bytes()).hexdigest()
    manifest.append({'source': relative.as_posix(), 'path': dst.relative_to(DEST).as_posix(),
                     'bytes': dst.stat().st_size, 'sha256': original})

for name in ('go2-autonomy', 'go2-teleop', 'go2-perception', 'go2-remote-access'):
    base = ROOT / 'work' / name
    for p in sorted(base.rglob('*')):
        rel = p.relative_to(base)
        if not p.is_file() or any(x in SKIP for x in rel.parts):
            continue
        if p.name in {'known_hosts', '.env', '3d_map.ply'}:
            continue
        if p.suffix.lower() in TEXT | ASSETS or p.name in {'LICENSE', 'CMakeLists.txt', '.gitignore', '.gitmodules'}:
            copy_source(p)

for base in sorted((ROOT / 'outputs').glob('go2-*')):
    if not base.is_dir() or base == DEST or base.name.startswith('go2-all-source-'):
        continue
    for p in sorted(base.rglob('*')):
        rel = p.relative_to(base)
        if not p.is_file() or any(x in SKIP for x in rel.parts):
            continue
        include = p.suffix.lower() in TEXT
        include |= base.name == 'go2-autonomy' and p.parent == base and p.suffix == '.zip'
        include |= p.name == 'wire-stamps.jsonl'
        include |= p.suffix.lower() in ASSETS and (any(x.startswith('audit-') for x in rel.parts) or p.name == 'projection-review.png')
        if include:
            copy_source(p)

copy_source(Path(__file__).resolve())
current = [x for x in manifest if x['source'].startswith('work/go2-') and '/reference-go2-ros2/' not in x['source'] and x['source'].endswith('.py')]
for item in current:
    path = DEST / item['path']
    ast.parse(path.read_text(encoding='utf-8-sig'), filename=item['path'])

README = r'''# Все наработки по физическому Go2

Снимок исходников и документации от 27 сентября 2026 года. Файлы скопированы без изменения содержимого. Это передача разработки, а не новая установка на робота.

## Сначала прочитать

Полный понятный отчёт: [Go2 — вся работа и причины неготовности](project/outputs/go2-autonomy/full-audit-20260927/Go2%20—%20вся%20работа%20и%20причины%20неготовности.html).

**Подтверждённых автономных проходов физического препятствия пока нет.** Есть ручные проходы, запись датчиков, алгоритмы, диагностика и тесты. Последняя подтверждённая установка на Pi: `training-bc3e1fd08b2e139d5a02`, 27.09.2026 14:21 МСК. Подробности: `project/outputs/go2-autonomy/Настройка 20260927-1421.md`. Соответствующий архив: `training-alignment-settle.zip`.

Старые README и журналы сохранены как были: их даты и заявления относятся к соответствующим версиям. Основной актуальный код — в **project/work**, а опубликованные и исторические копии в **project/outputs** могут отставать. Исторический Webots L1 — отдельный проект; здесь собрана работа по физическому роботу.

## Карта исходников

| Папка | Что находится |
|---|---|
| `project/work/go2-autonomy/wolf_go2` | Основная бортовая программа: приём датчиков, геометрия, восприятие, маршруты, автомат миссии, команды, запись и проверки готовности |
| `project/work/go2-autonomy/tests` | Локальные тесты автономии |
| `project/work/go2-autonomy/training_panel.py` | Окно отдельных испытаний: змейка, горка, качели, рампа |
| `project/work/go2-autonomy/training_entry.py` | Бортовая точка входа тренировочного режима |
| `project/work/go2-autonomy/install_pi_release.py` | Установка подготовленной версии на Pi |
| `project/work/go2-autonomy/pulse-test` | Сохранённые скрипты коротких реальных испытаний движения; не пакетный набор для запуска подряд |
| `project/work/go2-autonomy/audit_*.py` и другие скрипты анализа | Проверка времени, геометрии, декодирования, проекции, записей и построение отчётов |
| `project/work/go2-autonomy/reference-go2-ros2` | Сторонний справочный SDK: использовался при изучении геометрии; не наша разработка и не установленный стек ROS на Pi |
| `project/work/go2-teleop` | Ручное управление, видео, отметки, запись, скачивание, работа с сеансами и тесты |
| `project/work/go2-perception` | Обработка сохранённых изображений/облаков, разбор движения и геометрии, проверки целостности |
| `project/work/go2-remote-access` | Подключение, сбор датчиков и короткие диагностические испытания |
| `project/outputs/go2-teleop` | Пользовательские CMD/PowerShell-запуски и опубликованные копии ручного инструмента |
| `project/outputs/go2-autonomy` | Предыдущие пакеты, планы, отчёты, метаданные испытаний и виртуальная арена HTML |
| `project/outputs/go2-training-v2`, `go2-other-pc` | Ранее собранные варианты переноса ручного инструмента |

Полный перечень: `КАТАЛОГ.html`. Для каждого оригинала записаны путь, размер и SHA256 в `SOURCE_MANIFEST.json`. Проверка всего передаваемого набора — `FILE_MANIFEST.json` и `tools/verify_bundle.py`.

## Что включено, а что хранится отдельно

Включены все найденные исходники четырёх основных физических модулей, тесты, настройки и примеры, запускатели, прежние опубликованные копии в каталогах go2-*, одиннадцать сохранённых тренировочных ZIP-пакетов, отчёты и небольшие диагностические JSON. Сторонний справочный код включён с лицензией и указанием происхождения.

Не включены `.venv`, Git-история, кэши, ключи SSH и known_hosts, сырые записи камер/облаков/телеметрии, клиентские журналы, большие карты и установленные бинарные программы. Архив не является резервной копией данных. Исходные записи остаются в прежней рабочей папке, в частности `outputs/go2-teleop/recordings/teleop_logs`; часть данных есть только на Pi.

Некоторые ссылки отчётов ведут на не включённые сырые материалы — для них нужна исходная папка. Два полных HTML-отчёта содержат встроенные иллюстрации и читаются самостоятельно.

## Проверка архива без робота

Распакуйте ZIP целиком. Из папки с этим файлом выполните:

```powershell
python tools/verify_bundle.py
```

Нужен только стандартный Python 3.12+; программа сравнивает SHA256 файлов и разбирает синтаксис наших текущих Python-исходников. Она не импортирует бортовой код, не устанавливает пакеты и не подключается к сети или роботу. Можно использовать `Проверить исходники.cmd`, если Python доступен через `py` или `python`.

## Подготовка окружения разработчика на другом Windows-ПК

Нужен Python 3.12+ с tkinter. Из корня распакованного набора:

```powershell
py -3.12 -m venv project/work/go2-perception/.venv
& ./project/work/go2-perception/.venv/Scripts/python.exe -m pip install -e ./project/work/go2-autonomy
& ./project/work/go2-perception/.venv/Scripts/python.exe -m pip install Pillow matplotlib
```

Это создание окружения для ПК, не инструкция менять окружение организаторов на Pi. Python, библиотеки и SDK в ZIP не вшиты; установка требует доступа к реестру пакетов. Точные версии окружений, использовавшихся ранее, приведены в техническом отчёте и `pi-inventory.json`; перенос на новые версии библиотек требует проверки.

## Команды, не подключающиеся к роботу

Полный набор тестов автономии (последний зафиксированный результат разработки — 335 успешных тестов; при упаковке весь набор повторно не запускался):

```powershell
Set-Location project/work/go2-autonomy
& ../go2-perception/.venv/Scripts/python.exe -m unittest discover -s tests -v
```

Отсюда же пример кинематической демонстрации:

```powershell
& ../go2-perception/.venv/Scripts/python.exe -m wolf_go2 demo --seed 0 --output demo-new.json
```

`demo-new.json` должен быть новым файлом. Это модель без физики лап и сцепления, не подтверждение реального прохода.

Показать окно ручного управления без подключения (из корня распакованного набора):

```powershell
& ./project/work/go2-perception/.venv/Scripts/python.exe ./project/work/go2-teleop/keyboard_client.py --dry-run
```

## Подключение и перенос запускателей

Последний известный адрес Pi — 192.168.11.40, пользователь ubuntu. Актуальность адреса и доступность слота при упаковке не проверялись. SSH-ключ передаётся отдельно; по умолчанию клиент ищет его в `~/.ssh/go2-pi-0008/id_ed25519`. Ключа в этом архиве нет.

Некоторые исторические PS1 и диагностические скрипты используют абсолютный путь `C:\Users\VladO\Documents\Codex\2026-09-09\new-chat`. Они сохранены без переделки и не являются переносимыми запускателями. Для переноса читайте аргументы Python-точек входа и исправляйте конфигурацию путей под новую папку. Исторические `build_release.py` и публикационные скрипты могут менять копии и относиться к старой версии: не запускать их для «обновить всё».

Запускатели live, arm_*, run_* в pulse-test и команды установки/сброса сеансов способны менять состояние Pi или двигать робота. Это отдельные инструменты испытаний, а не обязательный шаг проверки архива.

Файл `autonomy-profile.json` не подставлялся: подтверждённого действующего профиля нет. Кнопки автономных навыков остаются ограничены незавершёнными проверками/интеграцией. Нельзя считать передачу исходников готовым автономным продуктом.

## Сторонний справочный код

`reference-go2-ros2`: https://github.com/abizovnuralem/go2_ros2_sdk.git, локальный HEAD `b440609591a249e7bdd4bbc88e056a3660575447`. Лицензия в его `LICENSE`. Git-служебные файлы и `3d_map.ply` исключены. Параметры этой модели не аттестованы как калибровка нашего Go2.

Штатный `go2.py` организаторов и внутреннее ПО собаки не выдаются за наш код. Для реального запуска используются доступные на назначенном Pi SDK и его окружение.
'''
(DEST / 'НАЧНИТЕ ЗДЕСЬ.md').write_text(README, encoding='utf-8-sig')
(DEST / 'tools').mkdir()
verifier = '''"""Проверить переданные файлы без импорта кода и соединения с роботом."""
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
'''
(DEST / 'tools/verify_bundle.py').write_text(verifier, encoding='utf-8')
(DEST / 'Проверить исходники.cmd').write_text('@echo off\nsetlocal\ncd /d "%~dp0"\nwhere py >nul 2>nul\nif errorlevel 1 (\n  python tools\\verify_bundle.py\n) else (\n  py -3 tools\\verify_bundle.py\n)\npause\nendlocal\n', encoding='ascii')
info = {'created': datetime.now().isoformat(), 'scope': 'physical_go2_source_and_documentation',
        'files': manifest, 'copied_without_changes': True, 'python_files_syntax_checked': len(current),
        'robot_connected': False, 'unit_tests_rerun': False}
(DEST / 'SOURCE_MANIFEST.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
rows = ''.join('<tr><td><a href="'+html.escape(x['path'],quote=True)+'">'+html.escape(x['source'])+'</a></td><td>'+str(x['bytes'])+'</td><td>'+x['sha256']+'</td></tr>' for x in manifest)
page = '<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Go2 — все исходники</title><style>body{font:16px/1.6 system-ui;margin:35px;color:#18384b}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left}td:last-child{font:11px monospace;overflow-wrap:anywhere}a{color:#146484}input{padding:12px;width:90%;max-width:700px}</style><h1>Все наработки физического Go2</h1><p>Снимок исходников и документации. Автономный проход физической полосы не подтверждён.</p><p><a href="НАЧНИТЕ ЗДЕСЬ.md">Инструкция и карта проекта</a> · <a href="project/outputs/go2-autonomy/full-audit-20260927/Go2 — вся работа и причины неготовности.html">Полный отчёт</a></p><p>'+str(len(manifest))+' оригинальных файлов; '+str(len(current))+' текущих Python-файлов проверены на синтаксис.</p><input id="q" aria-label="Фильтр файлов" placeholder="Найти файл или папку…"><table><thead><tr><th>Исходный путь</th><th>Байт</th><th>SHA256</th></tr></thead><tbody>'+rows+'</tbody></table><script>document.getElementById("q").oninput=e=>{let q=e.target.value.toLowerCase();document.querySelectorAll("tbody tr").forEach(r=>r.hidden=!r.textContent.toLowerCase().includes(q))}</script></html>'
(DEST / 'КАТАЛОГ.html').write_text(page, encoding='utf-8')
checks = [{'path':p.relative_to(DEST).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(DEST.rglob('*')) if p.is_file()]
(DEST / 'FILE_MANIFEST.json').write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding='utf-8')
archive = DEST.with_suffix('.zip')
with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as pack:
    for p in sorted(DEST.rglob('*')):
        if p.is_file(): pack.write(p, DEST.name+'/'+p.relative_to(DEST).as_posix())
with zipfile.ZipFile(archive) as pack:
    assert pack.testzip() is None
    for item in checks:
        assert hashlib.sha256(pack.read(DEST.name+'/'+item['path'])).hexdigest() == item['sha256']
digest = hashlib.sha256(archive.read_bytes()).hexdigest()
archive.with_suffix('.zip.sha256').write_text(digest+'  '+archive.name+'\n', encoding='ascii')
print(json.dumps({'folder':str(DEST),'archive':str(archive),'archive_bytes':archive.stat().st_size,'files':len(checks)+1,'source_files':len(manifest),'python_checked':len(current),'sha256':digest},ensure_ascii=False))
