"""Структурная проверка HTML и комплекта; команды не выполняются."""
from html.parser import HTMLParser
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parents[2]
out = root / 'outputs/go2-training-v2'
class Check(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids, self.links, self.commands = [], [], []
        self.buttons = 0
        self.in_command = False
        self.capture = False
        self.text = ''
    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if 'id' in a: self.ids.append(a['id'])
        if tag == 'a': self.links.append(a.get('href', ''))
        if tag == 'button' and a.get('class') == 'copy':
            self.buttons += 1
            self.in_command = True
        if tag == 'code' and self.in_command:
            self.capture = True
            self.text = ''
        assert not (tag in ('script', 'img', 'link') and ('src' in a or 'href' in a)), 'Внешний ресурс'
    def handle_data(self, text):
        if self.capture: self.text += text
    def handle_endtag(self, tag):
        if tag == 'code' and self.capture:
            self.commands.append(self.text)
            self.capture = self.in_command = False

p = Check()
p.feed(next(out.glob('*.html')).read_text(encoding='utf-8'))
assert len(p.ids) == len(set(p.ids))
assert all(link.startswith('#') and link[1:] in p.ids for link in p.links)
assert len(p.commands) == p.buttons == 15
assert all('\\"' not in c for c in p.commands)
(root / 'outputs/go2-teleop/guide-commands.json').write_text(json.dumps(p.commands, ensure_ascii=False), encoding='utf-8')
manifest = json.loads((out / 'SHA256.json').read_text(encoding='utf-8'))
for name, digest in manifest.items():
    assert hashlib.sha256((out / name).read_bytes()).hexdigest() == digest
with zipfile.ZipFile(root / 'outputs/Go2-training-v2.zip') as z:
    assert z.testzip() is None
    assert set(z.namelist()) == set(manifest) | {'SHA256.json'}
    assert len(z.namelist()) == 7
print('OK: 15 команд, якоря HTML, автономный файл, 7 файлов ZIP, хеши совпадают')
