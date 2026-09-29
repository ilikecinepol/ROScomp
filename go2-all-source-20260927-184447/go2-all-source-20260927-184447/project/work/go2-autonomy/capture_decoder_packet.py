"""Один исходный пакет для независимой проверки декодеров; только observe."""
import json
from pathlib import Path
import sys

TEAM = Path('/home/ubuntu/ai-robot/team_wolf_setup')
sys.path.insert(0, str(TEAM/'autonomy-current'))
from wolf_go2.packet_audit import PacketAudit
import training_entry

original = PacketAudit.decode


def capture(self, payload, metadata):
    if self.count == 20:
        header = {k: v for k, v in metadata.items() if k != 'data'}
        # Файлы одного сеанса; сохраняем до любых преобразований декодером.
        (self.journal.path/'raw-lidar.bin').write_bytes(bytes(payload))
        (self.journal.path/'raw-lidar.json').write_text(json.dumps(header), encoding='utf-8')
    return original(self, payload, metadata)


if __name__ == '__main__':
    PacketAudit.decode = capture
    sys.argv = [sys.argv[0], '--observe']
    raise SystemExit(training_entry.main())
