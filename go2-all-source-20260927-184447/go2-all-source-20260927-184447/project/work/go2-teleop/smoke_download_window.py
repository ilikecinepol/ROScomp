"""Проверка отдельного окна с поддельным SSH, без сети."""
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import tkinter as tk
from tkinter import ttk
from download_recordings import open_downloads, saved_host

with tempfile.TemporaryDirectory() as d:
    config=Path(d)/'config.json'
    config.write_text('{"Ip":"192.168.11.40"}',encoding='utf-8-sig')
    assert saved_host(config)=='192.168.11.40'
    data=[dict(name='20260924T104031.431138Z',size=1048576,count=12,complete=True)]
    with patch('download_recordings.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(data).encode())) as ssh:
        win=open_downloads(None,saved_host(config),Path(d)/'key',Path(d)/'records',config)
        def descendants(w):
            for c in w.winfo_children():
                yield c
                yield from descendants(c)
        def verify():
            try:
                listing=next(c for c in descendants(win) if isinstance(c,tk.Listbox))
                assert listing.size()==1
                assert '1.00' in listing.get(0)
                assert ssh.call_count==1
                delete=next(c for c in descendants(win) if isinstance(c,ttk.Button) and str(c.cget('text')).startswith('Удалить все'))
                with patch('tkinter.messagebox.askyesno',return_value=False) as confirm:
                    delete.invoke()
                    confirm.assert_called_once()
                assert ssh.call_count==1, 'Отмена не должна запускать удаление'
                entry=next(c for c in descendants(win) if isinstance(c,ttk.Entry))
                entry.delete(0,'end');entry.insert(0,'192.168.11.41')
                assert listing.size()==0
                win.test_passed=True
            finally:win.destroy()
        win.after(600,verify)
        win.after(5000,win.destroy)
        win.mainloop()
        assert getattr(win,'test_passed',False)
print('Standalone downloader UI: OK; no network used')
