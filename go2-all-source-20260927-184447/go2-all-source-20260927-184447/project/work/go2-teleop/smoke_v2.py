"""Проверка окна с поддельным транспортом: ни SSH, ни робот не используются."""
import tkinter as tk
from keyboard_client import KeyboardWindow, DemoTransport

root = tk.Tk()
transport = DemoTransport()
window = KeyboardWindow(root, transport, dry_run=True)
root.update()
root.focus_force()

def check():
    assert window.state.arm_intent, 'Автоподготовка не сработала'
    assert not any(m.get('deadman') for m in transport.sent), 'Движение без клавиши'
    window.profile_box.current(2)
    window.change_profile()
    root.after(400, record)

def record():
    assert transport.profile == 'floor'
    window.record('record_start')
    root.after(400, finish)

def finish():
    assert transport.recording == 'demo_run'
    window.record('record_stop')
    root.after(400, done)

def done():
    assert transport.recording is None
    assert transport.saved_recording == 'demo_run'
    root.update()
    for widget in (window.profile_box, window.record_start_button, window.record_stop_button):
        assert widget.winfo_ismapped()
        assert widget.winfo_rooty() + widget.winfo_height() <= root.winfo_rooty() + root.winfo_height()
    print('UI OK: автоподготовка, профиль, начало/конец записи; движения не было')
    root.destroy()

root.after(2500, check)
root.after(12000, root.destroy)
root.mainloop()
