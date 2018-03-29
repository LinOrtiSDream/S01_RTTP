import tkinter as tk
from tkinter.scrolledtext import ScrolledText


class LogWidget(ScrolledText):
    def __init__(self, parent, evnt, **kwargs):
        super().__init__(parent, **kwargs)

        @evnt.on('log')
        def on_log(log_text):
            self.insert(tk.END, log_text)
            self.yview_moveto(1)


class StatusBar(tk.Frame):
    def __init__(self, parent, evnt, *args, bd=1, relief=tk.RAISED, **kwargs):
        super().__init__(parent,  bd=bd, relief=relief, **kwargs)
        self.status_string = tk.StringVar()

        frame_status = tk.Frame(self, bd=1, relief=tk.SUNKEN).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Label(frame_status, textvariable=self.status_string, anchor=tk.W).pack(side=tk.LEFT, fill=tk.X, expand=True)

        for arg in reversed(args):
            subframe = tk.Frame(self, bd=1, relief=tk.SUNKEN).pack(side=tk.RIGHT)
            for txt in reversed(arg):
                tk.Label(subframe, text=txt).pack(side=tk.RIGHT)

        @evnt.on('status')
        def on_status(status_text):
            self.status_string.set(status_text)
            self.update_idletasks()