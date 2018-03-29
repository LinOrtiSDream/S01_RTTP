#!/usr/bin/env python

# Built-in modules
import logging
import tkinter


class TextHandler(logging.Handler):
    """This class allows you to log to a tkinter Text or ScrolledText widget"""
    def __init__(self, text):
        # run the regular Handler __init__
        logging.Handler.__init__(self)
        # Store a reference to the Text it will log to
        self.text = text

    def emit(self, record):
        msg = self.format(record)

        def append():
            self.text.configure(state='normal')
            self.text.insert(tkinter.END, msg + '\n')
            self.text.configure(state='disabled')
            # Autoscroll to the bottom
            self.text.yview(tkinter.END)
        # This is necessary because we can't modify the Text from other threads
        self.text.after(0, append)


# Sample usage
if __name__ == '__main__':
    # Create the GUI
    root = tkinter.Tk()

    from tkinter.scrolledtext import ScrolledText
    st = ScrolledText(root, state='disabled')
    st.configure(font='TkFixedFont')
    st.pack()

    # Create textLogger
    text_handler = TextHandler(st)

    # Add the handler to log
    log = logging.getLogger()
    log.addHandler(text_handler)

    # Log some messages
    log.debug('debug message')
    log.info('info message')
    log.warning('warn message')
    log.error('error message')
    log.critical('critical message')

    root.mainloop()
