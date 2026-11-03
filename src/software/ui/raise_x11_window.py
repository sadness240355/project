"""Request that the X11 window manager activate an existing window."""
import ctypes
import os
import sys

lib = ctypes.CDLL('libX11.so.6')
lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
lib.XOpenDisplay.restype = ctypes.c_void_p
lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
lib.XDefaultRootWindow.restype = ctypes.c_ulong
lib.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
lib.XInternAtom.restype = ctypes.c_ulong
lib.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                           ctypes.c_long, ctypes.c_void_p]
lib.XSendEvent.restype = ctypes.c_int
lib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
lib.XFlush.argtypes = [ctypes.c_void_p]
lib.XCloseDisplay.argtypes = [ctypes.c_void_p]


class ClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                ('format', ctypes.c_int), ('data', ctypes.c_long * 5)]


window = int(sys.argv[1], 0)
display = lib.XOpenDisplay(os.environ.get('DISPLAY', ':0').encode())
if not display:
    raise SystemExit('cannot open X display')
root = lib.XDefaultRootWindow(display)
atom = lib.XInternAtom(display, b'_NET_ACTIVE_WINDOW', 0)
event = ClientMessage()
event.type = 33  # ClientMessage
event.display = display
event.window = window
event.message_type = atom
event.format = 32
event.data[0] = 1  # application request
lib.XRaiseWindow(display, window)
if not lib.XSendEvent(display, root, 0, (1 << 19) | (1 << 20),
                      ctypes.byref(event)):
    raise SystemExit('window manager did not accept activation request')
if len(sys.argv) > 2 and sys.argv[2] == 'maximize':
    event.message_type = lib.XInternAtom(display, b'_NET_WM_STATE', 0)
    event.data[0] = 1  # add
    event.data[1] = lib.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_VERT', 0)
    event.data[2] = lib.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_HORZ', 0)
    if not lib.XSendEvent(display, root, 0, (1 << 19) | (1 << 20),
                          ctypes.byref(event)):
        raise SystemExit('window manager did not accept maximize request')
lib.XFlush(display)
lib.XCloseDisplay(display)
print('ACTIVATION_REQUESTED', hex(window))
