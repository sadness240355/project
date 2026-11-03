"""Solid selected markers for native ttk checkbuttons, including HiDPI scaling."""
import tkinter as tk

class Checkmarks:
    def __init__(self, root, style, background, accent, muted):
        self.colors = background, accent, muted
        self.images = {key: tk.PhotoImage(master=root) for key in
                       ('off', 'on', 'disabled_off', 'disabled_on', 'mixed')}
        self.resize(1.0)
        name = 'Lrdma.Check.indicator'
        style.element_create(name, 'image', self.images['off'],
                             ('disabled', 'selected', self.images['disabled_on']),
                             ('disabled', self.images['disabled_off']),
                             ('alternate', self.images['mixed']),
                             ('selected', self.images['on']), sticky='')
        def replace(layout):
            result = []
            for element, options in layout:
                options = dict(options)
                if element.endswith('.indicator'):
                    element = name
                if 'children' in options:
                    options['children'] = replace(options['children'])
                result.append((element, options))
            return result
        style.layout('TCheckbutton', replace(style.layout('TCheckbutton')))

    def resize(self, scale):
        n = max(14, round(18 * scale))
        edge = max(1, round(scale))
        gap = max(2, round(3 * scale))
        bg, accent, muted = self.colors
        for key, im in self.images.items():
            im.configure(width=n + gap, height=n)
            im.blank()
            border = muted if key.startswith('disabled') else accent
            im.put(border, to=(0, 0, n, n))
            im.put(bg, to=(edge, edge, n-edge, n-edge))
            if key in ('on', 'disabled_on'):
                im.put(border, to=(gap, gap, n-gap, n-gap))
            elif key == 'mixed':
                im.put(border, to=(gap, n//2-edge, n-gap, n//2+edge))
