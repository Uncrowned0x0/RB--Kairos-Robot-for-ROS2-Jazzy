# Copyright 2026 Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
GUI Dashboard for RB-KAIROS 48V Battery & Drive Voltage Monitoring.

Author: Kamil BENMADI <kamil.benmadi@sigma-clermont.fr>
"""

import os
import struct
import sys
import time
import tkinter as tk

import can


class KairosBatteryMonitor:
    """GUI Dashboard for RB-KAIROS 48V Battery & Drive Voltage Monitoring."""

    def __init__(self, root):
        self.root = root
        self.root.title('RB-KAIROS — 48V Battery Monitor')
        geom = os.environ.get('BATTERY_GUI_GEOMETRY', '480x560')
        for arg in sys.argv[1:]:
            if arg.startswith('--geometry='):
                geom = arg.split('=', 1)[1]
        self.root.geometry(geom)
        self.root.minsize(440, 500)
        self.root.configure(bg='#181825')

        self.bus = None
        self.nodes = [1, 2, 3, 4]
        self.voltages = {n: 0.0 for n in self.nodes}
        self.node_names = {
            1: 'Node 1 (Front-Left)',
            2: 'Node 2 (Rear-Left)',
            3: 'Node 3 (Front-Right)',
            4: 'Node 4 (Rear-Right)'
        }

        self.setup_ui()
        self.connect_can()
        self.poll_loop()

    def setup_ui(self):
        """Build the GUI widgets."""
        # Header
        header = tk.Frame(self.root, bg='#181825')
        header.pack(fill='x', padx=20, pady=(15, 5))

        title = tk.Label(
            header,
            text='🔋 RB-KAIROS — 48V BATTERY CHARGE',
            font=('Helvetica', 13, 'bold'),
            fg='#89b4fa',
            bg='#181825'
        )
        title.pack()

        subtitle = tk.Label(
            header,
            text='⚠️ 48V battery energy level (this is NOT the robot drive speed!)',
            font=('Helvetica', 9, 'italic'),
            fg='#fab387',
            bg='#181825'
        )
        subtitle.pack(pady=(2, 0))

        # Main Battery Card
        self.card = tk.Frame(self.root, bg='#1e1e2e', bd=0, relief='flat')
        self.card.pack(fill='x', padx=20, pady=10)

        self.card_header = tk.Label(
            self.card,
            text='REMAINING 48V BATTERY ENERGY',
            font=('Helvetica', 10, 'bold'),
            fg='#a6adc8',
            bg='#1e1e2e'
        )
        self.card_header.pack(pady=(10, 0))

        # Big percentage label
        self.pct_label = tk.Label(
            self.card,
            text='-- %',
            font=('Helvetica', 38, 'bold'),
            fg='#a6e3a1',
            bg='#1e1e2e'
        )
        self.pct_label.pack(pady=(5, 0))

        # Big voltage label
        self.volt_label = tk.Label(
            self.card,
            text='--.- V',
            font=('Helvetica', 18, 'bold'),
            fg='#cdd6f4',
            bg='#1e1e2e'
        )
        self.volt_label.pack(pady=(0, 10))

        # Canvas gauge
        self.canvas_gauge = tk.Canvas(
            self.card,
            height=26,
            bg='#313244',
            highlightthickness=0
        )
        self.canvas_gauge.pack(fill='x', padx=25, pady=(0, 15))

        # Status badge
        self.status_badge = tk.Label(
            self.card,
            text='INITIALIZING...',
            font=('Helvetica', 10, 'bold'),
            fg='#181825',
            bg='#f9e2af',
            padx=12,
            pady=4
        )
        self.status_badge.pack(pady=(0, 15))

        # Individual Drives Card
        drives_card = tk.LabelFrame(
            self.root,
            text=' DC Bus Voltage per Drive (48V) ',
            font=('Helvetica', 10, 'bold'),
            fg='#cdd6f4',
            bg='#1e1e2e',
            bd=1,
            relief='solid'
        )
        drives_card.pack(fill='both', expand=True, padx=20, pady=10)

        self.drive_labels = {}
        for n in self.nodes:
            row = tk.Frame(drives_card, bg='#1e1e2e')
            row.pack(fill='x', padx=15, pady=4)

            name_lbl = tk.Label(
                row,
                text=self.node_names[n],
                font=('Helvetica', 10),
                fg='#bac2de',
                bg='#1e1e2e'
            )
            name_lbl.pack(side='left')

            val_lbl = tk.Label(
                row,
                text='--.- V',
                font=('Helvetica', 10, 'bold'),
                fg='#cdd6f4',
                bg='#1e1e2e'
            )
            val_lbl.pack(side='right')
            self.drive_labels[n] = val_lbl

        # Footer / Info
        footer = tk.Frame(self.root, bg='#181825')
        footer.pack(fill='x', padx=20, pady=10)

        self.can_status_lbl = tk.Label(
            footer,
            text='CAN: Connecting can0...',
            font=('Helvetica', 9),
            fg='#6c7086',
            bg='#181825'
        )
        self.can_status_lbl.pack(side='left')

        refresh_btn = tk.Button(
            footer,
            text='Refresh',
            font=('Helvetica', 9, 'bold'),
            command=self.poll_can,
            bg='#313244',
            fg='#cdd6f4',
            activebackground='#45475a',
            activeforeground='#ffffff',
            relief='flat',
            padx=10,
            pady=2
        )
        refresh_btn.pack(side='right')

    def connect_can(self):
        """Open socketcan can0."""
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
            self.can_status_lbl.config(text='CAN: can0 @ 1Mbps connected', fg='#a6e3a1')
        except Exception as e:
            self.can_status_lbl.config(text=f'CAN: Error ({e})', fg='#f38ba8')
            self.bus = None

    def poll_can(self):
        """Query 0x200F on all 4 drives and update UI."""
        if not self.bus:
            self.connect_can()
            if not self.bus:
                return

        for node in self.nodes:
            # SDO Upload Request: Index 0x200F, Sub 0x01
            msg = can.Message(
                arbitration_id=0x600 + node,
                data=[0x40, 0x0F, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            try:
                self.bus.send(msg)
            except Exception:
                pass

        # Collect replies for 150ms
        start = time.time()
        while time.time() - start < 0.15:
            try:
                resp = self.bus.recv(timeout=0.03)
                if resp and (0x581 <= resp.arbitration_id <= 0x584) and len(resp.data) >= 8:
                    if resp.data[1] == 0x0F and resp.data[2] == 0x20 and resp.data[3] == 0x01:
                        n = resp.arbitration_id - 0x580
                        raw = struct.unpack('<i', bytes(resp.data[4:8]))[0]
                        voltage = raw / 177.3160173
                        self.voltages[n] = voltage
            except Exception:
                break

        self.update_display()

    def update_display(self):
        """Update display elements based on read voltages."""
        v_main = self.voltages[1]
        # Update individual labels
        for n in self.nodes:
            v = self.voltages[n]
            color = '#a6e3a1' if v > 44.0 else ('#fab387' if v > 20.0 else '#6c7086')
            self.drive_labels[n].config(text=f'{v:.2f} V', fg=color)

        if v_main < 20.0:
            # Contactor open / Power cut
            self.pct_label.config(text='-- %', fg='#6c7086')
            self.volt_label.config(text=f'{v_main:.1f} V (Cut)', fg='#f38ba8')
            self.status_badge.config(
                text='CONTACTOR OPEN (48V INACTIVE - Press blue button)',
                bg='#f38ba8',
                fg='#11111b'
            )
            self.draw_gauge(0.0, '#6c7086')
        else:
            # Active 48V battery
            # 44V = 0%, 53.3V = 100%
            pct = max(0.0, min(100.0, (v_main - 44.0) / (53.3 - 44.0) * 100.0))
            color = '#a6e3a1' if pct > 45.0 else ('#f9e2af' if pct > 20.0 else '#f38ba8')

            self.pct_label.config(text=f'{pct:.0f} %', fg=color)
            self.volt_label.config(text=f'{v_main:.2f} V', fg='#cdd6f4')

            if v_main >= 53.0:
                status_text = 'BATTERY FULL / CHARGING'
                badge_bg = '#a6e3a1'
            elif pct > 20.0:
                status_text = 'OPERATIONAL (48V ACTIVE)'
                badge_bg = '#89b4fa'
            else:
                status_text = 'WARNING: LOW BATTERY (< 20%)'
                badge_bg = '#fab387'

            self.status_badge.config(text=status_text, bg=badge_bg, fg='#11111b')
            self.draw_gauge(pct / 100.0, color)

    def draw_gauge(self, ratio, color):
        """Draw progress bar on canvas."""
        self.canvas_gauge.delete('all')
        w = self.canvas_gauge.winfo_width()
        if w < 10:
            w = 390
        fill_w = int(w * max(0.0, min(1.0, ratio)))
        if fill_w > 0:
            self.canvas_gauge.create_rectangle(0, 0, fill_w, 26, fill=color, width=0)

    def poll_loop(self):
        """Recurring poll loop every 1000ms."""
        self.poll_can()
        self.root.after(1000, self.poll_loop)


def main():
    """Start the Tkinter battery dashboard."""
    root = tk.Tk()
    KairosBatteryMonitor(root)
    root.mainloop()


if __name__ == '__main__':
    main()
