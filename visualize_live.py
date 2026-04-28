#!/usr/bin/env python3
"""
ARCUIA Live Flight HUD
=======================
Reads CSV data from the Teensy over USB Serial and renders a real-time HUD.

Expects 20-column CSV from AHRSFilter::printCSV():
    px, py, pz, vx, vy, vz,
    qw, qx, qy, qz,
    alt_ft, climbrate_ftmin, airspeed_kts, heading_deg,
    pitch_deg, roll_deg,
    yaw_rad, pitch_rad, roll_rad,
    baro_pa

USAGE
-----
    pip install pyserial pyqtgraph PyQt5 PyOpenGL numpy
    python visualize_live.py --port COM3
    python visualize_live.py --port /dev/ttyACM0     # Linux/macOS

Press Ctrl-C or close the window to exit.
"""

import sys
import time
import argparse
import threading
import numpy as np

try:
    import serial
except ImportError:
    print("Missing pyserial — run: pip install pyserial"); sys.exit(1)
try:
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError:
    print("Missing pyqtgraph — run: pip install pyqtgraph PyQt5 PyOpenGL"); sys.exit(1)


# ─── Constants ────────────────────────────────────────────────────────────────
N_COLS    = 20         # columns in incoming CSV
WINDOW_S  = 10.0       # rolling window length [s]
RENDER_HZ = 30         # GUI redraw rate (decoupled from data rate)

# Colors (matching old MATLAB HUD)
C_BG       = '#0d0d0d'
C_GRID     = '#4d4d4d'
C_CYAN     = (0,   255, 255)
C_RED      = (255,  77,  77)
C_GREEN    = (0,   255,   0)
C_YELLOW   = (255, 255,   0)
C_ORANGE   = (255, 153,   0)
C_WHITE    = '#ffffff'


# ─────────────────────────────────────────────────────────────────────────────
#  SERIAL READER  —  background thread, lock-free numpy circular buffer
# ─────────────────────────────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port, baud, buffer_size=5000):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.buf  = np.zeros((buffer_size, N_COLS + 1), dtype=np.float32)
        self.idx  = 0
        self.cnt  = 0
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.t0 = None

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            ser.reset_input_buffer()
        except Exception as e:
            print(f"[Serial] Open failed: {e}")
            return

        self.connected = True
        self.t0 = time.time()
        print(f"[Serial] Connected: {self.port} @ {self.baud}")

        while self.running:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) < N_COLS:
                    continue
                vals = np.fromstring(','.join(parts[:N_COLS]), sep=',', dtype=np.float32)
                if vals.size != N_COLS:
                    continue
                t = time.time() - self.t0
                with self.lock:
                    self.buf[self.idx, 0] = t
                    self.buf[self.idx, 1:] = vals
                    self.idx = (self.idx + 1) % self.buf.shape[0]
                    self.cnt = min(self.cnt + 1, self.buf.shape[0])
            except Exception:
                continue

        ser.close()
        print("[Serial] Closed")

    def snapshot(self):
        """Return a copy of the buffer in chronological order."""
        with self.lock:
            if self.cnt == 0:
                return np.zeros((0, N_COLS + 1), dtype=np.float32)
            if self.cnt < self.buf.shape[0]:
                return self.buf[:self.cnt].copy()
            return np.vstack((self.buf[self.idx:], self.buf[:self.idx])).copy()

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────────────────────
#  HUD WINDOW
# ─────────────────────────────────────────────────────────────────────────────
class HUD(QtWidgets.QMainWindow):
    def __init__(self, reader):
        super().__init__()
        self.reader = reader
        self.max_alt = -float('inf')

        self.setWindowTitle('ARCUIA Flight HUD — Live')
        self.setStyleSheet(f'background-color: black; color: {C_WHITE};')

        pg.setConfigOption('background', C_BG)
        pg.setConfigOption('foreground', C_WHITE)
        pg.setConfigOptions(antialias=True)

        # ── Master layout ────────────────────────────────────────────────────
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        grid = QtWidgets.QGridLayout(central)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setSpacing(6)

        # ── 3D trajectory (top-left) ─────────────────────────────────────────
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor((13, 13, 13, 255))
        self.gl_view.setCameraPosition(distance=300, elevation=25, azimuth=45)
        floor = gl.GLGridItem()
        floor.setSize(x=400, y=400); floor.setSpacing(x=20, y=20)
        floor.setColor((0, 255, 0, 50))
        self.gl_view.addItem(floor)
        self.traj_line = gl.GLLinePlotItem(pos=np.zeros((1,3), dtype=np.float32),
                                             color=(0,1,1,1), width=2, antialias=True)
        self.pos_dot   = gl.GLScatterPlotItem(pos=np.zeros((1,3), dtype=np.float32),
                                                color=(1,0.3,0.3,1), size=12)
        self.gl_view.addItem(self.traj_line)
        self.gl_view.addItem(self.pos_dot)
        self._wrap(self.gl_view, '3D TRAJECTORY', grid, 0, 0)

        # ── Time-series stack (bottom-left) ──────────────────────────────────
        ts_box = QtWidgets.QWidget()
        ts_layout = QtWidgets.QVBoxLayout(ts_box)
        ts_layout.setContentsMargins(0, 0, 0, 0); ts_layout.setSpacing(4)

        self.alt_plot, self.alt_curve = self._make_plot(
            'ALTITUDE [ft]', [('Alt', C_CYAN)])
        self.vel_plot, self.vel_curves = self._make_plot(
            'VELOCITY [N E D] (m/s)', [('N', C_RED), ('E', C_GREEN), ('D', C_CYAN)])
        self.ori_plot, self.ori_curves = self._make_plot(
            'ORIENTATION [Roll Pitch Yaw] (deg)',
            [('Roll', C_RED), ('Pitch', C_GREEN), ('Yaw', C_CYAN)])

        ts_layout.addWidget(self.alt_plot)
        ts_layout.addWidget(self.vel_plot)
        ts_layout.addWidget(self.ori_plot)
        grid.addWidget(ts_box, 1, 0)

        # ── Readout panel (top-right) ────────────────────────────────────────
        self.readout = QtWidgets.QLabel('WAITING FOR DATA…')
        self.readout.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.readout.setStyleSheet(f"""
            color: #00ffff;
            background-color: {C_BG};
            border: 1px solid white;
            font-family: Consolas, 'Courier New', monospace;
            font-size: 14px;
            font-weight: bold;
            padding: 12px;
        """)
        self.readout.setMinimumWidth(280)
        grid.addWidget(self.readout, 0, 1)

        # ── Ground track (bottom-right) ──────────────────────────────────────
        self.gt_plot = pg.PlotWidget()
        self.gt_plot.setAspectLocked(True)
        self.gt_plot.showGrid(x=True, y=True, alpha=0.4)
        self.gt_plot.setLabel('bottom', 'East (m)')
        self.gt_plot.setLabel('left',   'North (m)')
        self.gt_curve  = self.gt_plot.plot(pen=pg.mkPen(C_CYAN, width=2))
        self.gt_marker = self.gt_plot.plot(pen=None, symbol='o',
                                            symbolBrush=C_RED, symbolSize=12)
        self._wrap(self.gt_plot, 'GROUND TRACK', grid, 1, 1)

        # ── Column / row stretch ─────────────────────────────────────────────
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 4)
        grid.setRowStretch(1, 5)

        # ── Render timer ─────────────────────────────────────────────────────
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000 / RENDER_HZ))

    # -------------------------------------------------------------------------
    def _make_plot(self, title, series):
        """Build a styled time-series plot with N labelled curves."""
        plot = pg.PlotWidget(title=title)
        plot.showGrid(x=True, y=True, alpha=0.4)
        plot.getPlotItem().titleLabel.item.setHtml(
            f"<span style='color:{C_WHITE};font-weight:bold;font-size:9pt'>{title}</span>")
        legend = plot.addLegend(offset=(-5, 5))
        curves = []
        for label, col in series:
            c = plot.plot(pen=pg.mkPen(col, width=1), name=label)
            curves.append(c)
        return plot, curves if len(curves) > 1 else curves[0]

    def _wrap(self, widget, title, grid, row, col):
        """Wrap a widget in a titled bordered frame."""
        frame = QtWidgets.QFrame()
        frame.setFrameStyle(QtWidgets.QFrame.Box)
        frame.setStyleSheet(f'border: 1px solid {C_GRID}; color: {C_WHITE};')
        v = QtWidgets.QVBoxLayout(frame)
        v.setContentsMargins(4, 2, 4, 4); v.setSpacing(2)
        title_lbl = QtWidgets.QLabel(title)
        title_lbl.setStyleSheet(f'color:{C_WHITE}; font-weight:bold; '
                                 f'font-family:Consolas; font-size:10pt; border:none;')
        v.addWidget(title_lbl); v.addWidget(widget)
        grid.addWidget(frame, row, col)

    # -------------------------------------------------------------------------
    def refresh(self):
        data = self.reader.snapshot()
        if data.shape[0] < 2:
            if not self.reader.connected:
                self.readout.setText('NO SERIAL CONNECTION')
            return

        # Trim to last WINDOW_S seconds
        t_now = data[-1, 0]
        mask  = data[:, 0] >= (t_now - WINDOW_S)
        data  = data[mask]

        t        = data[:, 0]
        pos      = data[:, 1:4]      # NED
        vel      = data[:, 4:7]
        alt      = data[:, 11]
        clb      = data[:, 12]
        spd      = data[:, 13]
        hdg      = data[:, 14]
        pitch_d  = data[:, 15]
        roll_d   = data[:, 16]
        yaw_r    = data[:, 17]
        baro     = data[:, 20]

        # ── 3D trajectory  (NED → plot frame: X=East, Y=North, Z=Up) ─────────
        traj = np.column_stack((pos[:, 1], pos[:, 0], -pos[:, 2])).astype(np.float32)
        self.traj_line.setData(pos=traj)
        self.pos_dot.setData(pos=traj[-1:].copy())

        # ── Time series ──────────────────────────────────────────────────────
        self.alt_curve.setData(t, alt)
        self.vel_curves[0].setData(t, vel[:, 0])
        self.vel_curves[1].setData(t, vel[:, 1])
        self.vel_curves[2].setData(t, vel[:, 2])
        self.ori_curves[0].setData(t, roll_d)
        self.ori_curves[1].setData(t, pitch_d)
        self.ori_curves[2].setData(t, np.degrees(yaw_r))

        # ── Ground track ─────────────────────────────────────────────────────
        self.gt_curve.setData(pos[:, 1], pos[:, 0])
        self.gt_marker.setData([pos[-1, 1]], [pos[-1, 0]])

        # ── Readout ──────────────────────────────────────────────────────────
        self.max_alt = max(self.max_alt, float(alt[-1]))
        apogee_line  = (f'MAX ALT:    {self.max_alt:7.1f} ft\n'
                        if self.max_alt > alt[-1] + 5 else '')
        text = (
            f"{apogee_line}"
            f"ALT:        {alt[-1]:7.1f} ft\n"
            f"AIRSPEED:   {spd[-1]:7.1f} kts\n"
            f"HEADING:    {hdg[-1]%360:07.1f}°\n"
            f"CLIMB RATE: {clb[-1]:7.1f} ft/min\n"
            f"BARO:       {baro[-1]:7.0f} Pa\n"
            f"\n"
            f"t = {t[-1]:7.3f} s\n"
            f"\n"
            f"ATTITUDE\n"
            f"  ROLL:  {roll_d[-1]:7.2f}°\n"
            f"  PITCH: {pitch_d[-1]:7.2f}°\n"
            f"  YAW:   {np.degrees(yaw_r[-1]):7.2f}°\n"
            f"\n"
            f"POS NED  N:{pos[-1,0]:7.1f}\n"
            f"         E:{pos[-1,1]:7.1f}\n"
            f"         D:{pos[-1,2]:7.1f}"
        )
        self.readout.setText(text)

    # -------------------------------------------------------------------------
    def closeEvent(self, e):
        self.reader.stop()
        e.accept()


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='ARCUIA Live Flight HUD')
    ap.add_argument('--port', '-p', default='COM3',
                     help='Serial port (e.g. COM3, /dev/ttyACM0)')
    ap.add_argument('--baud', '-b', type=int, default=1000000,
                     help='Baud rate (Teensy USB ignores this but pyserial requires it)')
    ap.add_argument('--buffer', type=int, default=5000,
                     help='Sample buffer (5000 = 10 s @ 500 Hz)')
    args = ap.parse_args()

    reader = SerialReader(args.port, args.baud, args.buffer)
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    hud = HUD(reader)
    hud.resize(1600, 900)
    hud.show()

    code = app.exec_()
    reader.stop()
    sys.exit(code)


if __name__ == '__main__':
    main()
