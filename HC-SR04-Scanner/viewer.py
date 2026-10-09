"""Live polar return map. pip install pyserial matplotlib
python viewer.py --port COM5
python viewer.py --port /dev/cu.usbmodemXXXX
python viewer.py --demo  # explicitly synthetic data, no hardware
"""
import argparse
import json
import math
import time
from pathlib import Path


class ScanMap:
    def __init__(self):
        self.bins = {}
        self.epoch = None
        self.max_cm = 250
        self.paused = False
        self.latest_bearing = 0

    def ingest(self, msg, now=None):
        now = time.monotonic() if now is None else now
        kind = msg.get('type')
        if kind == 'reset':
            self.bins.clear()
            self.epoch = msg['epoch']
            self.max_cm = msg.get('max_cm', 250)
        elif kind == 'status':
            self.paused = bool(msg['paused'])
        elif kind == 'point':
            if self.epoch != msg['epoch']:
                self.bins.clear()
                self.epoch = msg['epoch']
            bearing = float(msg['bearing'])
            distance = float(msg['mm']) / 10
            if not -90 <= bearing <= 90 or distance < 0:
                return
            self.latest_bearing = bearing
            b = dict(msg)
            b['received_at'] = now
            b['cm'] = distance
            # x right, y forward. A return lies somewhere in the sonar beam;
            # plotting its central bearing is only an approximation.
            b['x_cm'] = distance * math.sin(math.radians(bearing))
            b['y_cm'] = distance * math.cos(math.radians(bearing))
            self.bins[int(msg['angle'])] = b

    def export(self, filename):
        now = time.monotonic()
        records = []
        for angle in sorted(self.bins):
            b = dict(self.bins[angle])
            b['age_since_receipt_s'] = round(now - b.pop('received_at'), 3)
            if not b['valid']:
                b['cm'] = b['x_cm'] = b['y_cm'] = None
            records.append(b)
        Path(filename).write_text(json.dumps({
            'description': 'Stationary local polar sonar returns; unknown is not free space',
            'coordinates': 'cm; x right, y forward; origin at sensor',
            'epoch': self.epoch, 'bins': records,
        }, indent=2), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Serial port; omitted to list ports')
    parser.add_argument('--demo', action='store_true', help='Synthetic data only')
    parser.add_argument('--stale-seconds', type=float, default=15)
    parser.add_argument('--snapshot', help='Save synthetic preview PNG and exit (requires --demo)')
    args = parser.parse_args()
    if args.snapshot and not args.demo:
        parser.error('--snapshot requires --demo')
    if args.stale_seconds <= 0:
        parser.error('--stale-seconds must be positive')
    if not args.demo and not args.port:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            print(p.device, p.description)
        if not ports:
            print('No serial ports found. Plug in the UNO using a data-capable USB cable.')
        print('Run again with --port PORT, or use --demo.')
        return

    import matplotlib
    if args.snapshot:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button
    serial_port = None
    if not args.demo:
        import serial
        try:
            serial_port = serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2)
        except serial.SerialException as exc:
            raise SystemExit(f'Cannot open {args.port}: {exc}. Close Arduino Serial Monitor.')

    model = ScanMap()
    pending = bytearray()
    sequence = list(range(10, 171, 10)) + list(range(160, 10, -10))
    demo_index = 0
    last_demo = 0.0
    connection_error = ''
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(10, 7))
    fig.subplots_adjust(bottom=0.22)
    fig.canvas.manager.set_window_title('HC-SR04 local scan')

    def command(value):
        nonlocal connection_error
        if serial_port:
            try:
                serial_port.write(value.encode('ascii'))
            except Exception as exc:
                connection_error = str(exc)
        else:
            model.paused = value == 'p'
            if value in ('c', 'r'):
                model.bins.clear()

    buttons = []
    for x, title, callback in [
        (0.12, 'Pause', lambda _: command('p')),
        (0.32, 'Resume + clear', lambda _: command('r')),
        (0.55, 'Clear', lambda _: command('c')),
        (0.73, 'Export JSON', lambda _: model.export('local_scan.json')),
    ]:
        b = Button(fig.add_axes([x, 0.06, 0.16, 0.055]), title, color='#263546', hovercolor='#405570')
        b.on_clicked(callback)
        buttons.append(b)
    fig.text(0.5, 0.015, 'Return arcs = uncertain bearing  |  x = invalid echo  |  grey = stale  |  blank = unobserved',
             ha='center', fontsize=9)

    def demo_point(angle, now):
        # An artificial wall plus a nearby obstacle. Never presented as real data.
        bearing = 90 - angle
        distance = min(220, 115 / max(math.cos(math.radians(bearing)), 0.01))
        if 30 <= angle <= 50:
            distance = 65
        model.ingest({'type': 'point', 'epoch': 1, 'angle': angle, 'bearing': bearing,
                      'mm': round(distance * 10), 'valid': angle != 130,
                      'samples': 0 if angle == 130 else 3, 'sweep': 0, 'ms': 0}, now)

    def update(_frame):
        nonlocal pending, demo_index, last_demo, connection_error
        now = time.monotonic()
        if serial_port and not connection_error:
            try:
                pending.extend(serial_port.read(min(serial_port.in_waiting, 8192)))
                while b'\n' in pending:
                    raw, _, pending = pending.partition(b'\n')
                    try:
                        model.ingest(json.loads(raw), now)
                    except (ValueError, KeyError, TypeError):
                        pass  # Ignore startup noise/incomplete first record.
                if len(pending) > 65536:
                    pending.clear()
            except Exception as exc:
                connection_error = str(exc)
        elif args.demo and not model.paused and now - last_demo >= 0.3:
            demo_point(sequence[demo_index % len(sequence)], now)
            demo_index += 1
            last_demo = now

        ax.clear()
        limit = model.max_cm
        ax.set(xlim=(-limit - 10, limit + 10), ylim=(-15, limit + 15),
               xlabel='Left / right (cm)', ylabel='Forward (cm)', aspect='equal')
        for radius in range(50, limit + 1, 50):
            angles = range(-90, 91)
            ax.plot([radius * math.sin(math.radians(a)) for a in angles],
                    [radius * math.cos(math.radians(a)) for a in angles], color='#263546', lw=0.8)
            ax.text(2, radius, f'{radius} cm', color='#728399', fontsize=8)
        fresh_valid = fresh_invalid = 0
        for b in model.bins.values():
            stale = now - b['received_at'] > args.stale_seconds
            color = '#657080' if stale else ('#42e3af' if b['valid'] else '#f5ad66')
            if not b['valid']:
                angle = math.radians(b['bearing'])
                ax.plot(limit * math.sin(angle), limit * math.cos(angle), marker='x', color=color)
                fresh_invalid += not stale
                continue
            fresh_valid += not stale
            ax.plot(b['x_cm'], b['y_cm'], '.', color=color, ms=7)
            # Illustrative 15-degree total beam only; actual response depends on target.
            angles = [b['bearing'] - 7.5 + i * 0.5 for i in range(31)]
            ax.plot([b['cm'] * math.sin(math.radians(a)) for a in angles],
                    [b['cm'] * math.cos(math.radians(a)) for a in angles], color=color, alpha=0.45)
        angle = math.radians(model.latest_bearing)
        ax.plot([0, limit * math.sin(angle)], [0, limit * math.cos(angle)], color='#629eff', alpha=0.45)
        ax.plot(0, 0, '^', ms=11, color='white')
        label = 'DEMO / SYNTHETIC DATA' if args.demo else args.port
        state = 'PAUSED' if model.paused else 'SCANNING'
        if connection_error:
            state = 'DISCONNECTED: ' + connection_error[:80]
        ax.set_title(f'Local obstacle scan | {label}\n{state} | fresh returns: {fresh_valid}, invalid: {fresh_invalid}', fontsize=11)
        return []

    if args.snapshot:
        now = time.monotonic()
        for angle in range(10, 171, 10):
            demo_point(angle, now)
        last_demo = now
        update(0)
        fig.savefig(args.snapshot, dpi=120)
    else:
        animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        try:
            plt.show()
        finally:
            if serial_port:
                serial_port.close()


if __name__ == '__main__':
    main()
