import argparse
import sys
import time
from collections import deque

import serial  # pip install pyserial
import matplotlib.pyplot as plt  # pip install matplotlib
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import FuncFormatter


def parse_args():
    p = argparse.ArgumentParser(description="Realtime UART CSV plotter: tick(ms),temp(C),press(hPa)")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--window", type=int, default=120, help="Window in seconds to display")
    p.add_argument("--smooth", type=float, default=0.2, help="EMA smoothing factor 0..0.95 (0=off)")
    p.add_argument("--max-points", type=int, default=2000, help="Max points to keep in memory")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        # Clear any stale bytes so we start on a line boundary
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Failed to open serial {args.port}: {e}")
        sys.exit(1)

    ticks = deque(maxlen=args.max_points)
    temps = deque(maxlen=args.max_points)
    press = deque(maxlen=args.max_points)
    parsed_count = 0

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    line_temp, = ax1.plot([], [], 'r-', label='Temp (C)')
    line_press, = ax2.plot([], [], 'b-', label='Pressure (hPa)')

    ax1.set_xlabel('Time (mm:ss)')
    ax1.set_ylabel('Temp (C)', color='r')
    ax2.set_ylabel('Pressure (hPa)', color='b')
    ax1.grid(True)
    ax1.set_autoscaley_on(True)
    ax2.set_autoscaley_on(True)
    fig.legend(loc='upper left')

    start_wall = time.time()
    last_text = ax1.text(0.01, 0.95, '', transform=ax1.transAxes)

    # Format X as mm:ss for readability
    def _fmt_time(x, _pos):
        if x < 0:
            x = 0
        m = int(x // 60)
        s = int(x % 60)
        return f"{m:02d}:{s:02d}"
    ax1.xaxis.set_major_formatter(FuncFormatter(_fmt_time))

    def update(_frame):
        nonlocal parsed_count
        # Read full lines available right now
        read_any = False
        max_reads = max(10, (ser.in_waiting or 0) // 8)  # cap per frame to keep UI responsive
        for _ in range(max_reads):
            try:
                raw_line = ser.readline()
            except Exception:
                break
            if not raw_line:
                break
            read_any = True
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 3:
                continue
            try:
                tick_ms = float(parts[0])
                t = float(parts[1])
                p = float(parts[2])
            except ValueError:
                continue

            ticks.append(tick_ms / 1000.0)
            # Optional exponential moving average smoothing for visual stability
            if 0 < args.smooth < 0.95 and temps:
                t = args.smooth * t + (1.0 - args.smooth) * temps[-1]
                p = args.smooth * p + (1.0 - args.smooth) * press[-1]
            temps.append(t)
            press.append(p)
            last_text.set_text(f"last: t={t:.2f}C  p={p:.2f}hPa")
            parsed_count += 1

        if not ticks:
            # Provide initial visible ranges so window isn't empty
            ax1.set_xlim(0, 10)
            ax1.set_ylim(0, 1)
            ax2.set_ylim(0, 1)
            return line_temp, line_press, last_text

        # Limit x-window
        tmax = ticks[-1]
        tmin = max(0.0, tmax - args.window)
        # Build visible subset indices
        i0 = 0
        for i in range(len(ticks)):
            if ticks[i] >= tmin:
                i0 = i
                break

        x = list(ticks)[i0:]
        y1 = list(temps)[i0:]
        y2 = list(press)[i0:]

        line_temp.set_data(x, y1)
        line_press.set_data(x, y2)

        ax1.set_xlim(max(0.0, tmin), tmax if tmax > 1 else 1)
        if y1:
            ymin = min(y1)
            ymax = max(y1)
            yr = ymax - ymin
            if yr < 0.01:
                m = (ymin + ymax) / 2.0
                ymin = m - 0.5
                ymax = m + 0.5
            else:
                pad = max(0.1, yr * 0.1)
                ymin -= pad
                ymax += pad
            ax1.set_ylim(ymin, ymax)
        if y2:
            pmin = min(y2)
            pmax = max(y2)
            pr = pmax - pmin
            if pr < 0.1:
                m = (pmin + pmax) / 2.0
                pmin = m - 1.0
                pmax = m + 1.0
            else:
                pad = max(0.5, pr * 0.05)
                pmin -= pad
                pmax += pad
            ax2.set_ylim(pmin, pmax)

        # update window title with count
        try:
            fig.canvas.manager.set_window_title(f"UART Realtime Plot - {parsed_count} samples")
        except Exception:
            pass

        return line_temp, line_press, last_text

    ani = FuncAnimation(fig, update, interval=120, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()


