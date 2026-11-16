# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib"]
# ///
"""plot_flight.py

Plots acceleration (LSM9DS1) and altitude (BMP390) for the flight in a log.

Reads a raw image from tools/swd_dump.sh (.bin) or a capture from
flight_reader (.hex, one page per line). Altitude needs the BMP390
calibration line printed by sensor_test, as hex or a file holding it.

Usage:
  uv run tools/plot_flight.py LOG --calib CALIB [--out flight.png]
                              [--before 5] [--after SECONDS]

Time zero is liftoff, the start of the first stretch above 1.5 g that
reaches 3 g, near launch detection. Ground level is the median pressure over
the 10 s before. Altitude is median filtered over 0.5 s so ejection charge
pressure spikes don't count as apogee; the raw trace is drawn faintly behind.
Also checks the accelerometer scale against gravity on the pad.
"""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

PAGE = 256
SAMPLE = 22
SAMPLES_PER_PAGE = 11
FORMAT = 0xA2
MG_PER_LSB = 0.732          # accel at the "+/-16 g" setting, spans about +/-24 g
COUNTS_PER_G = 1000 / MG_PER_LSB
CLIP = 32760                # raw counts at or past this are at the limit
FT_PER_M = 3.28084


# --- reading the log --------------------------------------------------------

def load_pages(path):
    """Return the log as bytes, 256 per page, from a .bin or .hex file."""
    if path.suffix.lower() == ".bin":
        return path.read_bytes()
    pages = []
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if len(line) == 2 * PAGE:
            try:
                pages.append(bytes.fromhex(line))
            except ValueError:
                pass  # garbled line from the UART
    return b"".join(pages)


def parse(data):
    """Decode written pages into per-sample arrays."""
    t, acc, p_raw, t_raw, rate = [], [], [], [], []
    expected = None
    for off in range(0, len(data) - PAGE + 1, PAGE):
        page = data[off:off + PAGE]
        seq = struct.unpack_from("<I", page, 252)[0]
        if seq == 0xFFFFFFFF:
            break
        if page[251] != FORMAT:
            sys.exit(f"page {off // PAGE}: format {page[251]:#x}, expected {FORMAT:#x}")
        if expected is not None and seq != expected:
            print(f"warning: sequence jumps from {expected - 1} to {seq}")
        expected = seq + 1
        for i in range(SAMPLES_PER_PAGE):
            s = i * SAMPLE
            ts, ax, ay, az = struct.unpack_from("<Ihhh", page, s)
            t.append(ts)
            acc.append((ax, ay, az))
            p_raw.append(int.from_bytes(page[s + 16:s + 19], "little"))
            t_raw.append(int.from_bytes(page[s + 19:s + 22], "little"))
            rate.append(page[248])
        last = page
    if expected is None:
        sys.exit("no written pages in the log")
    missed, errors = struct.unpack_from("<IH", last, 242)
    print(f"{expected} pages, {missed} missed deadlines, {errors} errors")

    # timestamps are microseconds since boot and wrap every 2^32 us
    t = np.array(t, dtype=np.int64)
    wraps = np.concatenate(([0], np.cumsum(np.diff(t) < 0)))
    t = (t + wraps * 2**32) / 1e6
    return (t, np.array(acc, float) * MG_PER_LSB / 1000,
            np.array(p_raw, float), np.array(t_raw, float), np.array(rate))


# --- barometer ----------------------------------------------------------------

def bmp390_pressure(calib_hex, p_raw, t_raw):
    """Compensated pressure in Pa, from the BMP390 datasheet's float formulas."""
    raw = bytes.fromhex(calib_hex)
    if len(raw) != 21:
        sys.exit(f"calibration should be 21 bytes, got {len(raw)}")
    (t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11) = \
        struct.unpack("<HHbhhbbHHbbhbb", raw)
    pt1, pt2, pt3 = t1 * 2.0**8, t2 / 2.0**30, t3 / 2.0**48
    pp1, pp2 = (p1 - 2**14) / 2.0**20, (p2 - 2**14) / 2.0**29
    pp3, pp4 = p3 / 2.0**32, p4 / 2.0**37
    pp5, pp6, pp7, pp8 = p5 * 2.0**3, p6 / 2.0**6, p7 / 2.0**8, p8 / 2.0**15
    pp9, pp10, pp11 = p9 / 2.0**48, p10 / 2.0**48, p11 / 2.0**65

    d = t_raw - pt1
    tc = d * pt2 + d * d * pt3                       # deg C
    out1 = pp5 + pp6 * tc + pp7 * tc**2 + pp8 * tc**3
    out2 = p_raw * (pp1 + pp2 * tc + pp3 * tc**2 + pp4 * tc**3)
    out3 = p_raw**2 * (pp9 + pp10 * tc) + p_raw**3 * pp11
    return out1 + out2 + out3, tc


def altitude(p, p0):
    """Height above the pressure p0, in metres, standard atmosphere."""
    return 44330.0 * (1.0 - (p / p0) ** (1 / 5.255))


# --- finding the flight -----------------------------------------------------

def find_liftoff(t, g, rate):
    """Liftoff time and launch detection time for the first flight.

    The rate byte is per page, so the first flight-rate page can start with
    ground-rate samples. Detection is the first 10 ms gap after that.
    """
    flight = np.flatnonzero(rate == 1)
    if flight.size == 0:
        sys.exit("the log never switched to flight rate, so no launch was detected")
    gaps = np.diff(t[flight[0]:])
    detect = flight[0] + 1 + int(np.argmax(gaps < 0.05))
    # first reading above 3 g within 2 s before detection, then back over 1.5 g
    near = np.flatnonzero((t >= t[detect] - 2.0) & (t <= t[detect] + 0.5) & (g > 3.0))
    i = near[0] if near.size else detect
    while i > 0 and g[i - 1] > 1.5:
        i -= 1
    return t[i], t[detect]


def rolling_median(x, n):
    """Centered median over n samples, n odd; NaN where the window has NaN."""
    from numpy.lib.stride_tricks import sliding_window_view
    pad = np.pad(x, n // 2, mode="edge")
    return np.median(sliding_window_view(pad, n), axis=1)


def find_landing(t, h, start):
    """First time after apogee that altitude holds still near the ground."""
    after = np.flatnonzero(t >= start)
    peak = after[np.argmax(h[after])]
    ground = max(10.0, 0.05 * h[peak])
    for i in range(peak, len(t)):
        if h[i] > ground:
            continue
        j = np.searchsorted(t, t[i] + 5.0)
        if j >= len(t):
            break
        if np.ptp(h[i:j]) < 2.0:
            return t[i]
    return None


# --- main ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help=".bin from swd_dump.sh or .hex from flight_reader")
    ap.add_argument("--calib", required=True,
                    help="BMP390 calibration from sensor_test, as hex or a file holding it")
    ap.add_argument("--out", type=Path, help="image to write, default LOG with .png")
    ap.add_argument("--before", type=float, default=5.0, help="seconds shown before liftoff")
    ap.add_argument("--after", type=float, help="seconds shown after liftoff, default to landing")
    ap.add_argument("--show", action="store_true", help="also open a window")
    args = ap.parse_args()

    calib = args.calib
    if Path(calib).is_file():
        calib = Path(calib).read_text().strip()

    t, acc, p_raw, t_raw, rate = parse(load_pages(args.log))
    g = np.linalg.norm(acc, axis=1)
    t0, t_detect = find_liftoff(t, g, rate)

    # scale check: at rest the magnitude should be 1 g, 1366 counts
    rest = (t < t0 - 2.0)
    counts = np.median(g[rest]) * COUNTS_PER_G
    print(f"pad gravity {counts:.0f} counts = {counts / COUNTS_PER_G:.3f} g "
          f"({counts / COUNTS_PER_G * 100 - 100:+.1f}%), "
          f"expect about 1366 for +/-24 g span, 2048 if it were a true +/-16 g")

    pressure, temp_c = bmp390_pressure(calib, p_raw, t_raw)
    pad = (t >= t0 - 10.0) & (t < t0) & (p_raw > 0)
    if not pad.any():
        sys.exit("no barometer readings in the 10 s before liftoff")
    p0 = np.median(pressure[pad])
    h_raw = np.where(p_raw > 0, altitude(pressure, p0), np.nan)
    h = rolling_median(np.nan_to_num(h_raw), 51)    # 0.5 s at 100 Hz

    if args.after is not None:
        t_end = t0 + args.after
    else:
        landing = find_landing(t, h, t0)
        t_end = (landing + 5.0) if landing else t[rate == 1][-1]

    w = (t >= t0 - args.before) & (t <= t_end)
    x = t[w] - t0
    hw, hr, gw = h[w], h_raw[w], g[w]
    flight = x >= 0
    i_apo = np.nanargmax(np.where(flight, hw, -np.inf))
    boost = flight & (x < x[i_apo])
    i_max = np.argmax(np.where(boost, gw, -np.inf))
    i_hit = np.argmax(np.where(x > x[i_apo], gw, -np.inf))
    clipped = (np.abs(acc[w]) * COUNTS_PER_G >= CLIP).any(axis=1)

    print(f"ground pressure {p0 / 100:.2f} hPa, {np.median(temp_c[pad]):.1f} C")
    print(f"liftoff at {t0:.3f} s since boot, launch detected {t_detect - t0:.2f} s later")
    print(f"max boost acceleration {gw[i_max]:.1f} g at {x[i_max]:.2f} s")
    print(f"largest after apogee {gw[i_hit]:.1f} g at {x[i_hit]:.2f} s"
          + (", clipped at the accel limit" if clipped[i_hit] else ""))
    if clipped.any():
        print(f"{int(clipped.sum())} samples have an axis at the accel limit")
    print(f"apogee {hw[i_apo]:.1f} m ({hw[i_apo] * FT_PER_M:.0f} ft) at {x[i_apo]:.2f} s")
    if args.after is None and t_end != t[rate == 1][-1]:
        print(f"landed about {t_end - 5.0 - t0:.1f} s after liftoff")

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7.5),
                                   constrained_layout=True)
    fig.suptitle(f"{args.log.name}: acceleration and altitude")

    for k, name, color in ((0, "X", "#d97706"), (1, "Y", "#7c3aed"), (2, "Z", "#0891b2")):
        ax1.plot(x, acc[w, k], lw=0.6, alpha=0.55, color=color, label=name)
    ax1.plot(x, gw, lw=1.1, color="#111827", label="|a|")
    ax1.axhline(3.0, ls=":", lw=0.8, color="#9ca3af")
    ax1.annotate(f"boost {gw[i_max]:.1f} g", (x[i_max], gw[i_max]),
                 xytext=(8, 0), textcoords="offset points", fontsize=9)
    if clipped.any():
        ax1.plot(x[clipped], gw[clipped], "x", color="#dc2626", ms=5, label="clipped")
    ax1.set_ylabel("acceleration (g)")
    ax1.legend(loc="upper left", ncol=5, fontsize=8, frameon=False)
    ax1.grid(alpha=0.3)

    ax2.plot(x, hr, lw=0.6, alpha=0.35, color="#93c5fd", label="raw")
    ax2.plot(x, hw, lw=1.4, color="#2563eb", label="0.5 s median")
    span = np.nanmax(hw) - np.nanmin(hw)
    ax2.set_ylim(np.nanmin(hw) - 0.1 * span, np.nanmax(hw) + 0.15 * span)
    ax2.legend(loc="upper right", fontsize=8, frameon=False)
    ax2.plot(x[i_apo], hw[i_apo], "o", color="#dc2626", ms=5)
    ax2.annotate(f"apogee {hw[i_apo]:.0f} m / {hw[i_apo] * FT_PER_M:.0f} ft\nat {x[i_apo]:.1f} s",
                 (x[i_apo], hw[i_apo]), xytext=(8, 6), textcoords="offset points", fontsize=9)
    ax2.set_ylabel("altitude above pad (m)")
    ax2.set_xlabel("time since liftoff (s)")
    ax2.grid(alpha=0.3)
    ft = ax2.secondary_yaxis("right", functions=(lambda m: m * FT_PER_M, lambda f: f / FT_PER_M))
    ft.set_ylabel("ft")

    for ax in (ax1, ax2):
        ax.axvline(0, lw=0.8, color="#16a34a")
        ax.axvline(t_detect - t0, lw=0.8, ls="--", color="#16a34a")
    ax1.text(t_detect - t0, ax1.get_ylim()[1], " detected", va="top", fontsize=8, color="#16a34a")

    out = args.out or args.log.with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
