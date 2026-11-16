# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib"]
# ///
"""plot_attitude.py

Combines the gyro and accelerometer to show which way the rocket was really
accelerating, in a ground frame instead of the sensor's own frame.

  1. Gyro bias is the median over the 30 s before liftoff, when it sat still.
  2. The starting attitude lines the pad gravity reading up with vertical.
  3. The gyro is integrated sample by sample to track rotation from there.
  4. Each accel reading is rotated into the ground frame and gravity removed.

Heading is relative to the pad, since there is no magnetometer in the log, so
"horizontal" is a size without a compass direction. Gyro drift adds up, so
trust it most early in the flight. As a check, vertical acceleration is
integrated once and twice and compared with the barometer, differentiated
for speed, up to ejection.

Usage:
  uv run tools/plot_attitude.py LOG --calib CALIB [--out FILE] [--after S] [--show]
"""

import argparse
from pathlib import Path

import numpy as np

import plot_flight as pf

G = 9.80665


def rotation(v):
    """Rotation matrix for a rotation vector v (radians), Rodrigues formula."""
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def align_to_up(f):
    """Rotation taking the body vector f onto world +Z."""
    f = f / np.linalg.norm(f)
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(f, z)
    s, c = np.linalg.norm(axis), np.dot(f, z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    return rotation(axis / s * np.arctan2(s, c))


def smooth(x, n):
    """Centered moving average over n samples, n odd, edges held."""
    pad = np.pad(x, n // 2, mode="edge")
    return np.convolve(pad, np.ones(n) / n, mode="valid")


def integrate(t, x):
    """Cumulative trapezoid integral, starting at zero."""
    return np.concatenate(([0.0], np.cumsum(np.diff(t) * (x[1:] + x[:-1]) / 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--before", type=float, default=2.0)
    ap.add_argument("--after", type=float, default=40.0)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    calib = args.calib
    if Path(calib).is_file():
        calib = Path(calib).read_text().strip()

    t, acc, p_raw, t_raw, rate, gyro = pf.parse(pf.load_pages(args.log), with_gyro=True)
    g = np.linalg.norm(acc, axis=1)
    t0, t_detect = pf.find_liftoff(t, g, rate)

    # pad: bias and starting attitude
    pad = (t >= t0 - 30.0) & (t < t0 - 1.0)
    bias = np.median(gyro[pad], axis=0)
    f_pad = np.median(acc[pad], axis=0)
    nose = np.zeros(3)
    nose[np.argmax(np.abs(f_pad))] = np.sign(f_pad[np.argmax(np.abs(f_pad))])
    axis = int(np.argmax(np.abs(nose)))
    print(f"gyro bias {np.round(bias, 2)} deg/s, nose is body "
          f"{'-' if nose[axis] < 0 else '+'}{'XYZ'[axis]}")
    print(f"pad tilt {np.degrees(np.arccos(abs(f_pad @ nose) / np.linalg.norm(f_pad))):.1f} deg "
          f"from vertical")

    # integrate attitude from 1 s before liftoff
    w = np.flatnonzero((t >= t0 - 1.0) & (t <= t0 + args.after))
    tw = t[w]
    omega = np.radians(gyro[w] - bias)
    R = np.empty((len(w), 3, 3))
    R[0] = align_to_up(f_pad)
    for k in range(1, len(w)):
        step = (omega[k] + omega[k - 1]) / 2 * (tw[k] - tw[k - 1])
        R[k] = R[k - 1] @ rotation(step)
    f_world = np.einsum("kij,kj->ki", R, acc[w])
    a_world = f_world - np.array([0.0, 0.0, 1.0])          # g, gravity removed
    up = a_world[:, 2]
    horiz = np.linalg.norm(a_world[:, :2], axis=1)
    nose_world = np.einsum("kij,j->ki", R, nose)
    tilt = np.degrees(np.arccos(np.clip(nose_world[:, 2], -1, 1)))
    x = tw - t0

    # barometer for the check
    pressure, _ = pf.bmp390_pressure(calib, p_raw, t_raw)
    p0 = np.median(pressure[(t >= t0 - 10.0) & (t < t0) & (p_raw > 0)])
    h_full = pf.rolling_median(np.nan_to_num(pf.altitude(pressure, p0)), 51)
    h_baro = h_full[w]
    i_apo = np.argmax(h_baro)
    # differentiate on an even 100 Hz grid, since the pad was logged at 10 Hz
    grid = np.arange(tw[0], tw[-1], 0.01)
    v_grid = smooth(np.gradient(np.interp(grid, tw, h_baro), grid), 51)  # 0.5 s
    v_baro = np.interp(tw, grid, v_grid)

    # dead reckoning up to the first shock after apogee, about ejection
    shock = np.flatnonzero((np.arange(len(w)) > i_apo) & (g[w] > 5.0))
    i_end = shock[0] if shock.size else len(w) - 1
    v = integrate(tw[:i_end], up[:i_end] * G)
    h_imu = integrate(tw[:i_end], v)

    boost = (x >= 0) & (x < 1.5)
    i_peak = np.flatnonzero(boost)[np.argmax(np.linalg.norm(a_world[boost], axis=1))]
    i_vmax = np.argmax(v)
    i_top = np.argmax(h_imu)
    print(f"peak boost: {up[i_peak]:.1f} g up, {horiz[i_peak]:.1f} g sideways, "
          f"tilt {tilt[i_peak]:.1f} deg, at {x[i_peak]:.2f} s")
    print(f"max vertical speed {v[i_vmax]:.0f} m/s ({v[i_vmax] * 3.28084:.0f} ft/s) "
          f"at {x[i_vmax]:.2f} s")
    print(f"IMU apogee {h_imu[i_top]:.0f} m at {x[i_top]:.2f} s; "
          f"barometer {h_baro[i_apo]:.0f} m at {x[i_apo]:.2f} s")
    after = np.arange(len(w)) > i_apo
    desc = after & (h_baro > 0.2 * h_baro[i_apo]) & (h_baro < 0.8 * h_baro[i_apo])
    # barometer speed during boost is unreliable, so only the descent is reported
    print(f"barometer descent {-np.median(v_baro[desc]):.1f} m/s under the chute")
    print(f"tilt at IMU apogee {tilt[i_top]:.0f} deg; dead reckoning stops at {x[i_end]:.2f} s")

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    show = x >= -args.before
    # altitude, velocity, ground-frame acceleration, tilt, gyro
    # altitude, speed, ground-frame acceleration, nose tilt; raw gyro rates
    # are on the --raw chart from plot_flight.py
    fig, (ax3, ax5, ax1, ax2) = plt.subplots(
        4, 1, sharex=True, figsize=(11, 11.5), constrained_layout=True,
        gridspec_kw={"height_ratios": [3, 2.5, 2.5, 1.6]})
    fig.suptitle(f"{args.log.name}: acceleration in the ground frame (gyro + accel)")

    ax1.plot(x[show], up[show], lw=1.0, color="#2563eb", label="vertical (up +)")
    ax1.plot(x[show], horiz[show], lw=1.0, color="#d97706", label="horizontal")
    ax1.axhline(0, lw=0.6, color="#6b7280")
    ax1.axhline(-1, lw=0.6, ls=":", color="#6b7280", label="free fall (-1 g)")
    ax1.set_ylabel("acceleration\n(g, gravity removed)")
    ax1.grid(alpha=0.3)

    ax2.plot(x[show], tilt[show], lw=1.0, color="#7c3aed")
    ax2.set_ylabel("nose tilt\nfrom vertical (deg)")
    ax2.set_ylim(0, 180)
    ax2.set_yticks([0, 30, 60, 90, 120, 150, 180])
    ax2.grid(alpha=0.3)

    xs = x[:i_end]
    ax3.plot(x[show], h_baro[show], lw=1.2, color="#2563eb", label="barometer")
    ax3.plot(xs, h_imu, lw=1.2, ls="--", color="#dc2626", label="IMU, vertical accel integrated twice")
    ax3.set_ylabel("altitude\nabove pad (m)")
    ax3.grid(alpha=0.3)
    ax3.legend(loc="upper right", fontsize=8, frameon=False)

    ax5.plot(x[show], v_baro[show], lw=1.2, color="#2563eb",
             label="barometer, altitude differentiated")
    ax5.plot(xs, v, lw=1.2, ls="--", color="#dc2626",
             label="IMU, vertical accel integrated once")
    ax5.axhline(0, lw=0.6, color="#6b7280")
    lo = min(v.min(), v_baro[show].min())
    hi = max(v.max(), v_baro[show].max())
    ax5.set_ylim(lo - 0.05 * (hi - lo), hi + 0.08 * (hi - lo))
    # the barometer reads wrong while air rushes past it in boost
    early = np.flatnonzero(show & (x >= 0) & (x < 2.0))
    i_spike = early[np.argmax(v_baro[early])]
    if v_baro[i_spike] > 1.2 * v.max():
        ax5.annotate("pressure disturbance in boost,\nnot real speed",
                     (x[i_spike], v_baro[i_spike]), xytext=(14, -6),
                     textcoords="offset points", va="top", fontsize=8, color="#2563eb")
    ax5.set_ylabel("vertical speed\n(m/s)")
    ax5.legend(loc="upper right", fontsize=8, frameon=False)
    ax5.grid(alpha=0.3)

    ax2.set_xlabel("time since liftoff (s)")

    # flight events: a line through every panel, named in the altitude panel
    landing = pf.find_landing(t, h_full, t0)
    events = [(0.0, "liftoff / launch detected", "#16a34a", "-"),
              (t_detect - t0, "", "#16a34a", "--"),
              (x[i_vmax], "burnout", "#d97706", ":"),
              (x[i_apo], "apogee", "#dc2626", ":"),
              (x[i_end], "ejection", "#6b7280", ":")]
    if landing is not None:
        events.append((landing - t0, "landing", "#6b7280", ":"))
    for ax in (ax1, ax2, ax3, ax5):
        for ex, _, color, ls in events:
            ax.axvline(ex, lw=0.8, ls=ls, color=color)
    for ex, name, color, _ in events:
        if not name:
            continue
        # early events go top left of their line, where the climb leaves room;
        # later ones go bottom right, under the arc
        early = ex < 2.0
        right = not early or name == "burnout"
        ax3.text(ex + (0.15 if right else -0.15), 0.97 if early else 0.1, name,
                 rotation=90, transform=ax3.get_xaxis_transform(),
                 va="top" if early else "bottom", ha="left" if right else "right",
                 fontsize=8, color=color)

    ax3.set_ylim(top=1.12 * h_baro[i_apo])
    ax3.plot(x[i_apo], h_baro[i_apo], "o", color="#dc2626", ms=5)
    ax3.annotate(f"{h_baro[i_apo]:.0f} m / {h_baro[i_apo] * 3.28084:.0f} ft",
                 (x[i_apo], h_baro[i_apo]), xytext=(6, 6), textcoords="offset points", fontsize=9)
    ax3.plot(x[i_top], h_imu[i_top], "o", color="#dc2626", ms=4, mfc="none")
    ax3.annotate(f"IMU {h_imu[i_top]:.0f} m", (x[i_top], h_imu[i_top]),
                 xytext=(0, -8), textcoords="offset points", ha="center", va="top", fontsize=8)
    ax5.plot(x[i_vmax], v[i_vmax], "o", color="#dc2626", ms=4)
    ax5.annotate(f"{v[i_vmax]:.0f} m/s", (x[i_vmax], v[i_vmax]),
                 xytext=(6, 0), textcoords="offset points", fontsize=9)
    # samples with an axis past the rated +/-16 g: shaded where they feed the
    # acceleration and the IMU speed and altitude integrated from it
    over = pf.beyond_rated(acc[w]) & show
    for k, (lo, hi) in enumerate(pf.spans(over, x)):
        for ax in (ax1, ax5):
            ax.axvspan(lo, hi, color="#fdba74", alpha=0.35, lw=0,
                       label="axis past rated +/-16 g" if k == 0 and ax is ax1 else None)
    ax1.annotate(f"boost {up[i_peak]:.1f} g" + ("\npast rated range" if over[i_peak] else ""),
                 (x[i_peak], up[i_peak]),
                 xytext=(8, 0), textcoords="offset points", fontsize=9)

    # accel samples with an axis at the sensor limit
    clipped = np.flatnonzero((np.abs(acc[w]) * pf.COUNTS_PER_G >= pf.CLIP).any(axis=1) & show)
    if clipped.size:
        peak = np.maximum(np.abs(up[clipped]), horiz[clipped])
        ax1.plot(x[clipped], peak, "x", color="#dc2626", ms=6, label="accel axis clipped")
        axes = "".join(sorted({"XYZ"[a] for i in clipped
                               for a in np.flatnonzero(np.abs(acc[w][i]) * pf.COUNTS_PER_G >= pf.CLIP)}))
        ax1.annotate(f"{axes} clipped, true peak higher", (x[clipped[0]], peak.max()),
                     xytext=(-8, -4), textcoords="offset points", ha="right", va="top",
                     fontsize=8, color="#dc2626")
    ax1.legend(loc="upper left", bbox_to_anchor=(0.22, 1.0), ncol=2, fontsize=8, frameon=False)

    out = args.out or args.log.with_name(args.log.stem + "_attitude.png")
    fig.align_ylabels([ax3, ax5, ax1, ax2])
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
