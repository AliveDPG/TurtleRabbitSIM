"""Phase 1 ceiling model, 1-D along the pad->ship line (fitted to the empty-field flights of the frozen controller).

gravity along the line g(s) = 0.01225 - 1.95e-6 s  (fitted on coasts, 0.0115 at 300 m, 0.0099 at 1200 m)
+x thrust 0.1746 m/s^2, 4.322 kg per m/s, finish at s = 1255.8 m (50 m sphere on the line).
Launch: effective start delay t0 (pitch-over) and waste W (kg): the burn is v' = A - g from t0.
Optional: second burn (mid-route top-up), extra off-axis jets (a, kg/mps).
"""
import numpy as np

A = 0.1746
KG = 4.322
L = 1255.8
DT = 0.02


def g(s):
    return 0.01225 - 1.95e-6 * s


def fly(vpk, t0=7.8, waste=2.4, trims=0.3, a=A, kg=KG, top=0.0):
    """Burn from t0 to peak speed vpk; optional `top`: hold the speed at vpk for `top` m/s of extra burn."""
    s = v = t = burn = 0.0
    phase = 0
    topped = 0.0
    while s < L and t < 900:
        acc = -g(s)
        if t >= t0 and phase == 0:
            if v < vpk:
                acc += a; burn += DT
            else:
                phase = 1
        if phase == 1 and topped < top and v < vpk:      # keep speed up against gravity (sustainer)
            acc += a; burn += DT; topped += a * DT
        v += acc * DT; s += v * DT; t += DT
    fuel = kg * a * burn + waste + trims
    return t, fuel, 100 * (120 / t - fuel / 75) + 50


def best(**kw):
    r = max(((fly(vp, **kw), vp) for vp in np.arange(6.0, 14.01, 0.1)), key=lambda z: z[0][2])
    return r[1], r[0]


if __name__ == "__main__":
    print("calibration vs empty-field flights (harness t_end at the 50 m crossing):")
    for vpk, real in ((7.45, "218.9 s 37.14 kg 55.30"), (9.08, "182.0 s 44.56 kg 56.53"), (9.93, "170.5 s 48.67 kg 55.49")):
        t, f, s = fly(vpk)
        print(f"  peak {vpk:5.2f}: model {t:6.1f} s {f:5.2f} kg {s:5.2f}   real {real}")
    cases = [("frozen launch t0 7.8 W 2.4", dict()),
             ("launch t0 7.0 W 2.0", dict(t0=7.0, waste=2.0)),
             ("launch t0 6.0 W 2.0", dict(t0=6.0, waste=2.0)),
             ("instant launch t0 0 W 0 trims 0", dict(t0=0.0, waste=0.0, trims=0.0)),
             ("frozen launch + sustainer 1 m/s", dict(top=1.0)),
             ("frozen launch, +x+y combo a 0.124", dict(a=0.1241, kg=6.079)),
             ]
    for label, kw in cases:
        vp, (t, f, s) = best(**kw)
        print(f"{label:38s} best peak {vp:5.2f}: {t:6.1f} s {f:5.2f} kg score {s:5.2f}")
    print("score vs peak speed, frozen launch:")
    for vp in (7.45, 8.0, 8.5, 9.0, 9.5, 10.0, 11.0, 12.0):
        t, f, s = fly(vp)
        print(f"  {vp:5.2f}: {t:6.1f} s {f:5.2f} kg {s:5.2f}")
