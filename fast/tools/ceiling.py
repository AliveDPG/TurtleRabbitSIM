"""Phase 1 ceiling model (no rocks): 3-D point mass in the Phobos frame with the controller's gravity model.

Launch = effective delay t0 (pitch-over) with waste W kg, then +x at A_X along the aim direction until the
coast (gravity-bent) reaches the 50 m sphere; fuel = KG_PER_MPS * A_X * t_burn + W + cruise trims.
Calibrated against the empty-field flights of the frozen controller (see fast/NOTES.md).
"""
import sys
import numpy as np

sys.path.insert(0, r"C:\Users\darpan\Documents\SpaceTeams\TurtleRabbitSIM\fast")
from guidance import coast_acc

P0 = np.array([-651.0228, 95.1990, -8239.6680])
TGT = np.array([-850.0, 120.0, -9530.0])
A_X = 0.1746          # +x capability, m/s^2 (actuator LP replica)
KG_PER_MPS = 4.322    # 3388.7 kg / 784 m/s
FINISH = 50.0
DT = 0.05


def aim_dir(p, v, vc):
    """Velocity of speed ~vc whose gravity-bent coast hits the target centre (guidance.aim_velocity)."""
    tau = np.linalg.norm(TGT - p) / vc
    g = coast_acc(p, (TGT - p) / tau)
    return (TGT - p - 0.5 * g * tau * tau) / tau


def fly(vc, t0=7.8, waste=2.4, trims=0.3, a=A_X, loft=0.0):
    p = P0.copy(); v = np.zeros(3); t = 0.0; tb = 0.0; burning = True
    while t < 600:
        g = coast_acc(p, v)
        acc = g.copy()
        if burning and t >= t0:
            dv = aim_dir(p, v, vc) - v
            if np.linalg.norm(dv) < a * DT:
                v = v + dv; burning = False
            else:
                acc = acc + a * dv / np.linalg.norm(dv); tb += DT
        v = v + acc * DT; p = p + v * DT; t += DT
        if np.linalg.norm(p - TGT) <= FINISH:
            break
    fuel = KG_PER_MPS * a * tb + waste + trims
    return t, fuel, 100 * (120 / t - fuel / 75) + 50, np.linalg.norm(v)


if __name__ == "__main__":
    print("calibration (frozen launch t0 7.8 s, waste 2.4 kg):")
    for vc in (7.0, 8.5, 9.4):
        t, f, s, va = fly(vc)
        print(f"  vc {vc:4.1f}: t {t:6.1f} s fuel {f:5.1f} kg score {s:5.2f} arrive {va:.2f}")
    for label, kw in (("frozen launch", dict(t0=7.8, waste=2.4)),
                      ("better pitch-over (t0 5, waste 2.0)", dict(t0=5.0, waste=2.0)),
                      ("ideal instant launch (t0 2, waste 0.5)", dict(t0=2.0, waste=0.5)),
                      ("zero-cost instant launch", dict(t0=0.0, waste=0.0, trims=0.0))):
        best = max(((fly(vc, **kw), vc) for vc in np.arange(6.0, 14.01, 0.25)), key=lambda z: z[0][2])
        (t, f, s, va), vc = best
        print(f"{label:40s}: best vc {vc:5.2f} t {t:6.1f} fuel {f:5.1f} score {s:5.2f}")
        for vc in (7.0, 8.0, 9.0, 10.0, 11.0, 12.0):
            t, f, s, va = fly(vc, **kw)
            print(f"      vc {vc:4.1f}: t {t:6.1f} fuel {f:5.1f} score {s:5.2f}")
