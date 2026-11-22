"""Planar pitch-over study (no rocks): how early can the +x burn effectively start?

Body pitch angle phi from the pad pose (goal along body +z at phi = 0, along +x at phi = 90 deg).
Per tick (1/60 s) one action, capabilities from the actuator LP replica (fast/tools, thrusters.JetModel):
  Z    +z 0.0958 m/s^2                       0.414 kg/s
  X    +x 0.1746                              0.7545
  RU   pure pitch-up  0.0903 rad/s^2          0.4533
  RD   pure pitch-down 0.0764                 0.4431
  CZU  combo +z 0.1189 with pitch-up 0.0595   0.534
Gravity 0.0125 along the goal line.  Result per policy: speed along the goal line at 20 s, cross-line speed
(must be cancelled later: 4.32 kg per m/s and time), fuel; effective start t0 = 20 - v20 / (0.1746 - 0.0125).
"""
import math
import itertools

DT = 1 / 60
G = 0.0125
ACT = {"Z": (0.0, 0.0958, 0.0, 0.414), "X": (0.1746, 0.0, 0.0, 0.7545), "RU": (0, 0, 0.0903, 0.4533),
       "RD": (0, 0, -0.0764, 0.4431), "CZU": (0.0, 0.1189, 0.0595, 0.534), "N": (0, 0, 0, 0)}


def run(phi_sw, x_from_deg, t_start=0.0, rd_share=1.0, until=20.0):
    """Spin up with CZU until phi >= phi_sw (deg), then brake with RD (interleaved with X once the nose is within
    x_from_deg of the goal, X share 1 - rd_share) until the rate is zero at ~90 deg, then X."""
    phi = w = 0.0; vu = vp = 0.0; fuel = 0.0; t = 0.0; k = 0
    phase = 0
    while t < until:
        if t < t_start:
            a = "N"
        elif phase == 0:
            a = "CZU"
            if math.degrees(phi) >= phi_sw:
                phase = 1
        elif phase == 1:
            # brake so that w reaches 0 at 90 deg; fire X when the nose is close enough and braking has slack
            rem = math.radians(90) - phi
            need = w * w / (2 * rem) if rem > 1e-4 else 9.0
            nose_err = 90 - math.degrees(phi)
            if w <= 0.002 or rem <= 0:
                phase = 2; a = "X"
            elif need >= 0.0764 * rd_share * 0.98 or nose_err > x_from_deg:
                a = "RD"
            else:
                a = "X"
        else:
            a = "X"
        ax, az, al, fr = ACT[a]
        # body accel in the goal frame: +z body at angle phi from the goal, +x at 90 - phi
        au = az * math.cos(phi) + ax * math.sin(phi) - G
        ap = -az * math.sin(phi) + ax * math.cos(phi)
        vu += au * DT; vp += ap * DT; w += al * DT; phi += w * DT; fuel += fr * DT; t += DT; k += 1
    t0 = until - vu / (0.1746 - G)
    return vu, vp, fuel, t0, math.degrees(phi), w


if __name__ == "__main__":
    print("frozen controller in flight: speed 1.96 m/s and 11.51 kg at 20 s (t0 ~7.8 s, waste ~2.4 kg)")
    rows = []
    for phi_sw, xf, rds in itertools.product((30, 40, 45, 50, 55, 60), (5, 10, 20, 30), (0.6, 0.8, 1.0)):
        vu, vp, f, t0, ph, w = run(phi_sw, xf, rd_share=rds)
        waste = f - 4.322 * vu + 4.322 * abs(vp)      # fuel beyond the line speed, plus cancelling the cross speed
        rows.append((t0 + 0.0, waste, phi_sw, xf, rds, vu, vp, f, ph))
    rows.sort()
    print(" t0    waste  phi_sw x_from rd  v20   vperp  fuel  phi_end")
    for r in rows[:8]:
        print(" %.2f  %.2f   %3d    %3d  %.1f  %.3f %+.3f %.2f %.1f" % r)
    best = min(rows, key=lambda r: 0.30 * r[0] + 1.333 * r[1])   # 0.30 pts/s, 1.333 pts/kg
    print("best by points (0.30 pts/s, 1.33 pts/kg):", " ".join("%.3g" % x for x in best))
    print("first tick at 2.2 s (no pre-launch command):", " ".join("%.3g" % x for x in run(best[2], best[3], 2.2, best[4])))
