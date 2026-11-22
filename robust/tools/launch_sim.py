"""Milestone 5 item C: offline launch replica (pad -> 45 s) of robust/'s ThrustLaw + Pulser on the actuator LP replica
(thrusters.JetModel.realised), rigid body without gyroscopic terms, point-mass gravity, the pad holds the shuttle until
thrust lifts it, controller ticks bootstrapped from recorded intervals, pre-launch command held until the first tick
(cut at 0.15 rad/s as in Controller.prelaunch_check).  Prints the launch waste fuel - 4.32 x speed at 15/20/30/45 s,
the pitch-over overshoot and the time to 5 deg, mean over draws.  Env switches (RB_BRAKE, RB_BRAKE_*) apply as in flight.
usage: RB_SRC=robust/dev [RB_BRAKE=1] <sim py> robust/tools/launch_sim.py [draws]"""
import os, sys, math
import numpy as np
sys.path.insert(0, os.environ.get("RB_SRC", "robust/dev"))
import thrusters, thrust_law

MASS = 3388.7
INERTIA = np.array([[3025.8, -171.7, 461.7], [-171.7, 5927.5, 313.6], [461.7, 313.6, 6756.0]])
P_START = np.array([-651.03, 95.199, -8239.668])
P_GOAL = np.array([-850.0, 120.0, -9530.0])
R0 = np.array([[0.9400129853749776, 0.3325956784146644, -0.07586634317212979],
               [0.33142261727637834, -0.9430688616073919, -0.027931541744288274],
               [-0.08083709596526525, 0.0011121899238790804, -0.9967267062487468]])
MU = 707.5e-6 * 1e9
DELAY = int(os.environ.get("LS_DELAY", "0"))   # controller ticks between a command and its effect (actuator latency)
TICKS = np.array([0.0165, 0.0172, 0.0177, 0.018, 0.0182, 0.0183, 0.0184, 0.0186, 0.019, 0.02, 0.022, 0.026, 0.03, 0.035,
                  0.04, 0.045, 0.05, 0.034, 0.018, 0.0183])


def rot_step(R_w2b, w, dt):
    th = float(np.linalg.norm(w)) * dt
    if th < 1e-12:
        return R_w2b
    k = w / np.linalg.norm(w)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    Rk = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    return Rk.T @ R_w2b


def fly(seed, t_end=45.0):
    rng = np.random.default_rng(seed)
    pulser = thrusters.Pulser(MASS, INERTIA)
    law = thrust_law.ThrustLaw(pulser)
    jets = pulser.jets
    R = R0.copy(); w = np.zeros(3); p = P_START.copy(); v = np.zeros(3)
    up = P_START / np.linalg.norm(P_START)
    t, fuel = 0.0, 0.0
    t_first = float(os.environ["LS_T_FIRST"]) if "LS_T_FIRST" in os.environ else rng.uniform(2.5, 3.1)
    nocut = os.environ.get("LS_NOCUT", "0") == "1"      # slow start: the pre-launch rotation is not cut at 0.15 rad/s
    sc = min(0.45, 0.045 / 0.5)
    pre_cmd = (np.array([0.0, 0.0, sc]), np.array([0.0, -sc * 0.5, 0.0])); cut = False
    goal_u = (P_GOAL - P_START) / np.linalg.norm(P_GOAL - P_START)
    v_tgt = goal_u * 8.13
    rec = {}; att_min_after = None; t5 = None; max_over = 0.0; passed = False
    prev_att = None
    queue = []
    while t < t_end:
        dt = float(rng.choice(TICKS))
        if t < t_first:
            if not cut and not nocut and -w[1] >= 0.15:
                pre_cmd = (np.array([0.0, 0.0, 0.45]), np.zeros(3)); cut = True
            cmd = pre_cmd
            att = None
        else:
            a_b, al_b, att = law.step(R, w, v, v_tgt, dt=dt)
            cmd = pulser.step(a_b, al_b)
        queue.append(cmd)
        cmd = queue.pop(0) if len(queue) > DELAY else (np.zeros(3), np.zeros(3))
        acc_b, alp_b, f = jets.realised(np.concatenate(cmd), dt=thrusters.DT_TICK)
        # the actuator holds the command for the controller tick: scale per-tick values to dt
        fuel += f * dt / thrusters.DT_TICK
        g = -MU * p / np.linalg.norm(p) ** 3
        a_w = R.T @ acc_b + g
        on_pad = float(np.linalg.norm(p - P_START)) < 0.05 and float(np.dot(a_w, up)) <= 0.0 and float(np.linalg.norm(v)) < 1e-3
        if not on_pad:
            v = v + a_w * dt
            p = p + v * dt
        w = w + alp_b * dt
        R = rot_step(R, w, dt)
        t += dt
        if att is not None:
            # overshoot: once the nose first comes within 5 deg of the burn, the largest error after that while slewing
            if t5 is None and att < math.radians(5.0):
                t5 = t
            if t5 is not None and t < t5 + 15.0:
                max_over = max(max_over, math.degrees(att))
        for tt in (15.0, 20.0, 30.0, 45.0):
            if tt not in rec and t >= tt:
                rec[tt] = (fuel, float(np.linalg.norm(v)))
    return [rec[tt][0] - 4.32 * rec[tt][1] for tt in (15.0, 20.0, 30.0, 45.0)] + [rec[30.0][1], t5 or np.nan, max_over, rec[30.0][0]]


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    X = np.array([fly(s) for s in range(n)])
    m = X.mean(0)
    print(f"t_first={os.environ.get('LS_T_FIRST', '2.5-3.1')} nocut={os.environ.get('LS_NOCUT', '0')} delay={DELAY} RB_BRAKE={os.environ.get('RB_BRAKE', '0')} keep={os.environ.get('RB_BRAKE_KEEP', '0')} band={thrust_law.SLEW_BRAKE_BAND} n={n}: waste@15/20/30/45 {m[0]:.2f} {m[1]:.2f} {m[2]:.2f} {m[3]:.2f} kg | "
          f"v@30 {m[4]:.2f} m/s | nose within 5 deg at {np.nanmean(X[:, 5]):.1f} s | overshoot after that {m[6]:.1f} deg (max {X[:, 6].max():.1f}) | fuel@30 {m[7]:.2f}")
