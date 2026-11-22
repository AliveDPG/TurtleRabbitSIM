"""Thrust and attitude law: turns (velocity, target velocity, attitude, body rate) into the
(acceleration, angular acceleration) request for the Pulser.

Fuel is the L1 norm of the body-frame velocity change, and every rad/s of body rate costs 4-7 kg
to put on and take off again.  So the law keeps burns on single body axes and turns the hull as
little and as slowly as it can afford:
  * pad and main burn: nose on the velocity error, tight deadband, so the burn stays on the +x jets;
  * cruise and dodges: nose loosely on the flight direction (the cameras look along it), re-pointed slowly;
  * while a velocity error is being flown, the cheapest way of putting it on ONE body axis is priced
    and latched: fly it as it is, roll about the nose, or a full re-orientation (see choose);
  * a body axis is only pulsed when its rate error exceeds a fraction of one pulse's rate change, so
    there is no chatter and no undamped residual drift.
"""
import math, os

import numpy as np

from guidance import unit

A_REQUEST = 1.0                         # m/s^2 asked of the pulser: more than any jet gives (0.10-0.18), i.e. full duty
VEL_DEADBAND = 0.06                     # m/s: velocity errors below this are not chased (one pulse is 0.003 m/s)
BURN_MODE_MPS = 1.0                     # a velocity error above this, within BURN_CONE of the target velocity, is a main burn
BURN_CONE = math.radians(25.0)
PAD_ALIGN = math.radians(20.0)          # on the pad, thrust waits until the nose is within this of the burn ...
PAD_L1_MAX = 0.5                        # ... unless the burn lies near some body axis (L1/L2 - 1 below this): lift off at once
LAUNCH_EXIT_MPS = 1.0                   # the launch phase ends above this speed, once the pitch-over is done
LAUNCH_TRANS_SHARE = 0.5                # share of the ticks translation keeps during the launch
SLEW_ON = math.radians(15.0)            # pad pointing error that starts the fast pitch-over
ALPHA_SLEW, W_SLEW_MAX = 0.05, 0.2      # rad/s^2, rad/s of the pitch-over (the jets give 0.08-0.125 rad/s^2)
ALPHA_HOLD = 0.025                      # rad/s^2 of every in-flight re-point: fuel per correction ~ sqrt(alpha)
W_HOLD_MAX = 0.01                       # rad/s cap of an in-flight re-point (a fast one costs 2 x rate x 5.5 kg per rad/s)
DB_BURN, DB_BURN_OFF = math.radians(1.5), math.radians(0.5)       # pointing deadband on / off, pad and burn
DB_CRUISE, DB_CRUISE_OFF = math.radians(10.0), math.radians(4.0)  # ... and in cruise, where pointing costs no fuel
DAMP_FRAC = 0.6                         # an axis is pulsed when its rate error exceeds this share of one pulse
BRAKE_MARGIN = 0.7                      # share of the angular acceleration the braking curves count on
TUMBLE_ON, TUMBLE_OFF = 0.40, 0.10      # rad/s: body rate that starts / ends tumble recovery
ALPHA_TUMBLE = 1.0                      # rad/s^2 asked for while recovering: everything the jets give
SLEW_BRAKE = os.environ.get("RB_BRAKE", "0") == "1"   # robust: full-strength braking of the pitch-over (mode 4)
RECOVER_SLEW = os.environ.get("RB_RECOVER", "0") == "1"   # robust: fast re-point after a tumble (see step)
RECOVER_DVBURN = os.environ.get("RB_RECOVER_DVBURN", "0") == "1"   # after a tumble: main-jet burn along the velocity error
RECOVER_BURN = math.radians(float(os.environ.get("RB_RECOVER_BURN_DEG", "0")))   # ... translation held while the nose is this far off (0 = off)
RECOVER_ATT = math.radians(float(os.environ.get("RB_RECOVER_DEG", "0")))   # ... and whenever the nose is this far off in flight (0 = off)
SLEW_BRAKE_BAND = 0.01                  # rad/s above the braking curve that triggers full-strength braking
# robust (milestone 5): the full-strength brake cost 0.55-0.75 kg in every launch (speed at 15 s 1.05 vs 1.23 m/s at
# equal fuel: xp_brake vs xp_ctrl, conf, m4c, m5a); with RB_BRAKE_KEEP=1 the translation keeps its launch share of
# the ticks while braking once the nose is within PAD_ALIGN of the burn, and the brake gain is SLEW_BRAKE_K_KEEP.
SLEW_BRAKE_KEEP = os.environ.get("RB_BRAKE_KEEP", "0") == "1"
SLEW_BRAKE_K_KEEP = float(os.environ.get("RB_BRAKE_K", "5.0"))

ALIGN_DV_MIN = 0.4                      # m/s: a smaller velocity error is flown as it is
ALIGN_HORIZON_S = 15.0                  # s of burn expected before the plan changes again: only that part profits
ALIGN_SWITCH_KG = 0.3                   # another alignment must be this much cheaper to replace the latched one
ALIGN_TIME_KG = 0.03                    # kg per second of burn time (tie-break towards the strong axes)
ALIGN_RETURN_KG = 0.15                  # the slow re-point of the nose after a full alignment
ALIGN_DB = math.radians(2.0)            # an alignment rotation stops inside this error ...
ALIGN_CORR = math.radians(12.0)         # ... and starts again beyond this one
ALIGN_GATE = math.radians(15.0)         # the aligned channel fires only within this angle of the velocity error
ALIGN_TAU_FRAC, ALIGN_TAU_MIN = 0.3, 10.0     # a rotation takes this share of the burn time, at least this many s
W_ROLL, ALPHA_ROLL = 0.10, 0.08         # rad/s, rad/s^2 of a roll (4.0 kg per rad/s, the cheapest rotation)
W_FULL, ALPHA_FULL = 0.05, 0.05         # ... and of a full alignment
ALIGN_FULL_MAX = math.radians(60.0)     # no full alignment that needs more rotation than this
ALIGN_NOSE_MAX = math.radians(45.0)     # ... or leaves the flight direction further than this off the nose (camera pan range)
ALIGN_PITCH_MAX = math.radians(35.0)    # ... (this, when the offset is in camera pitch)
ALIGN_BURN_FRAC = 0.75                  # below this share of the target speed (the main burn): roll only, no gating


def _ang(a, b):
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return math.acos(float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))


class ThrustLaw:
    # side axes for a roll: (name, angle of the axis in the body y-z plane, translation channel)
    SIDE = (("+y", 0.0, 2), ("+z", 0.5 * math.pi, 4), ("-y", math.pi, 3), ("-z", -0.5 * math.pi, 5))

    def __init__(self, pulser):
        self.pulser = pulser
        ct, cr = pulser.cap_t, pulser.cap_r
        self.acc_ch = np.array([ct[c][2] for c in range(6)])      # +x -x +y -y +z -z, m/s^2
        # per body axis: angular acceleration of one rotation pulse (positive / negative channel)
        self.cap_rot = np.array([[cr[2 * i][2], cr[2 * i + 1][2]] for i in range(3)])
        self.cap_roll = float(min(cr[0][2], cr[1][2]))
        self.cap_rot_min = float(min(c[2] for c in cr))
        self.kg_per_mps = float(ct[0][3] / max(ct[0][2], 1e-9))                                   # 4.32
        self.kg_rot = np.array([0.5 * (cr[2 * i][3] / max(cr[2 * i][2], 1e-9) + cr[2 * i + 1][3] / max(cr[2 * i + 1][2], 1e-9))
                                for i in range(3)])                                               # 4.0 5.4 7.3 kg per rad/s
        self.dt_ctrl = 0.024            # controller tick period (running mean); one pulse lasts this long
        self.mode = "pad"
        self.launching = True
        self.slewing = False
        self.active = True              # pointing loop engaged (deadband hysteresis)
        self.tumbling = False
        self.recovering = False         # fast re-point after a tumble (RB_RECOVER)
        self.hit_burn = False           # after a tumble: burn the velocity error along the nose (RB_RECOVER_DVBURN)
        self.sel = None                 # latched alignment: None, ("roll", side index) or ("full", body channel)
        self.align_moving = False
        self.align_wlatch = 0.0

    def quantum(self, sign_vec):
        """Rate change of one pulse per body axis, for the channel that would fire (sign of the request)."""
        q = np.where(np.asarray(sign_vec) >= 0, self.cap_rot[:, 0], self.cap_rot[:, 1])
        return q * self.dt_ctrl

    def rate_loop(self, w_des, omega_b, alpha_max, k_rate=1.2):
        """Saturated proportional rate loop; an axis whose error is below DAMP_FRAC of a pulse asks for nothing
        (a pulse would overshoot it)."""
        err = np.asarray(w_des, float) - np.asarray(omega_b, float)
        q = self.quantum(err)
        err = np.where(np.abs(err) > DAMP_FRAC * q, err, 0.0)
        alpha = k_rate * err
        n = float(np.linalg.norm(alpha))
        if n > alpha_max:
            alpha *= alpha_max / n
        return alpha

    def pointing_rate(self, R_w2b, x_des_world, alpha_max, w_max):
        """Body rate that slews body +x onto x_des (roll left free): bang-bang braking curve capped at w_max."""
        x_b = R_w2b @ unit(x_des_world)
        axis = np.cross(np.array([1.0, 0.0, 0.0]), x_b)
        s = float(np.linalg.norm(axis))
        c = float(np.clip(x_b[0], -1.0, 1.0))
        ang = math.atan2(s, c)
        if s < 1e-9:
            axis_u = np.array([0.0, 0.0, 1.0]) if c < 0 else np.zeros(3)
        else:
            axis_u = axis / s
        w = axis_u * min(w_max, math.sqrt(2.0 * BRAKE_MARGIN * alpha_max * ang))
        w[0] = 0.0
        return w

    # ------------------------------------------------------------------ axis alignment
    def _burn_time(self, d_b):
        """Seconds to fly body-frame velocity change d_b one axis per tick."""
        t = 0.0
        for i in range(3):
            c = 2 * i + (0 if d_b[i] >= 0 else 1)
            t += abs(float(d_b[i])) / max(self.acc_ch[c], 1e-6)
        return t

    def _rot_profile(self, theta, n_b, w0_along, w_cap, alpha):
        """(fuel kg, time s) to rotate by theta about body unit axis n_b, starting at rate w0_along about it."""
        if theta < 1e-6:
            return 0.0, 0.0
        wp = min(w_cap, math.sqrt(2.0 * BRAKE_MARGIN * alpha * theta))
        k = float(np.sum(np.abs(n_b) * self.kg_rot))            # one axis per tick: L1 over the body axes
        fuel = k * (abs(wp - w0_along) + wp)
        return fuel, theta / wp + wp / alpha

    def _w_cap(self, ang, t_dv, w_max):
        """Peak rate of a rotation by ang ahead of a burn of t_dv s: slow is cheap, so it may take a share of the burn."""
        tau = max(ALIGN_TAU_MIN, ALIGN_TAU_FRAC * t_dv)
        return float(np.clip(ang / tau, 0.005, w_max))

    @staticmethod
    def _axis_angle(C):
        """Rotation axis (body frame) and angle of the rotation matrix C (columns = desired axes in current body axes)."""
        c = float(np.clip((np.trace(C) - 1.0) * 0.5, -1.0, 1.0))
        th = math.acos(c)
        v = np.array([C[2, 1] - C[1, 2], C[0, 2] - C[2, 0], C[1, 0] - C[0, 1]])
        s = float(np.linalg.norm(v))
        if s < 1e-9:
            return (np.zeros(3), 0.0) if th < 1.0 else (None, th)
        return v / s, th

    def _full_targets(self, R_w2b, u, vt_u):
        """Attitudes (rows = body axes in world) that put one body axis on the unit velocity error u with the nose
        near the flight direction vt_u: (channel, attitude, nose offset, plane of that offset in the camera)."""
        out = []
        nose_proj = vt_u - float(vt_u @ u) * u
        npn = float(np.linalg.norm(nose_proj))
        # +x on u: the flight direction goes into the body x-y plane (camera pan), roll sign closest to the current
        # z; the roll is left as it is when u is within 15 deg of the flight direction
        z = np.cross(u, vt_u)
        if float(np.linalg.norm(z)) < math.sin(math.radians(15.0)):
            z = R_w2b[2, :] - float(R_w2b[2, :] @ u) * u
        z = unit(z)
        if float(z @ R_w2b[2, :]) < 0:
            z = -z
        x = u
        out.append((0, np.array([x, np.cross(z, x), z]), math.acos(float(np.clip(x @ vt_u, -1, 1))), "yaw"))
        if npn > 1e-6:
            x = nose_proj / npn
            off = math.acos(float(np.clip(x @ vt_u, -1, 1)))
            for ch, sgn, ax in ((2, 1.0, 1), (3, -1.0, 1), (4, 1.0, 2), (5, -1.0, 2)):
                a = sgn * u                    # body axis that must lie along u for channel ch to push along u
                if ax == 1:
                    out.append((ch, np.array([x, a, np.cross(x, a)]), off, "yaw"))
                else:
                    out.append((ch, np.array([x, np.cross(a, x), a]), off, "pitch"))
        return out

    def choose(self, R_w2b, omega_b, dv, v_tgt, urgent):
        """Price every way of flying this tick's velocity error dv and latch the cheapest:
          none   : as it is, on every body axis it touches: kg/(m/s) x (L1 - |dv|) of excess;
          roll s : roll about the nose until the part of dv across the nose lies on side axis s.  The cameras are
                   pointed relative to the flight direction in the body frame, so a roll does not move the views;
          full k : rotate until body axis k lies on dv with the nose as near the flight direction as that allows.
        Each costs its rotation fuel (2 x peak rate x kg per rad/s) + the excess flown while rotating + the excess
        left + a small burn-time term, for the part of dv flown within ALIGN_HORIZON_S.
        Returns (selection, its target)."""
        KG = self.kg_per_mps
        d_b = R_w2b @ dv
        nd = float(np.linalg.norm(d_b))
        dx, dy, dz = float(d_b[0]), float(d_b[1]), float(d_b[2])
        L = math.hypot(dy, dz)
        excess = abs(dx) + abs(dy) + abs(dz) - nd
        tk = ALIGN_TIME_KG
        a_tr = 0.5 * float(np.mean(self.acc_ch[2:]))            # translation flown while rotating (shared ticks)
        D = min(nd, 2.0 * a_tr * ALIGN_HORIZON_S)
        h = D / max(nd, 1e-9)
        costs = {None: h * (KG * excess + tk * self._burn_time(d_b))}
        targets = {}
        if L >= 0.25 * ALIGN_DV_MIN:
            psi = math.atan2(dz, dy)
            a_roll = min(ALPHA_ROLL, self.cap_roll)
            for i, (_, ang, ch) in enumerate(self.SIDE):
                phi = (psi - ang + math.pi) % (2.0 * math.pi) - math.pi
                wcap = self._w_cap(abs(phi), L / self.acc_ch[ch], W_ROLL)
                f_rot, t_rot = self._rot_profile(abs(phi), np.array([1.0, 0, 0]), math.copysign(1, phi) * float(omega_b[0]),
                                                 wcap, a_roll)
                flown = min(D, a_tr * t_rot)
                left = abs(dx) + L - nd
                costs[("roll", i)] = (f_rot + KG * flown * 0.5 * max(excess - left, 0.0) / max(nd, 1e-9) + h * KG * left
                                      + h * tk * (abs(dx) / self.acc_ch[0 if dx >= 0 else 1] + L / self.acc_ch[ch]))
                targets[("roll", i)] = (phi, wcap)
        if not urgent and v_tgt is not None and float(np.linalg.norm(v_tgt)) > 1.0:
            u = unit(dv)
            vt_u = unit(v_tgt)
            for ch, Rd, off, plane in self._full_targets(R_w2b, u, vt_u):
                if off > ALIGN_NOSE_MAX or (plane == "pitch" and off > ALIGN_PITCH_MAX):
                    continue
                n_b, th = self._axis_angle(R_w2b @ Rd.T)
                if n_b is None or th > ALIGN_FULL_MAX:
                    continue
                wcap = self._w_cap(th, nd / self.acc_ch[ch], W_FULL)
                f_rot, t_rot = self._rot_profile(th, n_b, float(np.asarray(omega_b) @ n_b), wcap,
                                                 min(ALPHA_FULL, self.cap_rot_min))
                flown = min(D, a_tr * t_rot)
                ret = ALIGN_RETURN_KG if off > DB_CRUISE else 0.0
                costs[("full", ch)] = f_rot + KG * flown * 0.5 * excess / max(nd, 1e-9) + ret + h * tk * nd / self.acc_ch[ch]
                targets[("full", ch)] = (n_b, th, wcap)
        cur = self.sel if self.sel in costs else None
        best = min(costs, key=lambda k: costs[k])
        if best != cur and costs[best] < costs[cur] - ALIGN_SWITCH_KG:
            self.sel = best
            self.align_moving = True
            self.align_wlatch = 0.0
        else:
            self.sel = cur
        return self.sel, targets.get(self.sel)

    def _align(self, R_w2b, omega_b, dv, v_tgt, ndv_eff, on_pad, w_des, alpha_max, alpha_plain, a_body, urgent):
        """(angular acceleration, translation) with the latched alignment applied; the plain request when none is."""
        omega_b = np.asarray(omega_b, float)
        if on_pad or self.slewing or self.launching or ndv_eff < ALIGN_DV_MIN:
            self.sel = None
            return alpha_plain, a_body
        sel, tgt = self.choose(R_w2b, omega_b, np.asarray(dv, float), v_tgt, urgent)
        if sel is None:
            return alpha_plain, a_body
        if sel[0] == "roll":
            err, wcap = abs(float(tgt[0])), float(tgt[1])
        else:
            err, wcap = float(tgt[1]), float(tgt[2])
        if err > ALIGN_CORR and not self.align_moving:
            self.align_moving = True
            self.align_wlatch = 0.0
        elif err < ALIGN_DB:
            self.align_moving = False
        if self.align_moving:
            # the rate chosen at the start of the rotation is kept (trapezoid profile): angle / tau would decay
            # exponentially and, with the gated channel firing meanwhile, stall 10-15 deg short
            self.align_wlatch = max(self.align_wlatch, wcap)
            wcap = self.align_wlatch
        if sel[0] == "roll":
            phi = float(tgt[0])
            a = min(ALPHA_ROLL, self.cap_roll)
            wx = 0.0 if not self.align_moving else math.copysign(
                min(wcap, math.sqrt(2.0 * BRAKE_MARGIN * a * abs(phi))), phi)
            # pitch / yaw keep pointing the nose; roll has its own rate loop
            al = self.rate_loop(np.array([0.0, w_des[1], w_des[2]]), np.array([0.0, omega_b[1], omega_b[2]]), alpha_max)
            ax = self.rate_loop(np.array([wx, 0.0, 0.0]), np.array([omega_b[0], 0.0, 0.0]), a, k_rate=2.0)
            al[0] = ax[0]
        else:
            n_b, th = tgt[0], tgt[1]
            a = min(ALPHA_FULL, self.cap_rot_min)
            w = np.zeros(3) if not self.align_moving else n_b * min(wcap, math.sqrt(2.0 * BRAKE_MARGIN * a * th))
            al = self.rate_loop(w, omega_b, a)
        if not np.all(np.isfinite(al)):
            self.sel = None
            return alpha_plain, a_body
        if not urgent:
            # Fire only the channel(s) that carry dv once aligned (roll: x and the chosen side axis; full: the chosen
            # axis), with the right sign and only within ALIGN_GATE of dv: a pulse at angle e off dv makes cos(e) of
            # progress and sin(e) of cross-track that costs fuel again later.  The rest of dv waits for the rotation.
            a_body = np.asarray(a_body, float).copy()
            ch = self.SIDE[sel[1]][2] if sel[0] == "roll" else sel[1]
            ax, sg = ch // 2, (1.0 if ch % 2 == 0 else -1.0)
            keep = np.zeros(3, bool)
            keep[ax] = sg * a_body[ax] > 0.0 and err < ALIGN_GATE
            if sel[0] == "roll":
                keep[0] = True
            a_body[~keep] = 0.0
        return al, a_body

    # ------------------------------------------------------------------ main
    def step(self, R_w2b, omega_b, v, v_tgt, dt=None, hold=False, urgent=False):
        """Returns (a_body, alpha_body, pointing error in rad).
        hold: no translation (the planner has paused the burn).
        urgent: a dodge against a close threat is being flown: no full alignment and no gating, the dodge fires at
        once (a roll is still allowed: it puts the dodge on one strong side jet, which is faster as well as cheaper)."""
        if dt is not None and 0.004 < dt < 0.2:
            self.dt_ctrl = 0.95 * self.dt_ctrl + 0.05 * dt
        v = np.asarray(v, float)
        v_tgt = np.asarray(v_tgt, float)
        dv = v_tgt - v
        ndv = float(np.linalg.norm(dv))
        ndv_eff = 0.0 if hold else ndv
        speed = float(np.linalg.norm(v))
        on_pad = speed < 0.5 and ndv > 1.0
        burn = ndv > BURN_MODE_MPS and _ang(dv, v_tgt) < BURN_CONE
        # After a hit the velocity error can point anywhere (a 16 m/s reversal was flown in cruise mode with the
        # nose on v_tgt and the error 25+ deg off it: 6.7 kg per m/s).  Until the error is flown, point the nose
        # along the error and burn on the main jets, whatever its angle to v_tgt.
        if self.hit_burn and ndv < 1.0:
            self.hit_burn = False
        if self.hit_burn and ndv > BURN_MODE_MPS:
            burn = True
        self.mode = "pad" if on_pad else "burn" if burn else "cruise"
        x_des = dv if (on_pad or burn) else v_tgt
        if float(np.linalg.norm(x_des)) < 1e-6:
            x_des = R_w2b[0, :]          # nothing to point at: hold the current nose
        nose = R_w2b[0, :]
        burn_err = _ang(nose, dv) if ndv > 1e-6 else 0.0
        pad_gate = on_pad and burn_err > PAD_ALIGN
        if pad_gate and ndv > 1e-6:
            dvb = np.abs(R_w2b @ dv)
            if float(dvb.sum()) / max(float(np.linalg.norm(dvb)), 1e-9) - 1.0 <= PAD_L1_MAX:
                pad_gate = False             # dv is near some body axis: that jet pushes us off the pad usefully
        if ndv_eff < VEL_DEADBAND or pad_gate:
            a_world = np.zeros(3)
        else:
            a_world = dv / ndv * min(A_REQUEST, 2.0 * ndv)
        a_body = R_w2b @ a_world

        att_err = _ang(nose, x_des)
        if att_err > SLEW_ON and on_pad:
            # the fast slew is for the pitch-over only: in flight a dodge swings the pointing target by 10-40 deg
            # and each fast re-point would cost 2 x rate x 5.5 kg per rad/s
            self.slewing = True
        elif self.slewing and att_err < math.radians(3.0) and float(np.linalg.norm(omega_b)) < 0.01:
            self.slewing = False
        if self.launching and speed > LAUNCH_EXIT_MPS and not self.slewing:
            self.launching = False
        self.pulser.trans_min_share = LAUNCH_TRANS_SHARE if self.launching else 0.0
        tight = on_pad or burn
        if att_err > (DB_BURN if tight else DB_CRUISE) or self.slewing:
            self.active = True
        elif att_err < (DB_BURN_OFF if tight else DB_CRUISE_OFF):
            self.active = False
        # After a tumble the nose can be 90-160 deg off; the in-flight re-point (0.01 rad/s) then took 100-200 s
        # and one flight never finished.  Until the nose is back, re-point at the pitch-over rates (~2 kg).
        if self.recovering and att_err < math.radians(5.0) and float(np.linalg.norm(omega_b)) < 0.02:
            self.recovering = False
        elif RECOVER_ATT > 0.0 and att_err > RECOVER_ATT and not on_pad and not self.launching and not self.tumbling:
            self.recovering = True
        fast = self.slewing or self.recovering
        if self.recovering:
            self.active = True
        alpha_max = ALPHA_SLEW if fast else ALPHA_HOLD
        if self.active:
            w_des = self.pointing_rate(R_w2b, x_des, alpha_max, W_SLEW_MAX if fast else W_HOLD_MAX)
        else:
            w_des = np.zeros(3)
        alpha_body = self.rate_loop(w_des, omega_b, alpha_max)
        # Pitch-over braking.  The braking curve counts on 0.035 rad/s^2, but the proportional loop asks for that
        # only once the rate is 0.03 rad/s above the curve, and the launch keeps half the ticks for translation:
        # the nose overshot by 9 to 40 deg in 12 of 30 flights (0.5 to 3.4 kg, 2 to 8 s).  Above the curve the
        # rotation now gets every tick at full strength until the rate is back on it.
        if SLEW_BRAKE and self.slewing and self.active:
            excess = float(np.linalg.norm(np.asarray(omega_b, float)[1:])) - float(np.linalg.norm(w_des[1:]))
            if excess > SLEW_BRAKE_BAND:
                if SLEW_BRAKE_KEEP:
                    alpha_body = self.rate_loop(w_des, omega_b, ALPHA_TUMBLE, k_rate=SLEW_BRAKE_K_KEEP)
                    if att_err > PAD_ALIGN:
                        self.pulser.trans_min_share = 0.0
                else:
                    alpha_body = self.rate_loop(w_des, omega_b, ALPHA_TUMBLE, k_rate=20.0)
                    self.pulser.trans_min_share = 0.0

        # Tumble recovery.  A rock contact leaves the hull spinning at 2-4.5 rad/s.  The in-flight rate loop is
        # capped at ALPHA_HOLD (160 s for 4.5 rad/s) and a burn would go on at full duty on whatever axis sweeps
        # past dv (60-80 kg for 3 m/s).  No commanded rate exceeds 0.2 rad/s, so above TUMBLE_ON the translation
        # stops and the rate is braked with everything the jets give, until TUMBLE_OFF.
        w_now = float(np.linalg.norm(omega_b))
        if np.isfinite(w_now):
            if w_now > TUMBLE_ON and not self.tumbling:
                self.tumbling = True
            elif w_now < TUMBLE_OFF:
                if self.tumbling and RECOVER_SLEW:
                    self.recovering = True
                if self.tumbling and RECOVER_DVBURN:
                    self.hit_burn = True
                self.tumbling = False
        if self.tumbling:
            self.mode = "tumble"
            self.sel = None
            self.align_moving = False
            return np.zeros(3), self.rate_loop(np.zeros(3), omega_b, ALPHA_TUMBLE), att_err

        # In the main burn a gated alignment of a dodge holds the burn back (1.52 instead of 1.87 m/s 5 s after a
        # 3.1 m/s dodge at t = 20 s), and a slow shuttle in the cloud is the larger risk: treat it as urgent.
        if speed < ALIGN_BURN_FRAC * float(np.linalg.norm(v_tgt)):
            urgent = True
        alpha_body, a_body = self._align(R_w2b, omega_b, dv, v_tgt, ndv_eff, on_pad, w_des, alpha_max, alpha_body, a_body, urgent)
        # Recovery: after a hit the target can be 90-130 deg off the nose; burning the velocity error on whatever
        # axes face it cost 8.5 kg per m/s instead of 4.3 (contact flights).  Hold the translation until the
        # fast re-point has the nose within RECOVER_BURN_DEG, then burn on the main jets.
        if self.recovering and RECOVER_BURN > 0.0 and att_err > RECOVER_BURN:
            a_body = np.zeros(3)
        return a_body, alpha_body, att_err
