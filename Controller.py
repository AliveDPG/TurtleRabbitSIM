#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3,
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


#############################################################################################
# USER CODE - Escape from Phobos controller                                                 #
#                                                                                           #
#   nav.py     : ShuttleNav (state fusion), DebrisTracker (world-frame CV Kalman tracks),   #
#                Guidance (tube filter, path/velocity decomposition, burn-coast-trim)       #
#   vision.py  : threshold + blob finder, pixel -> bearing geometry                         #
#   this file  : sim I/O, attitude control, camera scheduling, status logging               #
#############################################################################################

import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav import ShuttleNav, DebrisTracker, Guidance, GuidanceParams, unit
import vision

# ----------------------------------------------------------------------------- tuning knobs
GP = GuidanceParams(
    a_max=0.5,            # sim caps |accel| at 0.5 m/s^2
    v_cruise=15.0,        # single main burn up to this speed (~30 s), then coast
    v_arrive=15.0,        # == v_cruise -> no braking; the run ends at the 50 m keep-out
    keep_out_m=50.0,
    tube_radius=80.0,
    horizon_s=20.0,       # debris accelerates ~0.1 m/s^2: forecast error ~10 m at 20 s, useless beyond ~30 s;
)
RADAR_N = 500             # ask for everything the radar has (it publishes 200)
RADAR_EVERY = 6           # loop ticks between radar reads (60 Hz loop -> 10 Hz)
CAMERA_PERIOD_S = 1.5     # seconds between captures (capture blocks ~0.1-0.3 s)
CAMERA_ONLY_WHEN_COASTING = True
CAMERA_RES = 384
CAMERA_FOV = 60.0
CAMERA_EV = 10.0
BLOB_THRESHOLD = 40.0     # 0-255 grey; None = auto
LOG_EVERY_S = 1.0
# attitude: body +x along the flight direction (cameras look forward), +z "up"
ATT_HOLD = False          # False: never fire the rotation jets (saves fuel; guidance works in the
                          # Phobos frame and is converted to body axes every tick, so orientation
                          # does not matter for flying - only the cameras care)
ATT_KP = 0.02
ATT_KD = 0.30
ANG_ACC_MAX = 0.05        # rad/s^2, sim cap


# ----------------------------------------------------------------------------- helpers
def quat_to_dcm(q):
    """Scalar-last quaternion -> rotation matrix (convention checked at start-up)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def body_axes_via_frames():
    """Columns = shuttle body axes expressed in the Phobos frame (ground truth from the API)."""
    cols = [st.frames.FramedVector(np.array(e, float), shuttleFrame).ExprIn(phobosFrame)
            for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return np.column_stack(cols)


def log(msg, tag="Guidance", sev=None):
    st.OnScreenLogMessage(msg, tag, sev if sev is not None else st.Severity.Info)


class AttitudeTracker:
    """Figures out, once, how the API quaternion maps to a body->world DCM."""
    def __init__(self):
        self.mode = None          # "R", "RT" or "frames"

    def body_to_world(self, q):
        if self.mode is None:
            R_true = body_axes_via_frames()
            R = quat_to_dcm(q)
            if np.allclose(R, R_true, atol=1e-3):
                self.mode = "R"
            elif np.allclose(R.T, R_true, atol=1e-3):
                self.mode = "RT"
            else:
                self.mode = "frames"
            log(f"Attitude convention: {self.mode}", "Attitude")
        if self.mode == "R":
            return quat_to_dcm(q)
        if self.mode == "RT":
            return quat_to_dcm(q).T
        return body_axes_via_frames()


def attitude_command(R_bw, omega_body, x_des_world, up_world):
    """PD on the rotation error that puts body +x on x_des and body +z near 'up'."""
    x_hat = unit(x_des_world)
    z_hat = unit(up_world - np.dot(up_world, x_hat) * x_hat)
    if np.linalg.norm(z_hat) < 1e-6:
        z_hat = unit(np.cross(x_hat, np.array([0.0, 1.0, 0.0])))
    y_hat = np.cross(z_hat, x_hat)
    R_des = np.column_stack((x_hat, y_hat, z_hat))         # body->world desired
    R_err = R_des @ R_bw.T                                   # world frame
    ang = math.acos(np.clip((np.trace(R_err) - 1.0) * 0.5, -1.0, 1.0))
    if ang < 1e-6:
        e_world = np.zeros(3)
    else:
        axis = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
        e_world = axis / (2.0 * math.sin(ang)) * ang
    e_body = R_bw.T @ e_world
    alpha = ATT_KP * e_body - ATT_KD * omega_body
    n = np.linalg.norm(alpha)
    if n > ANG_ACC_MAX:
        alpha *= ANG_ACC_MAX / n
    return alpha, ang


class CameraManager:
    """
    Captures a frame, thresholds it, finds blobs and fuses each blob bearing
    with the tracker.  Depth is used if the image object exposes it (the
    competition brief mentions depth images); otherwise radar supplies range.
    """
    CAM_POS_BODY = {"top": np.array([1.27, 0.0, 0.9]), "bottom": np.array([1.675, 0.0, -1.025])}

    def __init__(self):
        self.last_t = -1e9
        self.depth_attr = None
        self.probed = False
        self.frames = 0
        self.blobs_total = 0
        self.aim = {"top": (0.0, 0.0), "bottom": (0.0, 0.0)}
        self.ang_sigma = vision.pixel_ang_sigma(CAMERA_RES, CAMERA_FOV)

    def point(self, which, d_body):
        yaw, pitch = vision.body_dir_to_yaw_pitch(d_body)
        cam = topCamera if which == "top" else bottomCamera
        if which == "top":
            yaw, pitch = float(np.clip(yaw, -89, 89)), float(np.clip(pitch, -50, 0))
        else:
            yaw, pitch = float(np.clip(yaw, -89, 89)), float(np.clip(pitch, 0, 50))
        self.aim[which] = (yaw, pitch)
        PanTiltCamera(cam, yaw, pitch)

    def _probe_depth(self, image):
        self.probed = True
        names = [a for a in dir(image) if not a.startswith("_")]
        log(f"Image API attrs: {names}", "CameraAPI")
        for cand in ("as_Depth", "as_Depth32F", "as_DepthF32", "as_Depth16", "PixelsDepth", "Depth"):
            if hasattr(image, cand):
                self.depth_attr = cand
                log(f"Using depth accessor '{cand}'", "CameraAPI")
                break

    def _depth_array(self, image):
        if self.depth_attr is None:
            return None
        try:
            obj = getattr(image, self.depth_attr)
            obj = obj() if callable(obj) else obj
            for attr in ("Pixels", "PixelsDepth", "Depth"):
                if hasattr(obj, attr):
                    obj = getattr(obj, attr)
                    break
            arr = np.asarray(obj, dtype=float).reshape((CAMERA_RES, CAMERA_RES))
            return arr
        except Exception as ex:            # depth not usable; fall back to radar range
            log(f"Depth read failed ({ex}); using radar range only", "CameraAPI", st.Severity.Warning)
            self.depth_attr = None
            return None

    def capture_and_fuse(self, which, t, tracker, shuttle_pos, R_bw):
        cam = topCamera if which == "top" else bottomCamera
        props = st.CaptureImageProperties()
        props.nonphys_EV = CAMERA_EV
        props.FOV = CAMERA_FOV
        props.ResolutionX = CAMERA_RES
        props.ResolutionY = CAMERA_RES
        image = st.camera.CaptureImageBlocking(cam, props)
        if image is None:
            return 0
        if not self.probed:
            self._probe_depth(image)
        rgb = image.as_RGB8()
        r = np.asarray(rgb.PixelsR, dtype=np.float32).reshape((CAMERA_RES, CAMERA_RES))
        g = np.asarray(rgb.PixelsG, dtype=np.float32).reshape((CAMERA_RES, CAMERA_RES))
        b = np.asarray(rgb.PixelsB, dtype=np.float32).reshape((CAMERA_RES, CAMERA_RES))
        gray = vision.to_gray(r, g, b)
        blobs = vision.find_blobs(gray, BLOB_THRESHOLD)
        depth = self._depth_array(image)
        self.frames += 1
        self.blobs_total += len(blobs)

        yaw, pitch = self.aim[which]
        T_cb = vision.cam_to_body_dcm(yaw, pitch)
        origin = shuttle_pos + R_bw @ self.CAM_POS_BODY[which]
        fused = 0
        for bl in blobs[:25]:
            if bl.touches_edge and bl.area > 0.02 * CAMERA_RES ** 2:
                continue                                   # limb of Phobos / the ship
            d_cam = vision.pixel_to_dir_cam(bl.cx, bl.cy, CAMERA_RES, CAMERA_RES, CAMERA_FOV)
            d_world = R_bw @ (T_cb @ d_cam)
            ang_size = max(bl.width, bl.height) / vision.focal_px(CAMERA_RES, CAMERA_FOV)
            rng = None
            if depth is not None:
                y0, y1 = int(max(bl.cy - 1, 0)), int(min(bl.cy + 2, CAMERA_RES))
                x0, x1 = int(max(bl.cx - 1, 0)), int(min(bl.cx + 2, CAMERA_RES))
                patch = depth[y0:y1, x0:x1]
                patch = patch[np.isfinite(patch) & (patch > 0.5)]
                if patch.size:
                    rng = float(np.median(patch))
            if tracker.ingest_bearing(t, origin, d_world, self.ang_sigma, range_m=rng,
                                      range_sigma=1.0, ang_size=ang_size) is not None:
                fused += 1
        return fused


# ----------------------------------------------------------------------------- start-up
t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0: datetime.datetime = t0_sim.as_datetime()


def sim_time() -> float:
    return (st.SimGlobals.SimClock.GetTimeNow().as_datetime() - t0).total_seconds()


nav = ShuttleNav()
tracker = DebrisTracker(sigma_pos=2.0)
att = AttitudeTracker()
cams = CameraManager()

state0 = shuttle.getLocVelAcc()
p_start = np.asarray(state0.loc_WRT_ExprIn(phobosFrame), float)
p_goal = np.asarray(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float)
guidance = Guidance(p_start, p_goal, GP)
up_world = unit(p_start)                         # radial "up" from Phobos centre
log(f"Start {np.round(p_start, 1)}  goal {np.round(p_goal, 1)}  dist {np.linalg.norm(p_goal - p_start):.0f} m")

loop_hz = this.GetParam(st.VarType.double, "LoopFreqHz")
iteration = 0
last_log_t = -1e9
last_cmd_world = np.zeros(3)
exit_flag = False

# ----------------------------------------------------------------------------- main loop
while not exit_flag:
    time.sleep(1.0 / loop_hz)
    t = sim_time()

    # ---- own state: sim fix + thruster accelerometer -> ShuttleNav -----------------
    lva = shuttle.getLocVelAcc()
    p_fix = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
    v_fix = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
    q = np.asarray(shuttle.getRotation().Quat_WRT(phobosFrame), float)
    R_bw = att.body_to_world(q)
    omega_b = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), float)
    try:
        a_meas_body = np.asarray(shuttle.GetParam(st.VarType.doubleV3, ["JetSelection", "ResultantAccel"]), float)
    except Exception:
        a_meas_body = np.zeros(3)
    nav.propagate(t, R_bw @ a_meas_body)
    nav.fix(t, p_fix, v_fix)
    p, v = nav.pos, nav.vel

    # ---- radar -> world-frame tracks ----------------------------------------------
    if iteration % RADAR_EVERY == 0:
        try:
            tracker.ingest_radar(GetNearestDebrisLocations(RADAR_N), nav.pos_at, t, nav.vel_at)
        except Exception as ex:
            log(f"Radar read failed: {ex}", "Radar", st.Severity.Warning)
        tracker.prune(t)

    # ---- guidance -------------------------------------------------------------------
    out = guidance.command(t, p, v, tracker.tracks)
    a_world = out.accel_world
    last_cmd_world = a_world
    a_body = R_bw.T @ a_world

    # ---- attitude: nose along the flight direction, cameras look where we are going --
    x_des = unit(v) if np.linalg.norm(v) > 2.0 else unit(p_goal - p)
    alpha_body, att_err = attitude_command(R_bw, omega_b, x_des, up_world)
    if not ATT_HOLD:
        alpha_body = np.zeros(3)

    SetCommand(a_body, alpha_body)

    # ---- cameras: top looks ahead, bottom stares at the most pressing threat --------
    # the blocking capture (~0.1-0.3 s) is harmless during the long main burn or coast,
    # but we never stall the loop in the middle of a trim or the terminal steer
    quiet = out.mode in ("coast", "burn")
    if (t - cams.last_t) >= CAMERA_PERIOD_S and (quiet or not CAMERA_ONLY_WHEN_COASTING) and not out.done:
        cams.last_t = t
        try:
            cams.point("top", R_bw.T @ x_des)
            threat_dir = None
            # stare at the conflict closest in time, else the soonest tube object
            pool = out.threats or guidance.last_threats
            if pool:
                th = min(pool, key=lambda th: th.tau)
                threat_dir = R_bw.T @ unit(th.track.pos_at(t) - p)
            if threat_dir is not None and threat_dir[2] < 0:
                cams.point("bottom", threat_dir)
                which = "bottom"
            else:
                if threat_dir is not None:
                    cams.point("top", threat_dir)
                which = "top"
            cams.capture_and_fuse(which, t, tracker, p, R_bw)
        except Exception as ex:
            log(f"Camera step failed: {ex}", "Camera", st.Severity.Warning)

    # ---- status ------------------------------------------------------------------------
    if t - last_log_t >= LOG_EVERY_S:
        last_log_t = t
        n_cam_tracks = sum(1 for tr in tracker.tracks if tr.source == "camera")
        dmg = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
        fuel = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        pl = guidance.plan
        plan_s = (f"plan#{guidance.n_plans} off={pl.offset:.0f}m v={pl.speed:.0f} margin={min(pl.min_margin, 99):.0f}m "
                  if pl is not None else "plan=none ")
        log(f"t={t:6.1f}s {out.mode:10s} {plan_s}rem={out.remaining:7.1f}m v={out.v_along:5.1f}m/s "
            f"|v_lat|={np.linalg.norm(v - np.dot(v, unit(p_goal - p)) * unit(p_goal - p)):4.2f} "
            f"tracks={len(tracker.tracks)}(+{n_cam_tracks}cam) tube={out.in_tube} "
            f"conf={len(out.threats)} dv={out.dv_needed:4.2f} att={math.degrees(att_err):4.1f}deg "
            f"dmg={dmg:.0f}J fuel={fuel:.3f}kg frames={cams.frames}")
        if out.threats:
            th = min(out.threats, key=lambda th: th.tau)
            log(f"  nearest conflict: id{th.track.id} tau={th.tau:4.1f}s miss={np.linalg.norm(th.miss):5.1f}m "
                f"need={th.clearance:4.1f}m diam={th.track.diameter:4.1f}m", "Threat")

    if out.done:
        SetCommand(np.zeros(3), np.zeros(3))

    iteration += 1

st.leave_sim()
