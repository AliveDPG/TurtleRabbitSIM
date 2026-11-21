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

# Escape from Phobos flight controller.  This file is the sim I/O, the sensor scheduling and the control loop;
# the work is done in
#   depth_vision.py    rock detection in depth images, camera tracks, radar / camera fusion
#   radar_tracking.py  shuttle state history and radar rock tracks
#   guidance.py        score-weighted velocity planner
#   thrust_law.py      thrust and attitude law
#   thrusters.py       replica of the jet-selection actuator, and the jet pulser
#   pad_watch.py       camera views of the pad surroundings during the launch

# Flight profile.
CRUISE_SPEED_MPS = 7.0            # speed the launch burn aims for
LOOKAHEAD_S = 22.0                # collision look-ahead of the planner ...
EARLY_LOOKAHEAD_S = 22.0          # ... and during the first EARLY_LOOKAHEAD_UNTIL_S of flight, while tracks are
EARLY_LOOKAHEAD_UNTIL_S = 30.0    #     young: most dodges for rocks further ahead than this were needless there
WINDOW_EDGE_P_MAX = 0.0           # a rock whose closest approach lies beyond the look-ahead window costs nothing
                                  # while its hit probability is below this (0 = rule off)
HIT_COST_PTS = 600.0              # planner's price of a predicted hit, in score points (0 = never dodge)
P_GATE = 0.3                      # a rock counts once the plan in force has this chance of hitting it (0 = off)

# Pre-launch command.  The first loop tick comes ~2.2 s after the sim starts (imports, set-up).  Rather than sit
# on the pad until then, one held command is sent before the imports: +z thrust together with a -y pitch (the
# burn direction starts 4 deg off body +z and the nose has to come over to it).  The actuator shares jets between
# the two and gives 1.24 x the +z thrust plus 0.66 x the rotation.  Checkpoints between the set-up steps cut the
# rotation once the pitch rate reaches PRELAUNCH_RATE and stop everything after PRELAUNCH_MAX_S; the thrust law
# then takes over mid-slew.
PRELAUNCH_RATIO = 0.5             # rad/s^2 of pitch per m/s^2 of thrust in the command
PRELAUNCH_RATE = 0.15             # rad/s
PRELAUNCH_MAX_S = 8.0
ZERO = np.zeros(3)

t0_sim = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"]).as_datetime()


def sim_time():
    return (st.SimGlobals.SimClock.GetTimeNow().as_datetime() - t0_sim).total_seconds()


prelaunch = dict(on=False, t_on=0.0, cut=False)


def prelaunch_start():
    """Send the held command if the shuttle is at rest with the goal within 25 deg of body +z; True if sent."""
    lva = shuttle.getLocVelAcc()
    p = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
    v = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
    R = np.asarray(shuttle.getRotation().DCM_WRT(phobosFrame), float)
    g = np.asarray(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float) - p
    g_body = R @ (g / max(float(np.linalg.norm(g)), 1e-9))
    axis = np.cross(np.array([1.0, 0.0, 0.0]), g_body)        # the pitch-over axis: must be close to body -y
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)
    t = sim_time()
    if (float(np.linalg.norm(v)) > 0.3 or g_body[2] < np.cos(np.radians(25.0))
            or float(axis[1]) > -np.cos(np.radians(20.0)) or t > 5.0):
        return False
    sc = min(0.45, 0.045 / max(PRELAUNCH_RATIO, 1e-3))        # stays under the actuator's input clips
    SetCommand(np.array([0.0, 0.0, sc]), np.array([0.0, -sc * PRELAUNCH_RATIO, 0.0]))
    prelaunch.update(on=True, t_on=t)
    fatal_hook = sys.excepthook

    def jets_off_hook(exctype, value, tb):
        SetCommand(ZERO, ZERO)                 # a crash in the set-up must not leave the jets on
        fatal_hook(exctype, value, tb)
    sys.excepthook = jets_off_hook
    return True


def prelaunch_check():
    """Called between the set-up steps while the pre-launch command is held."""
    if not prelaunch["on"]:
        return
    t = sim_time()
    om = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), float)
    if t - prelaunch["t_on"] > PRELAUNCH_MAX_S:
        SetCommand(ZERO, ZERO)
        prelaunch["on"] = False
    elif not prelaunch["cut"] and (-float(om[1]) >= PRELAUNCH_RATE or float(np.linalg.norm(om)) >= PRELAUNCH_RATE + 0.05):
        SetCommand(np.array([0.0, 0.0, 0.45]), ZERO)           # +z thrust only from here
        prelaunch["cut"] = True


prelaunch_sent = prelaunch_start()

# Robust version: environment switches (defaults = the frozen baseline's behaviour).
RB_WATCH = os.environ.get("RB_WATCH", "0") == "1"     # pre-launch watchdog thread (failure mode 3)
RB_DELAY = float(os.environ.get("RB_DELAY", "0"))     # TEST ONLY: inject a start-up delay of this many s
RB_NEAR = os.environ.get("RB_NEAR", "0") == "1"       # near-ship aim: lag-aware miss + uncorrectable-miss price (mode 2)
RB_NEAR_FROM = float(os.environ.get("RB_NEAR_FROM", "1e9"))   # lag-aware miss only inside this distance (m)
RB_GATE_HOLD = float(os.environ.get("RB_GATE_HOLD", "0"))   # gate memory, s (mode 1: dodge / re-aim flip-flop)
RB_GATE_MODE = os.environ.get("RB_GATE_MODE", "hold")     # hold / excess / short / hyst (see guidance.GuideParams)
RB_RECOVER_SCORE = os.environ.get("RB_RECOVER_SCORE", "0") == "1"   # after a hit: score-formula time price, slower re-aims down to 0.4 x, time-cap guard
RB_RECOVER_PLAN = os.environ.get("RB_RECOVER_PLAN", "0") == "1"   # after a hit (damage jump or tumble): slower re-aim candidates for 90 s
RB_KICK_STRONG = float(os.environ.get("RB_KICK_STRONG", "0"))   # TEST ONLY: with RB_KICK, brake on -x this many s first (60 s = ~10 m/s)
RB_KICK_DIR = os.environ.get("RB_KICK_DIR", "x")   # TEST ONLY: strong kick direction, x = brake, y = sideways
RB_KICK = os.environ.get("RB_KICK", "")   # TEST ONLY "T": from T s, brake on -x and spin up to 0.5 rad/s (hit-like upset)
RB_PADPAUSE = os.environ.get("RB_PADPAUSE", "0") == "1"   # burn pauses allowed during the pad watch too (first 45 s)
RB_KICK_T = float(os.environ.get("RB_KICK_T", "0"))   # TEST ONLY: from this time, yaw the hull until the nose is 90 deg off, then release

# Pre-launch watchdog.  The checkpoints between the set-up steps only run when a step finishes: one slow step
# (a slow disk, a loaded machine) left the held pitch command unchecked and the hull over-rotated (0.28 rad/s at an
# 8 s start instead of 0.15).  A daemon thread runs the same check every 50 ms until the control loop takes over.
import threading
prelaunch_lock = threading.Lock()
prelaunch["handover"] = False
if RB_WATCH and prelaunch_sent:
    _check_unlocked = prelaunch_check

    def prelaunch_check():
        with prelaunch_lock:
            if not prelaunch["handover"]:
                _check_unlocked()

    def _prelaunch_watch():
        while not prelaunch["handover"] and prelaunch["on"]:
            try:
                prelaunch_check()
            except Exception:
                pass
            time.sleep(0.05)
    threading.Thread(target=_prelaunch_watch, name="prelaunch_watch", daemon=True).start()
if RB_DELAY > 0:
    time.sleep(RB_DELAY)

# The imports take over a second; the pre-launch command is held meanwhile.

import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
prelaunch_check()
import depth_vision as vision
prelaunch_check()
import guidance
prelaunch_check()
from radar_tracking import ShuttleNav, DebrisTracker
prelaunch_check()
from thrusters import Pulser
prelaunch_check()
from thrust_law import ThrustLaw
prelaunch_check()
from pad_watch import PadViews
prelaunch_check()

KEEP_OUT = 40.0                  # m: the run ends inside this radius of the target
RADAR_N = 500
RADAR_EVERY = 6                  # ticks between radar reads
PLAN_PERIOD_S = 0.2              # sim seconds between planner runs
FRAME_PERIOD_S = 0.25            # sim seconds between depth captures
LOG_EVERY_S = 1.0
DET_MAX_RANGE = 450.0
MAX_ROCK_R = 40.0                # detections bigger than this are terrain or structures (largest rock: 26 m radius)
MAX_CAM_ROCK_R = 27.0            # floor of the camera-track radius cap
SHIP_EXCLUDE_M = 200.0           # the target ship itself shows up in the depth images
VIEW_MAX_OFF_DEG = 50.0          # skip a view whose (clipped) boresight is this far from where it should look
CAM_MAX_AGE_S = 2.5              # a camera track unseen for this long is dropped
CAM_MIN_HITS_FAR = 4             # camera tracks farther than CAM_NEAR_M need this many sightings to count
CAM_NEAR_M = 60.0
PAUSE_STEPS_S = (3.0, 6.0, 10.0, 15.0)    # burn pauses the planner may choose as a dodge during the launch burn
# A dodge against a threat closer than URGENT_TAU_S is flown at once for URGENT_HOLD_S: a full re-orientation
# first delays the first 5 s of a dodge by a third.
URGENT_TAU_S = 25.0
URGENT_HOLD_S = 10.0

# Pad watch: while the shuttle is within PAD_WATCH_M of the pad and PAD_WATCH_S into the flight, every other
# frame is a wide view of the pad surroundings and camera tracks are kept longer (a pad rock is seen only every
# ~2 s).  Blobs low over the pad are normally structures (pad, habitat, tanks), but rocks rain onto the pad too:
# within PAD_ZONE_M of the pad such a blob is tracked during the pad watch, and at any time a track there
# reaches the planner only once it has PAD_ZONE_MIN_N sightings, has moved PAD_ZONE_MOVE_M (structures do not
# move), sits PAD_ZONE_ALT_M above the pad plane and is at least PAD_ZONE_RMIN_M in radius.  Of the blobs that
# moved without being rocks 49 of 55 sat at -9..+1 m; the pad rocks were at 7-12 m with radii of 7-27 m.
PAD_WATCH_M = 60.0
PAD_WATCH_S = 45.0
PAD_TRACK_AGE_S = 4.0
PAD_ZONE_M = 100.0
PAD_ZONE_MIN_N = 3
PAD_ZONE_MOVE_M = 6.0
PAD_ZONE_ALT_M = 4.0
PAD_ZONE_RMIN_M = 1.5

# Calibrated prediction-error model, per obstacle source and track maturity.  The miss-plane error is a core
# Gaussian of sigma = sqrt((ksp sigma_pos)^2 + (ksv sigma_vel age)^2 + floor^2), growing by ksv sigma_vel per
# second of prediction, plus a heavy tail of weight eps and sigma x wide.  The filters' own sigmas are scaled
# because their largest axis is mostly depth while the miss plane is cross-range.
#                (ksp, ksv, floor m, eps, wide)
ERR_CAMERA = ((0.0, 0.6, 5.0, 0.45, 4.8),       # up to 6 sightings
              (0.0, 0.5, 2.5, 0.25, 11.8),      # 7-20
              (0.6, 0.24, 1.6, 0.10, 14.0))     # more than 20
ERR_RADAR = (0.0, 0.43, 1.2, 0.06, 10.6)         # more than 6 returns
ERR_RADAR_YOUNG = (0.67, 0.73, 1.0, 0.10, 6.4)
ERR_FUSED = (0.0, 0.36, 1.0, 0.19, 17.7)
# For KICK_T s after the radar saw a contact on a rock, a fused or camera obstacle's velocity sigma is KICK_K x
# the model's and at least KICK_SV: the velocity error is 1.5-3.2 m/s at the first return after a contact and
# still 0.4-0.6 m/s 5 to 11 s later, at a claimed 0.12-0.26 m/s.
KICK_T, KICK_K, KICK_SV = 12.0, 3.0, 0.45

VIEWS = [                        # (camera, yaw offset, pitch offset, FOV, resolution) relative to the flight direction
    ("top", 0.0, -12.0, 70.0, 320),
    ("bottom", 0.0, 12.0, 70.0, 320),
    ("top", 50.0, -30.0, 100.0, 256),
    ("bottom", 85.0, 20.0, 100.0, 256),      # near-sideways: fast rocks (up to ~17 m/s) arrive from the side
    ("top", 0.0, -12.0, 70.0, 320),
    ("bottom", 0.0, 12.0, 70.0, 320),
    ("bottom", -50.0, 30.0, 100.0, 256),
    ("top", -85.0, -20.0, 100.0, 256),
    ("top", 0.0, -12.0, 70.0, 320),
    ("bottom", 0.0, 12.0, 70.0, 320),
    ("top", -50.0, -30.0, 100.0, 256),
    ("bottom", -85.0, 20.0, 100.0, 256),
    ("top", 0.0, -12.0, 70.0, 320),
    ("bottom", 0.0, 12.0, 70.0, 320),
    ("bottom", 50.0, 30.0, 100.0, 256),
    ("top", 85.0, -20.0, 100.0, 256),
]


def log(msg, tag="Flight", sev=None):
    st.OnScreenLogMessage(msg, tag, sev if sev is not None else st.Severity.Info)


def R_w2b_now():
    return np.asarray(shuttle.getRotation().DCM_WRT(phobosFrame), float)


class DepthCams:
    """Cycles the two pan-tilt cameras through the views and collects the depth images."""

    def __init__(self):
        self.k = 0
        self.pending = None          # capture in flight: (id, fov, res, t_request, cam_pos, R_w2c)
        self.pointed = None          # view the camera has been turned to: (camera, fov, res, t, yaw, pitch)
        self.ready = []              # (capture, image) pairs, filled by the image callback
        self.last_req_t = -1e9
        self.frames = 0

    def step(self, t, d_body, R_w2b):
        """Two phases: point the next view, then - once the API reports the camera at the commanded orientation
        (the render uses the settled pose) - request an asynchronous depth capture."""
        if self.pending is not None:
            if t - self.pending[3] > 2.0:            # lost frame
                self.pending = None
            else:
                return
        if self.pointed is None:
            if t - self.last_req_t < FRAME_PERIOD_S - 0.08:
                return
            view = None
            if pad_watch_on and padviews.next_is_pad():
                view = padviews.next_view(shuttle_pos, R_w2b)
            if view is None:
                yaw0, pitch0 = vision.yaw_pitch_for(d_body)
                for _ in range(len(VIEWS)):
                    name, dyaw, dpitch, fov, res = VIEWS[self.k % len(VIEWS)]
                    self.k += 1
                    y = float(np.clip(yaw0 + dyaw, -89, 89))
                    pt = float(np.clip(pitch0 + dpitch, -50, 0) if name == "top" else np.clip(pitch0 + dpitch, 0, 50))
                    off = math.degrees(math.acos(float(np.clip(vision.boresight_body(y, pt) @ guidance.unit(d_body), -1, 1))))
                    if off - abs(dyaw) - abs(dpitch) * 0.5 <= VIEW_MAX_OFF_DEG:
                        break
                else:
                    return
                view = name, y, pt, fov, res
            name, y, pt, fov, res = view
            PanTiltCamera(topCamera if name == "top" else bottomCamera, y, pt)
            self.pointed = (name, fov, res, t, y, pt)
            return
        name, fov, res, t_pt, y, pt = self.pointed
        if t - t_pt < 0.05 or t - self.last_req_t < FRAME_PERIOD_S:
            return
        cam = topCamera if name == "top" else bottomCamera
        R_w2c = np.asarray(cam.getRotation().DCM_WRT(phobosFrame), float)
        bore_api = R_w2c.T @ np.array([1.0, 0.0, 0.0])
        bore_cmd = R_w2b.T @ vision.boresight_body(y, pt)
        if float(bore_api @ bore_cmd) < math.cos(math.radians(1.0)):
            if t - t_pt > 1.0:                       # never settled: re-point next time
                self.pointed = None
            return
        props = st.CaptureImageProperties()
        props.nonphys_EV = 10.0
        props.FOV = fov
        props.ResolutionX = res
        props.ResolutionY = res
        props.output_mode = st.OutputMode.Depth_cm
        cam_pos = np.asarray(cam.getLocation().WRT_ExprIn(phobosFrame), float)
        cid = st.camera.CaptureImage(cam, props)
        pend = self.pending = (cid, fov, res, t, cam_pos, R_w2c)
        self.pointed = None
        self.last_req_t = t

        def on_image(image):
            try:
                self.ready.append((pend, np.asarray(image.as_f32().Pixels, dtype=np.float32)))
            except Exception:                    # a bad frame must not kill the callback thread
                self.ready.append((pend, None))

        st.camera.OnImageReceived(cid, on_image)

    def collect(self, tracker, ground_fn):
        while self.ready:
            pend, arr = self.ready.pop(0)
            if self.pending is not None and pend[0] == self.pending[0]:
                self.pending = None
            _, fov, res, t_req, cam_pos, R_w2c = pend
            if arr is None or arr.size != res * res:
                continue
            depth = arr.reshape(res, res) / 100.0
            dets = vision.detect_rocks(depth, fov, cam_pos, R_w2c, t_req, max_range=DET_MAX_RANGE, min_range=6.0,
                                       ground_fn=ground_fn)
            tracker.ingest(dets, cam_pos, math.radians(fov) / res)
            shipmap.add_depth(depth, fov, cam_pos, R_w2c)
            self.frames += 1


nav = ShuttleNav()
radar = DebrisTracker()
camtracks = vision.CamTracker(max_age_s=PAD_TRACK_AGE_S)
pad_watch_on = True
padviews = PadViews()
cams = DepthCams()

p_start = np.asarray(shuttle.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float)
p_goal = np.asarray(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float)
r_pad = float(np.linalg.norm(p_start))
up_pad = p_start / r_pad
shuttle_pos = p_start.copy()
GP = guidance.GuideParams(v_cruise=CRUISE_SPEED_MPS, keep_out_m=KEEP_OUT, edge_drop=WINDOW_EDGE_P_MAX > 0,
                          edge_pmax=WINDOW_EDGE_P_MAX, p_gate=P_GATE, hit_pts=HIT_COST_PTS,
                          short_pts=10.0 if HIT_COST_PTS > 0 else 0.0, near_fix=RB_NEAR, near_lag_from_m=RB_NEAR_FROM,
                          gate_hold_s=RB_GATE_HOLD, gate_mode=RB_GATE_MODE,
                          recover_score=RB_RECOVER_SCORE, recover_slow=(0.4, 0.6, 0.8) if RB_RECOVER_SCORE else (0.6, 0.8))
pulser = Pulser(shuttle.getMass(), np.asarray(shuttle.getInertia(shuttleFrame), float))
prelaunch_check()
for _pair in ((4, 3), (4, 2), (0, 3), (0, 2)):      # solve the pitch-over pairs (+z / +x with -+y) now, not in flight
    pulser.combo_entries(*_pair)
prelaunch_check()
law = ThrustLaw(pulser)
planner = guidance.Planner(p_goal, GP, R_w2b_fn=R_w2b_now, caps_body=[c[2] for c in pulser.cap_t])
shipmap = vision.StaticMap(p_goal, radius=SHIP_EXCLUDE_M, voxel=4.0, min_hits=2)
# The safe way in is along the target ship's own z axis: its ring / disc is 40-112 m out around the equator, and
# 11 of 12 ship hits were 76-106 deg off the axis.
try:
    _ax = guidance.unit(np.asarray(target.getRotation().DCM_WRT(phobosFrame), float)[2, :])
    if float(np.dot(p_start - p_goal, _ax)) < 0:
        _ax = -_ax
    planner.ship_axis = _ax
except Exception as ex:
    log(f"ship axis unavailable ({ex!r}); no approach cone", "Plan", st.Severity.Warning)
log(f"start {np.round(p_start, 1)} goal {np.round(p_goal, 1)} dist {np.linalg.norm(p_goal - p_start):.0f} m, cruise "
    f"{CRUISE_SPEED_MPS} m/s, look-ahead {EARLY_LOOKAHEAD_S}/{LOOKAHEAD_S} s, pre-launch command "
    f"{'sent' if prelaunch_sent else 'not sent'}, ship axis {planner.ship_axis}")


def ground_fn(center, r):
    """Not a rock: terrain- or structure-sized blobs, our own hull, anything near or below the pad plane, the ship."""
    if r > MAX_ROCK_R:
        return True
    if float(np.linalg.norm(center - shuttle_pos)) < 7.0 + r:
        return True                          # our own hull (wide views see it at 4-6 m)
    alt = float(np.dot(center - p_start, up_pad))
    d_pad = float(np.linalg.norm(center - p_start))
    if not (pad_watch_on and d_pad < PAD_ZONE_M):
        if alt < 20.0 and d_pad < 100.0:
            return True                      # pad, habitat, tanks, processor
        if alt < 15.0 and float(np.linalg.norm(center - shuttle_pos)) > 60.0:
            return True
        if float(np.linalg.norm(center)) < r_pad + 12.0 and d_pad < 600.0:
            return True
    return float(np.linalg.norm(center - p_goal)) < SHIP_EXCLUDE_M


def in_pad_zone(c):
    d_pad = float(np.linalg.norm(c - p_start))
    if d_pad >= PAD_ZONE_M:
        return False
    return (float(np.dot(c - p_start, up_pad)) < 20.0 and d_pad < 100.0) or float(np.linalg.norm(c)) < r_pad + 12.0


first_seen_at = {}               # camera track id -> position at its first sighting


def cam_tracks_for_planner(t):
    """Camera tracks, without those in the pad zone that have not yet shown themselves to be rocks."""
    out = []
    for tr in camtracks.tracks:
        first = first_seen_at.setdefault(tr.id, tr.pos.copy())
        c = tr.pos_at(t)
        if in_pad_zone(c) and (tr.n < PAD_ZONE_MIN_N or float(np.linalg.norm(tr.pos - first)) < PAD_ZONE_MOVE_M
                               or float(np.dot(c - p_start, up_pad)) < PAD_ZONE_ALT_M or float(tr.radius) < PAD_ZONE_RMIN_M):
            continue
        out.append(tr)
    if len(first_seen_at) > 5000:
        live = {tr.id for tr in camtracks.tracks}
        for k in [k for k in first_seen_at if k not in live]:
            del first_seen_at[k]
    return out


def obstacles(t):
    """Planner obstacles: the fused radar / camera rocks with the calibrated error model applied."""
    obl = vision.fuse_obstacles(radar.tracks, cam_tracks_for_planner(t), t, shuttle_pos, min_hits_far=CAM_MIN_HITS_FAR,
                                near_m=CAM_NEAR_M)
    if not obl:
        return None
    sig, sigv, eps, wide = [], [], [], []
    for o in obl:
        if o.src == "f":
            ksp, ksv, floor, ep, wd = ERR_FUSED
        elif o.src == "r":
            ksp, ksv, floor, ep, wd = ERR_RADAR_YOUNG if o.n <= 6 else ERR_RADAR
        else:
            ksp, ksv, floor, ep, wd = ERR_CAMERA[0 if o.n <= 6 else 1 if o.n <= 20 else 2]
        sv = ksv * o.svf
        if o.src != "r" and o.kick_age < KICK_T:
            sv = max(KICK_K * sv, KICK_SV)
        sig.append(math.sqrt((ksp * o.sp) ** 2 + (sv * o.age) ** 2 + floor ** 2))
        sigv.append(sv)
        eps.append(ep)
        wide.append(wd)
    return dict(ids=[o.name for o in obl], pos=np.array([o.pos for o in obl], float), vel=np.array([o.vel for o in obl], float),
                rad=np.array([o.radius for o in obl], float), sig=np.array(sig, float), sigv=np.array(sigv, float),
                eps=np.array(eps, float), wide=np.array(wide, float))


def relevant(obs, p, v):
    """Keep obstacles that can come within reach during the planning horizon."""
    if obs is None:
        return None
    rel = obs["pos"] - p
    dist = np.linalg.norm(rel, axis=1)
    reach = (float(np.linalg.norm(v)) + np.linalg.norm(obs["vel"], axis=1) + 6.0) * GP.horizon_s + obs["rad"] + 30.0
    keep = dist < reach
    # and in front of us or closing
    closing = -np.einsum("ij,ij->i", rel, obs["vel"] - v[None, :]) / np.maximum(dist, 1.0)
    keep &= (closing > -2.0) | (dist < 60.0)
    if not keep.any():
        return None
    idx = np.where(keep)[0]
    if len(idx) > 80:                        # nearest 80 by time-to-reach
        ttr = dist[idx] / np.maximum(closing[idx] + 1.0, 1.0)
        idx = idx[np.argsort(ttr)[:80]]
    return dict(ids=[obs["ids"][i] for i in idx], pos=obs["pos"][idx], vel=obs["vel"][idx], rad=obs["rad"][idx],
                sig=obs["sig"][idx], sigv=obs["sigv"][idx], eps=obs["eps"][idx], wide=obs["wide"][idx])


loop_hz = this.GetParam(st.VarType.double, "LoopFreqHz")
iteration = 0
done = False
info = guidance.PlanInfo()
plan_fail = 0
n_rel = 0
att_err = 0.0
t_wake = time.perf_counter()
t_first = None
t_prev_tick = None
last_log_t = -1e9
last_plan_t = -1e9
last_threat_log = -1e9
last_avoid = (-1e9, 1e9)         # (time, time to the worst threat) of the last avoid switch
kick = dict(state="idle", t0=0.0)   # RB_KICK_T test state
hit = dict(dmg=0.0)                 # last damage reading (RB_RECOVER_PLAN)
kick2 = dict(state="idle", t0=0.0)  # RB_KICK test state
prelaunch_check()

while True:
    # same cadence as time.sleep(1 / loop_hz) at 1x; at sim timescale k each tick spans the same sim time
    timescale = st.SimGlobals.SimClock.GetTimescale() or 1.0
    time.sleep(max(0.0, 1.0 / (loop_hz * timescale) + (time.perf_counter() - t_wake) * (1.0 / timescale - 1.0)))
    t_wake = time.perf_counter()
    t = sim_time()
    if t_first is None:
        t_first = t
        with prelaunch_lock:             # the watchdog must not send anything once the loop commands the jets
            prelaunch["handover"] = True
        log(f"first loop tick at t={t:.2f}s (watchdog {'on' if RB_WATCH else 'off'}, injected delay {RB_DELAY:.1f}s, "
            f"pre-launch cut {'yes' if prelaunch['cut'] else 'no'}; near {int(RB_NEAR)}/{RB_NEAR_FROM:.0f} gate_hold {RB_GATE_HOLD:.0f} {RB_GATE_MODE} brake {os.environ.get('RB_BRAKE', '0')} recover {os.environ.get('RB_RECOVER', '0')}/{os.environ.get('RB_RECOVER_DEG', '0')} kick {RB_KICK_T:.0f} padpause {int(RB_PADPAUSE)} recover_plan {int(RB_RECOVER_PLAN)} burn_deg {os.environ.get('RB_RECOVER_BURN_DEG', '0')} kick2 {RB_KICK or '-'}/{RB_KICK_STRONG:.0f} rscore {int(RB_RECOVER_SCORE)} dvburn {os.environ.get('RB_RECOVER_DVBURN', '0')})")

    lva = shuttle.getLocVelAcc()
    p = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
    v = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
    R_w2b = np.asarray(shuttle.getRotation().DCM_WRT(phobosFrame), float)
    omega_b = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame), float)
    shuttle_pos = p
    nav.propagate(t, R_w2b.T @ pulser.realised_body)     # the acceleration our own pulses produced
    nav.fix(t, p, v)

    if done:
        SetCommand(ZERO, ZERO)
        continue
    if float(np.linalg.norm(p - p_goal)) <= KEEP_OUT - 0.5:
        done = True
        log(f"inside keep-out at t={t:.2f}s fuel={shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed']):.3f}kg "
            f"dmg={shuttle.GetParam(st.VarType.double, ['Stats', 'Damage']):.0f}J")
        SetCommand(ZERO, ZERO)
        continue

    if pad_watch_on and (t > PAD_WATCH_S or float(np.linalg.norm(p - p_start)) > PAD_WATCH_M):
        pad_watch_on = False
        camtracks.max_age_s = CAM_MAX_AGE_S
        log(f"pad watch off at t={t:.1f}s, {np.linalg.norm(p - p_start):.0f} m from the pad")

    # ---- sensors
    if iteration % RADAR_EVERY == 0:
        try:
            radar.ingest_radar(GetNearestDebrisLocations(RADAR_N), nav.pos_at, t, nav.vel_at)
        except Exception as ex:
            log(f"radar read failed: {ex}", "Radar", st.Severity.Warning)
        radar.prune(t)
        if radar.tracks:
            # A camera blob's radius is capped at what the radar says rocks can be, since merged views would
            # otherwise ratchet it up - but never below the largest rock class: a big rock capped to 9 m kept a
            # hit probability of 0.1 through 30 s of clean sightings.
            vision.CamTrack.R_MAX = max(vision.CamTrack.R_MAX, 0.55 * max(tr.diameter for tr in radar.tracks), MAX_CAM_ROCK_R)
    v_dir = planner.v_tgt if planner.v_tgt is not None else (p_goal - p)
    try:
        cams.collect(camtracks, ground_fn)
        cams.step(t, R_w2b @ guidance.unit(v_dir), R_w2b)
    except Exception as ex:
        log(f"camera step failed: {ex!r}", "Camera", st.Severity.Warning)
    if iteration % 30 == 0:
        camtracks.prune(t)

    # ---- plan
    if t - last_plan_t >= PLAN_PERIOD_S:
        last_plan_t = t
        GP.horizon_s = EARLY_LOOKAHEAD_S if (t - t_first) < EARLY_LOOKAHEAD_UNTIL_S else LOOKAHEAD_S
        GP.pause_steps = () if (pad_watch_on and not RB_PADPAUSE) else PAUSE_STEPS_S
        if RB_RECOVER_PLAN or RB_RECOVER_SCORE:
            dmg_now = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            if dmg_now > hit["dmg"] + 1.0 or law.tumbling:
                if t > planner.recover_until - 89.0:
                    log(f"t={t:.1f} RECOVERY: {'damage +%.0f J' % (dmg_now - hit['dmg']) if dmg_now > hit['dmg'] + 1.0 else 'tumble'}, slower re-aims offered for 90 s", "Plan")
                planner.recover_until = t + 90.0
            hit["dmg"] = dmg_now
        v_prev = planner.v_tgt
        try:
            obs = relevant(obstacles(t), p, v)
            n_rel = 0 if obs is None else len(obs["rad"])
            v_tgt, info = planner.plan(t, p, v, obs, t - t_first,
                                       static_tree=shipmap.tree() if float(np.linalg.norm(p - p_goal)) < 600.0 else None)
            plan_fail = 0
        except Exception as ex:          # a planner fault must not stop the loop: the plan in force stays
            plan_fail += 1
            info = guidance.PlanInfo()
            if planner.v_tgt is None:
                planner.v_tgt = planner.aim_velocity(p, CRUISE_SPEED_MPS)
            if plan_fail == 1:
                log(f"plan failed: {ex!r}", "Plan", st.Severity.Warning)
        if info.threats and t - last_threat_log > 1.0:
            last_threat_log = t
            log(f"t={t:6.2f} THREATS " + " | ".join(f"{i} P={pp:.2f} tau={ta:.1f} miss={mm:.0f} r={rr:.0f} rng={rg:.0f} sig={sg:.0f} Pbest={pb:.2f}"
                                                    for i, pp, ta, mm, rr, rg, sg, pb in info.threats) +
                f" | best_coll={info.best_coll:.0f} cur_coll={info.costs.get('coll_cur', 0):.0f}", "Threat")
        if info.reason == "avoid" and not info.kind.startswith("pause"):     # a pause needs no urgent burn
            last_avoid = (t, info.worst_tau)
        if info.reason and v_prev is not None:
            log(f"t={t:6.2f} PLAN {info.reason}: dv={np.linalg.norm(v_tgt - v_prev):.2f} m/s (|v_tgt|={np.linalg.norm(v_tgt):.2f}) "
                f"worst={info.worst_id} P={info.worst_p:.2f} tau={info.worst_tau:.1f}s "
                f"miss={info.miss_m:.1f}m costs={ {k: round(x, 1) for k, x in info.costs.items()} } kind={info.kind}", "Plan")

    # ---- the target velocity is a coast plan: it falls with gravity exactly like we do
    dt_tick = (t - t_prev_tick) if t_prev_tick is not None else None
    if t_prev_tick is not None:
        planner.v_tgt = planner.v_tgt + guidance.coast_acc(p, planner.v_tgt) * (t - t_prev_tick)
    t_prev_tick = t
    v_tgt = planner.v_tgt

    # ---- thrust toward the target velocity
    urgent = (t - last_avoid[0] < URGENT_HOLD_S) and (last_avoid[1] < URGENT_TAU_S)
    try:
        a_body, alpha_body, att_err = law.step(R_w2b, omega_b, v, v_tgt, dt=dt_tick, hold=t < planner.hold_until, urgent=urgent)
    except Exception as ex:              # a fault in the law must not stop the loop: coast this tick
        a_body, alpha_body = ZERO, ZERO
        if t - last_log_t >= LOG_EVERY_S:
            log(f"thrust law failed: {ex!r}", "Plan", st.Severity.Warning)
    if RB_KICK_T > 0 and t >= RB_KICK_T and kick["state"] != "released":
        if kick["state"] == "idle":
            kick.update(state="kicking", t0=t)
            log(f"KICK start at t={t:.1f}s att={math.degrees(att_err):.1f}")
        if att_err < math.radians(90.0) and t - kick["t0"] < 60.0:
            a_body, alpha_body = ZERO, np.array([0.0, 0.0, float(np.clip(1.2 * (0.1 - omega_b[2]), -0.05, 0.05))])   # yaw at 0.1 rad/s
        else:
            kick["state"] = "released"
            log(f"KICK released at t={t:.1f}s att={math.degrees(att_err):.1f} w={np.linalg.norm(omega_b):.3f}")
    if RB_KICK and t >= float(RB_KICK) and kick2["state"] != "released":
        if kick2["state"] == "idle":
            kick2.update(state="kicking", t0=t)
            log(f"KICK2 start at t={t:.1f}s |v|={np.linalg.norm(v):.2f}")
        if RB_KICK_STRONG and t - kick2["t0"] < RB_KICK_STRONG:
            a_body, alpha_body = (np.array([0.0, 1.0, 0.0]) if RB_KICK_DIR == "y" else np.array([-1.0, 0.0, 0.0])), ZERO   # strong kick: brake (x) or sideways (y) this long
        elif not RB_KICK_STRONG and t - kick2["t0"] < 15.0:
            a_body, alpha_body = np.array([0.0, 1.0, 0.0]), ZERO       # 15 s sideways on body +y: ~2 m/s off the plan
        elif np.linalg.norm(omega_b) < 0.5 and t - kick2["t0"] < 30.0 + RB_KICK_STRONG:
            a_body, alpha_body = ZERO, np.array([0.0, 0.3, 0.3])       # then spin up to ~0.5 rad/s (29 deg/s)
        else:
            kick2["state"] = "released"
            log(f"KICK2 released at t={t:.1f}s |v|={np.linalg.norm(v):.2f} dv={np.linalg.norm(v_tgt - v):.2f} w={np.linalg.norm(omega_b):.2f}")
    SetCommand(*pulser.step(a_body, alpha_body))         # on/off jet pulses that average to the request

    if t - last_log_t >= LOG_EVERY_S:
        last_log_t = t
        log(f"t={t:6.1f} rem={np.linalg.norm(p_goal - p):7.1f} |v|={np.linalg.norm(v):5.2f} |vt|={np.linalg.norm(v_tgt):5.2f} "
            f"dv={np.linalg.norm(v_tgt - v):4.2f} att={math.degrees(att_err):5.1f} w={np.linalg.norm(omega_b):.2f} mode={law.mode} "
            f"radar={len(radar.tracks)} cam={len(camtracks.tracks)} rel={n_rel} frames={cams.frames} "
            f"worst={info.worst_id}:P={info.worst_p:.2f} miss={info.miss_m:.0f} ship={len(shipmap.cells)} "
            f"switches={planner.n_switch} paused={planner.pause_used:.0f}s "
            f"fuel={shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed']):.3f} "
            f"dmg={shuttle.GetParam(st.VarType.double, ['Stats', 'Damage']):.0f}")
    iteration += 1
