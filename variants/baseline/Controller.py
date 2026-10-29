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
# BASELINE: straight to the ship. One vector burn to V_CRUISE toward the target centre, then  #
# coast. No debris avoidance, no attitude control, no cameras. Used as the comparison floor.  #
#############################################################################################

V_CRUISE = 15.0
A_MAX = 0.5
KEEP_OUT = 50.0
DEADBAND = 0.15

t0: datetime.datetime = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"]).as_datetime()
loop_hz = this.GetParam(st.VarType.double, "LoopFreqHz")
p_goal = np.asarray(target.getLocVelAcc().loc_WRT_ExprIn(phobosFrame), float)
st.OnScreenLogMessage("BASELINE controller: burn to %.0f m/s straight at the ship, then coast" % V_CRUISE, "Baseline", st.Severity.Info)

iteration = 0
arrived = False
last_log = -1e9
while True:
    time.sleep(1.0 / loop_hz)
    t = (st.SimGlobals.SimClock.GetTimeNow().as_datetime() - t0).total_seconds()
    lva = shuttle.getLocVelAcc()
    p = np.asarray(lva.loc_WRT_ExprIn(phobosFrame), float)
    v = np.asarray(lva.vel_WRT_ExprIn(phobosFrame), float)
    rel = p_goal - p
    dist = float(np.linalg.norm(rel))

    a_world = np.zeros(3)
    mode = "coast"
    if dist <= KEEP_OUT:
        arrived = True
        mode = "arrived"
    elif not arrived:
        v_t = V_CRUISE * rel / dist
        dv = v_t - v
        if np.linalg.norm(dv) > DEADBAND:
            a_world = dv / 0.5
            n = np.linalg.norm(a_world)
            if n > A_MAX:
                a_world *= A_MAX / n
            mode = "burn"
    a_body = st.frames.FramedVector(a_world, phobosFrame).ExprIn(shuttleFrame)
    SetCommand(np.asarray(a_body, float), np.zeros(3))

    if t - last_log >= 1.0:
        last_log = t
        fuel = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])
        dmg = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
        st.OnScreenLogMessage(f"t={t:6.1f}s {mode:8s} dist={dist:7.1f}m v={np.linalg.norm(v):5.1f}m/s fuel={fuel:.3f}kg dmg={dmg:.0f}J",
                              "Baseline", st.Severity.Info)
    iteration += 1
