# -*- coding: utf-8 -*-
"""Mini 4WD lap simulation in MuJoCo, ported from the Unreal Engine runtime (ue/m4wd_drive.py, OPT branch).

Same ingredients as UE:
  * course  = the convex collision hulls (UCX_) of the generated course parts, laid out like the UE level
  * car     = chassis box + 4 tyre spheres + 6 roller spheres, all frictionless (walls guide the car)
  * forces  = motor drive along the car's forward axis when the tyres touch the track,
              coasting drag, short brake, per-axle tyre side grip (UE formula), angular damping
  * control = ue/m4wd_ctrl.py (IMU-only speed controller, virtual LSM6DS3) - the very same Python code

Units: SI (m, kg, s), right-handed world (UE x, -y, z).  Body axes: x forward, y left, z up.
Design parameters (mass, COM, gear / vmax, rollers, driver loss, IMU range ...) come from a dict, see DEFAULT_DESIGN.
"""
import copy
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "generator"))
sys.path.insert(0, os.path.join(PKG, "ue"))
from layout_preview import read_fbx, build as build_layout, xform   # noqa: E402
import m4wd_ctrl as CTRL                                              # noqa: E402

_COLORS = []
LAYOUT = "LC_UP,LC_DOWN,R,R,R,R,S,S,R,R,R,R"     # UE level Mini4WD_CourseOut
START_PIECE = 8                                  # placed index of "07_S" (bridge is inserted after index 0)
LANE_OFFSET = 0.0585
G = 9.80665
DT = 1.0 / 960.0                                 # physics step (UE ran ~1/480 s; grip uses the UE step below)
DT_UE = 1.0 / 480.0
MOTOR_K = 13.0 * 0.1259 * 7.0                    # F0 * VMAX [N m/s] of the Mach-Dash calibration (UE)

DEFAULT_DESIGN = dict(
    mass=0.090,            # kg
    com_dz=-0.008,         # m, COM shift below the shape centroid (UE com_nudge -0.8 cm)
    com_dx=0.0,            # m, COM shift forward
    vmax=8.8,              # m/s no-load top speed at duty 1 (gear ratio / tyre radius trade VMAX against F0)
    motor_k=1.0,           # x MOTOR_K: motor strength (1 = Mach-Dash PRO calibration)
    drive_eff=1.0,         # voltage kept after battery+driver losses (1 = ideal); scales F0 and VMAX
    mu_tire=1.0,           # tyre/track friction (traction & side-grip limit)
    roll_decel=0.25,       # coasting drag [1/s]
    roller_front_x=0.0715, roller_rear_x=-0.0715, roller_y=0.042,   # m (car_spec)
    roller_r=0.0065,
    brake=0,               # 1: short brake available on the hardware
    imu_range_g=16.0,      # accelerometer full scale
    ang_damp=0.3,
    mu_wall=0.06,          # roller/wall drag coefficient (calibrated to the UE run FC_p0)
    solref_t=0.004,        # contact time constant [s]
    air_react=0,           # 1: model the drive train's spin (wheels + gears + rotor): in the air the motor torque spins
                           #    the wheels and the reaction pitches the body; on landing the speed mismatch slips
    drive_J=2.8e-6,        # drive-train inertia seen at the axles [kg m^2] (4 wheels ~1.2e-6, gears ~0.3e-6, rotor x gear^2 ~1.3e-6)
    tire_r=0.012,          # tyre radius [m]
    downforce=0.0,         # suction / aero downforce while the tyres touch the track, x weight (fan car); adds to grip
)


class ImuSensorR(CTRL.ImuSensor):
    """ImuSensor with a configurable accelerometer full scale (design variable)"""
    def __init__(self, range_g=16.0):
        super().__init__()
        self.range_g = range_g

    def update(self, dt, R, v_world):
        if self.R is None:
            self.R, self.v, self.acc_t = R, v_world, 0.0
            return None
        self.acc_t += dt
        if self.acc_t < 1.0 / self.RATE_HZ - 1e-6:
            return None
        h = self.acc_t
        a = [(v_world[i] - self.v[i]) / h for i in range(3)]
        a[2] += G
        Rm = CTRL._orthonormalize([[0.5 * (self.R[i][j] + R[i][j]) for j in range(3)] for i in range(3)])
        f = CTRL._mat_vec(CTRL._mat_t(Rm), a)
        M = CTRL._mat_mul(CTRL._mat_t(self.R), R)
        w = [0.5 * (M[2][1] - M[1][2]) / h, 0.5 * (M[0][2] - M[2][0]) / h, 0.5 * (M[1][0] - M[0][1]) / h]
        self.R, self.v, self.acc_t = R, v_world, 0.0
        rg = self.range_g
        acc_g = [max(-rg, min(rg, round(c / G, 3))) for c in f]
        gyro = [max(-2000.0, min(2000.0, round(math.degrees(c), 1))) for c in w]
        return h, acc_g, gyro


# ------------------------------------------------------------------ course
def _course():
    d = os.path.join(PKG, "fbx")
    man = json.load(open(os.path.join(d, "parts_manifest.json")))
    placed, _ = build_layout(man, LAYOUT.split(","))
    hulls, cache = [], {}
    _HULL_PART.clear()
    _COLORS.clear()
    curve_i = 0
    for part, W in placed:
        if part.startswith("Curve"):
            col = "0.35 0.75 0.25 1" if curve_i % 2 == 0 else "0.25 0.45 0.85 1"; curve_i += 1
        else:
            col = "0.12 0.12 0.13 1"
        if part not in cache:
            cache[part] = [v for k, v in sorted(read_fbx(os.path.join(d, man["parts"][part]["file"])).items()) if k.startswith("UCX_")]
        for V, _ in cache[part]:
            Vw = xform(V, W) * [0.01, -0.01, 0.01]            # UE cm -> RH m
            hulls.append(Vw); _COLORS.append(col); _HULL_PART.append(part)
    return placed, hulls


def _ue_xf(W):
    """placed transform (UE cm, deg) -> RH position (m) and yaw (rad)"""
    return np.array([W[0] * 0.01, -W[1] * 0.01, W[2] * 0.01]), -math.radians(W[3])


_COURSE = None
_COLORS = []
_HULL_PART = []                                    # course part name of each hull (same order as hulls)


def course():
    global _COURSE
    if _COURSE is None:
        placed, hulls = _course()
        p, yaw = _ue_xf(placed[START_PIECE][1])
        f = np.array([math.cos(yaw), math.sin(yaw), 0.0]); left = np.array([-f[1], f[0], 0.0])
        allv = np.vstack(hulls)
        lc_up = _ue_xf(placed[0][1])[0]                       # foot of the up ramp region
        # centre of the lane-change bases (bounds centre of each base, averaged; as m4wd_drive._course_info in UE)
        bases = [i for i, (part, _) in enumerate(placed) if part == "LaneChange_Base"]
        lc_c = None
        pts = [h for h, part in zip(hulls, _HULL_PART) if part == "LaneChange_Base"]
        if pts:
            half = len(pts) // len(bases) if bases else len(pts)
            groups = [np.vstack(pts[j:j + half]) for j in range(0, len(pts), half)]
            lc_c = np.mean([(g.min(0) + g.max(0)) / 2.0 for g in groups], axis=0)
        _COURSE = dict(placed=placed, hulls=hulls, fin_p=p, fin_f=f, fin_l=left,
                       lo=allv.min(0), hi=allv.max(0), lc=lc_up, lc_c=lc_c)
    return _COURSE


# ------------------------------------------------------------------ model
def _car_geoms(ds):
    """(type, pos, size) in body frame (x fwd, y left, z up, origin on the ground under the centre)"""
    g = [("box", (0.0, 0.0, 0.0165), (0.068, 0.035, 0.0135))]
    for x in (0.04, -0.04):
        for y in (0.0298, -0.0298):
            g.append(("sphere", (x, y, 0.012), (0.012,)))
    r = ds["roller_r"]
    for x, z in ((ds["roller_front_x"], 0.009), (ds["roller_rear_x"], 0.009), (ds["roller_rear_x"] + 0.0055, 0.022)):
        for s in (1, -1):
            g.append(("sphere", (x, s * ds["roller_y"], z), (r,)))
    return g


def _inertia(geoms, mass, com_dx, com_dz):
    """uniform-density composite of the shapes (like Chaos), scaled to `mass`, COM nudged"""
    vols, coms, Is = [], [], []
    for t, p, s in geoms:
        if t == "box":
            v = 8 * s[0] * s[1] * s[2]
            I = v / 3 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2, s[0] ** 2 + s[1] ** 2])
        else:
            v = 4 / 3 * math.pi * s[0] ** 3
            I = 0.4 * v * s[0] ** 2 * np.eye(3)
        vols.append(v); coms.append(np.array(p)); Is.append(I)
    V = sum(vols); c = sum(v * p for v, p in zip(vols, coms)) / V
    I = np.zeros((3, 3))
    for v, p, Ii in zip(vols, coms, Is):
        r = p - c
        I += Ii + v * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    I *= mass / V
    c = c + np.array([com_dx, 0.0, com_dz])
    return c, I


def build_model(design=None):
    ds = dict(DEFAULT_DESIGN, **(design or {}))
    C = course()
    p0 = C["fin_p"] + C["fin_f"] * 0.02 - C["fin_l"] * LANE_OFFSET     # right lane (inner when running clockwise)
    yaw0 = math.atan2(C["fin_f"][1], C["fin_f"][0])
    geoms = _car_geoms(ds)
    com, I = _inertia(geoms, ds["mass"], ds["com_dx"], ds["com_dz"])
    if ds.get("com_abs") is not None:            # mass properties from the CAD model (cad/chassis.py)
        com = np.array(ds["com_abs"], float)
        I = np.array(ds["inertia"], float)
    xs = ['<mujoco model="m4wd">',
          '<compiler angle="radian"/>',
          '<visual><headlight ambient="0.55 0.55 0.55" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/><quality shadowsize="4096"/></visual>',
          '<option timestep="%g" gravity="0 0 %g" integrator="implicitfast" cone="pyramidal"/>' % (DT, -G),
          '<default><geom condim="1" friction="0 0 0" solref="%g 1" solimp="0.95 0.99 0.001" margin="0.0005"/></default>',
          '<asset>']
    xs[4] = xs[4] % ds["solref_t"]
    for i, V in enumerate(C["hulls"]):
        xs.append('<mesh name="h%d" vertex="%s"/>' % (i, " ".join("%.5f" % v for v in V.ravel())))
    xs.append('</asset><worldbody>')
    xs.append('<light pos="0.45 -0.5 2.5" dir="0 0 -1" diffuse="0.5 0.5 0.5" castshadow="true"/>')
    xs.append('<geom name="floor" type="plane" size="5 5 0.1" pos="0 0 0" contype="1" conaffinity="1" rgba="0.78 0.76 0.72 1"/>')
    for i in range(len(C["hulls"])):
        xs.append('<geom type="mesh" mesh="h%d" contype="1" conaffinity="1" group="1" rgba="%s"/>' % (i, _COLORS[i] if i < len(_COLORS) else "0.2 0.2 0.2 1"))
    xs.append('<body name="car" pos="%.5f %.5f 0.0095" euler="0 0 %.6f">' % (p0[0], p0[1], yaw0))
    xs.append('<freejoint/>')
    xs.append('<inertial pos="%.6f %.6f %.6f" mass="%.6f" fullinertia="%.4e %.4e %.4e %.4e %.4e %.4e"/>' % (
        com[0], com[1], com[2], ds["mass"], I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]))
    for k, (t, p, s) in enumerate(geoms):
        xs.append('<geom name="c%d" type="%s" pos="%.5f %.5f %.5f" size="%s" contype="2" conaffinity="1" rgba="0.9 0.2 0.2 1"/>' % (
            k, t, p[0], p[1], p[2], " ".join("%.5f" % v for v in s)))
    xs.append('</body></worldbody></mujoco>')
    for k, (tt, p, s) in enumerate(geoms):
        if k >= 5 and ds["mu_wall"] > 0:          # rollers: sliding contact with friction (frictionless elsewhere)
            xs[-2 - (len(geoms) - 1 - k)] = xs[-2 - (len(geoms) - 1 - k)].replace('contype="2"', 'contype="2" condim="3" friction="%g 0 0"' % ds["mu_wall"])
    m = mujoco.MjModel.from_xml_string("\n".join(xs))
    return m, ds


# ------------------------------------------------------------------ run one trial
class Run:
    def __init__(self, design=None, ctrl_params=None, dy=0.0, dyaw=0.0, laps=3, timeout=12.0, record=False, model=None, vmap=None):
        self.m, self.ds = model if model is not None else build_model(design)
        if model is not None:
            self.ds = dict(DEFAULT_DESIGN, **(design or {}))
        self.d = mujoco.MjData(self.m)
        self.bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "car")
        C = course()
        # start perturbation (UE: dy cm to the right, dyaw deg)
        if dy or dyaw:
            self.d.qpos[0:3] += -C["fin_l"] * dy * 0.01
            q = self.d.qpos[3:7].copy(); dq = np.zeros(4)
            mujoco.mju_axisAngle2Quat(dq, np.array([0, 0, 1.0]), -math.radians(dyaw))   # UE yaw + = clockwise
            mujoco.mju_mulQuat(self.d.qpos[3:7], dq, q)
        mujoco.mj_forward(self.m, self.d)
        self.vmax_eff = self.ds["vmax"] * self.ds["drive_eff"]
        self.f0 = MOTOR_K * self.ds["motor_k"] * self.ds["drive_eff"] / self.ds["vmax"]
        params = dict(ctrl_params or {})
        params.setdefault("brake", self.ds["brake"])
        if vmap is not None:                     # learned speed map + receding-horizon planning
            self.ctrl = CTRL.MapController(self.vmax_eff, params, vmap)
        else:
            self.ctrl = CTRL.Controller(self.vmax_eff, params)
        self.keylog = []                         # (t, map key) at the controller rate, for crash attribution
        self.ctrl.imu = ImuSensorR(self.ds["imu_range_g"])
        self.laps_n, self.timeout, self.record = laps, timeout, record
        self.mass = self.ds["mass"]
        ip = self.ds.get("imu_pos")
        self.imu_pos = None if ip is None else np.array(ip, float)
        self.Ib = self.m.body_inertia[self.bid].copy()
        self.ray_geom = np.zeros(1, np.int32)
        self.traj = [] if record else None
        self.vw = 0.0              # drive-train surface speed (wheel rim) [m/s]
        self.dvw = 0.0             # rim minus ground speed while on the track (slip after landing)
        self.was_grounded = True
        self.air_t = 0.0

    def _frame(self):
        R = self.d.xmat[self.bid].reshape(3, 3)
        return self.d.xpos[self.bid].copy(), R[:, 0].copy(), R[:, 1].copy(), R[:, 2].copy()

    def run(self):
        m, d, C = self.m, self.d, course()
        t, t_lap, laps, s_prev = 0.0, 0.0, [], None
        off_t, slow_t, first = None, None, True
        vlc, zmax, brake_t, outcome = 0.0, 0.0, 0.0, None
        qf = np.zeros(m.nv)
        while outcome is None:
            loc, f, l, u = self._frame()
            vel = d.qvel[0:3].copy(); w = d.qvel[3:6].copy()            # world linear, body angular
            if self.imu_pos is not None:                                  # IMU mounted away from the body origin
                Rb = d.xmat[self.bid].reshape(3, 3)
                vimu = vel + np.cross(Rb @ w, Rb @ self.imu_pos)
            else:
                vimu = vel
            vf = float(np.dot(vel, f))
            # ---- controller (same call pattern as UE)
            if first:
                self.ctrl.step(0.0, CTRL.body_matrix(tuple(f), tuple(l), tuple(u)), list(vimu)); duty = 0.0; first = False
            else:
                duty, _ = self.ctrl.step(DT, CTRL.body_matrix(tuple(f), tuple(l), tuple(u)), list(vimu))
            # ---- ground contact: ray from 2.5 cm above the origin to 1 cm below, along -up
            start = loc + u * 0.025
            dist = mujoco.mj_ray(m, d, start, -u, None, 1, self.bid, self.ray_geom)
            grounded = 0 <= dist <= 0.035
            qf[:] = 0.0
            d.qfrc_applied[:] = 0.0
            react = self.ds["air_react"]
            Jm = self.ds["drive_J"] / self.ds["tire_r"] ** 2           # mass-equivalent drive-train inertia [kg]
            if react and not grounded:
                # free wheels: the motor accelerates/brakes only the drive train; the body gets the reaction torque
                vw = max(self.vw, 0.0)
                if duty > 0.0:
                    Fm = self.f0 * max(0.0, duty - vw / self.vmax_eff)
                elif duty < 0.0:
                    Fm = -self.f0 * min(1.0, -duty) * vw / self.vmax_eff
                else:
                    Fm = 0.0
                Fm -= Jm * 2.0 * vw                                         # bearing / gear drag
                self.vw = max(0.0, self.vw + Fm / Jm * DT)
                mujoco.mj_applyFT(m, d, np.zeros(3), -l * Fm * self.ds["tire_r"], d.xipos[self.bid], self.bid, d.qfrc_applied)
                self.air_t += DT
            if grounded:
                if react:
                    # landing / wheelspin: rim speed vs ground speed mismatch is removed by tyre friction
                    if not self.was_grounded:
                        self.dvw = self.vw - vf                                # touchdown: rim vs ground speed
                    Fmax = self.ds["mu_tire"] * self.mass * G * (1.0 + self.ds["downforce"])
                    Fs = max(-Fmax, min(Fmax, Jm * self.dvw / 0.003))
                    if abs(Fs) > 1e-9:
                        mujoco.mj_applyFT(m, d, f * Fs, np.zeros(3), d.xipos[self.bid], self.bid, d.qfrc_applied)
                        self.dvw -= Fs * (1.0 / Jm + 1.0 / self.mass) * DT
                        if abs(self.dvw) < 1e-3:
                            self.dvw = 0.0
                    self.vw = vf + self.dvw            # otherwise the wheels roll with the car (calibrated ground model)
                if duty > 0.0:
                    F = self.f0 * max(0.0, duty - max(vf, 0.0) / self.vmax_eff)
                    a = min(F / self.mass, self.ds["mu_tire"] * G * (1.0 + self.ds["downforce"]))
                elif duty < 0.0:
                    F = -self.f0 * min(1.0, -duty) * max(vf, 0.0) / self.vmax_eff
                    a = max(F / self.mass - self.ds["roll_decel"] * vf, -self.ds["mu_tire"] * G * (1.0 + self.ds["downforce"]))
                    brake_t += DT * min(1.0, -duty)
                else:
                    a = -self.ds["roll_decel"] * vf
                mujoco.mj_applyFT(m, d, f * a * self.mass, np.zeros(3), d.xipos[self.bid], self.bid, d.qfrc_applied)
                fmax = self.ds["mu_tire"] * self.mass * G * 0.5 * (1.0 + self.ds["downforce"])
                if self.ds["downforce"] > 0.0:
                    mujoco.mj_applyFT(m, d, -u * self.ds["downforce"] * self.mass * G, np.zeros(3), d.xipos[self.bid], self.bid, d.qfrc_applied)
                wv = d.xmat[self.bid].reshape(3, 3) @ w                   # angular velocity in world
                for ax in (0.04, -0.04):
                    p = loc + f * ax + u * 0.012
                    vp = vel + np.cross(wv, p - d.xpos[self.bid])
                    Fl = -0.5 * self.mass * float(np.dot(vp, l)) / DT_UE * 0.8
                    Fl = max(-fmax, min(fmax, Fl))
                    mujoco.mj_applyFT(m, d, l * Fl, np.zeros(3), p, self.bid, d.qfrc_applied)
            self.was_grounded = grounded
            # angular damping (UE body instance 0.3)
            d.qfrc_applied[3:6] += -self.ds["ang_damp"] * self.Ib * w
            mujoco.mj_step(m, d)
            t += DT
            # ---- bookkeeping
            spd = float(np.linalg.norm(vel))
            zmax = max(zmax, loc[2])
            if self.ctrl.over(self.ctrl.n) and -5.0 < self.ctrl.rem <= 0.0:
                vlc = max(vlc, spd)
            if self.traj is not None:
                self.traj.append((t, *loc, *d.qpos[3:7], duty, spd, self.ctrl.est.v, self.ctrl.est.psi, len(laps),
                                  int(self.ctrl.zone), min(self.ctrl.v_lim, 99.0),
                                  int(grounded), math.degrees(math.asin(max(-1.0, min(1.0, f[2])))), self.vw, int(getattr(self.ctrl, "air", False)),
                                  math.degrees(math.asin(max(-1.0, min(1.0, l[2]))))))
            if getattr(self.ctrl, "key", None) is not None and (not self.keylog or self.keylog[-1][1] != self.ctrl.key):
                self.keylog.append((t, self.ctrl.key))
            dd = loc - C["fin_p"]
            s, lat = float(np.dot(dd, C["fin_f"])), float(np.dot(dd, C["fin_l"]))
            if s_prev is not None and s_prev < 0.0 <= s and abs(lat) < 0.15 and np.dot(vel, C["fin_f"]) > 0:
                if t - t_lap > 0.5:
                    laps.append(t - t_lap)
                t_lap = t
            s_prev = s
            offtrack = loc[2] < 0.0035
            if u[2] < 0.3:
                off_t = off_t if off_t is not None else t
                offtrack = offtrack or (t - off_t > 0.5)
            else:
                off_t = None
            outside = np.any(loc[:2] < C["lo"][:2] - 0.08) or np.any(loc[:2] > C["hi"][:2] + 0.08)
            if spd < 0.05 and t > 1.0:
                slow_t = slow_t if slow_t is not None else t
            else:
                slow_t = None
            if offtrack or outside:
                outcome = "courseout"
            elif len(laps) >= self.laps_n:
                outcome = "ok"
            elif t > self.timeout:
                outcome = "timeout"
            elif slow_t is not None and t - slow_t > 1.0:
                outcome = "stuck"
        total = sum(laps[:self.laps_n]) if outcome == "ok" else None
        t_fail = None
        if outcome in ("courseout", "stuck", "timeout"):
            t_fail = off_t if off_t is not None else t - 0.2
        return dict(outcome=outcome, total=total, laps=laps, t=t, vlc=vlc, zmax=zmax, brake_s=brake_t, air_s=self.air_t,
                    t_fail=t_fail)


def simulate(design=None, ctrl_params=None, **kw):
    return Run(design, ctrl_params, **kw).run()


if __name__ == "__main__":
    import time
    FC = dict(v_over=2.7, v_bridge=2.8, kp_bridge=8.0, a_coast=4.5, a_coast_late=5.0)
    t0 = time.time()
    r = Run(dict(mass=0.09, vmax=8.8), FC, record=True)
    print("model nbody/ngeom", r.m.nbody, r.m.ngeom)
    out = r.run()
    print(out, "wall %.2f s" % (time.time() - t0))
