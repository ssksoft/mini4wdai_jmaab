# -*- coding: utf-8 -*-
"""On-board speed controller of the Mini 4WD, driven ONLY by the 6-axis IMU (no unreal import: testable offline).

  ImuSensor   : virtual LSM6DS3 - body-frame specific force [g] and angular rate [dps] from the car motion
  Estimator   : strapdown attitude (gyro integration), heading psi, forward speed v_hat (accel integration)
  Controller  : PWM duty from heading-based course phase: 0..1 drive; with brake=1, -1..0 = short-brake duty

Body axes: x forward, y left, z up (right-handed).  World: right-handed, z up.  Clockwise running -> psi decreases.

Course phase from the heading only:
  start (far straight, heading 0) -> 4 right curves (psi 0 -> -180) -> LANE CHANGE (S-bend, net 0)
  -> 4 right curves (-180 -> -360) -> 2 straights -> next lap.
  The n-th lane-change crossing happens around psi = -180 - 360 n.
  The car alternates lanes every lap, so crossings 0, 2, 4 ... go OVER the bridge (inner-lane start),
  crossings 1, 3 ... go UNDER it.
"""
import math

G = 9.80665


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_t(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _mat_vec(A, v):
    return [A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3)]


def _orthonormalize(R):
    # columns = body axes in world
    x = [R[0][0], R[1][0], R[2][0]]
    z = [R[0][2], R[1][2], R[2][2]]
    n = math.sqrt(sum(c * c for c in x)); x = [c / n for c in x]
    d = sum(a * b for a, b in zip(x, z)); z = [a - d * b for a, b in zip(z, x)]
    n = math.sqrt(sum(c * c for c in z)); z = [c / n for c in z]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[i], y[i], z[i]] for i in range(3)]


def body_matrix(f, l, u):
    """3x3 matrix whose columns are the body axes (forward, left, up) in world coordinates"""
    return [[f[i], l[i], u[i]] for i in range(3)]


class ImuSensor:
    """6-axis IMU sampled at RATE_HZ like the real logger (LSM6DS3, 200 Hz).
    Each sample = mean specific force / mean angular rate over the sample period (sensor low-pass),
    range +-16 g / +-2000 dps, quantised to 0.001 g / 0.1 dps."""
    RATE_HZ = 200.0

    def __init__(self):
        self.R = None
        self.v = None
        self.acc_t = 0.0

    def update(self, dt, R, v_world):
        """feed the true body motion every physics tick; returns (dt_sample, acc_g, gyro_dps) or None"""
        if self.R is None:
            self.R, self.v, self.acc_t = R, v_world, 0.0
            return None
        self.acc_t += dt
        if self.acc_t < 1.0 / self.RATE_HZ - 1e-6:
            return None
        h = self.acc_t
        a = [(v_world[i] - self.v[i]) / h for i in range(3)]
        a[2] += G
        Rm = _orthonormalize([[0.5 * (self.R[i][j] + R[i][j]) for j in range(3)] for i in range(3)])
        f = _mat_vec(_mat_t(Rm), a)
        M = _mat_mul(_mat_t(self.R), R)
        w = [0.5 * (M[2][1] - M[1][2]) / h, 0.5 * (M[0][2] - M[2][0]) / h, 0.5 * (M[1][0] - M[0][1]) / h]
        self.R, self.v, self.acc_t = R, v_world, 0.0
        acc_g = [max(-16.0, min(16.0, round(c / G, 3))) for c in f]
        gyro = [max(-2000.0, min(2000.0, round(math.degrees(c), 1))) for c in w]
        return h, acc_g, gyro


class Estimator:
    """strapdown navigation from IMU samples only"""

    def __init__(self):
        self.R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]   # start: level, heading 0
        self.vw = [0.0, 0.0, 0.0]   # velocity estimate in the navigation frame [m/s]
        self.v = 0.0            # speed estimate [m/s]
        self.psi = 0.0          # unwrapped heading [deg]
        self._yaw_prev = 0.0
        self.pitch = 0.0        # [deg], nose up positive

    def update(self, dt, acc_g, gyro_dps):
        w = [math.radians(c) * dt for c in gyro_dps]
        dR = [[1.0, -w[2], w[1]], [w[2], 1.0, -w[0]], [-w[1], w[0], 1.0]]
        self.R = _orthonormalize(_mat_mul(self.R, dR))
        yaw = math.degrees(math.atan2(self.R[1][0], self.R[0][0]))
        d = (yaw - self._yaw_prev + 180.0) % 360.0 - 180.0
        self.psi += d
        self._yaw_prev = yaw
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.R[2][0]))))
        a_nav = _mat_vec(self.R, [c * G for c in acc_g])  # specific force -> navigation frame
        a_nav[2] -= G                                      # + gravity
        self.vw = [self.vw[i] + a_nav[i] * dt for i in range(3)]
        self.v = math.sqrt(sum(c * c for c in self.vw))


class EstimatorFwd(Estimator):
    """same attitude / heading as Estimator, but the speed uses the non-holonomic constraint of a car guided by
    walls: it only moves along its own forward axis.  v' = f_x - g * sin(pitch)  (forward specific force + gravity)
    -> lateral wall impacts and attitude errors no longer leak into the speed estimate."""

    def update(self, dt, acc_g, gyro_dps):
        w = [math.radians(c) * dt for c in gyro_dps]
        dR = [[1.0, -w[2], w[1]], [w[2], 1.0, -w[0]], [-w[1], w[0], 1.0]]
        R_prev = self.R
        self.R = _orthonormalize(_mat_mul(self.R, dR))
        yaw = math.degrees(math.atan2(self.R[1][0], self.R[0][0]))
        d = (yaw - self._yaw_prev + 180.0) % 360.0 - 180.0
        self.psi += d
        self._yaw_prev = yaw
        sin_p = 0.5 * (R_prev[2][0] + self.R[2][0])          # forward axis z (mid-interval)
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.R[2][0]))))
        self.v = max(0.0, self.v + (acc_g[0] * G - G * sin_p) * dt)
        self.vw = [self.R[i][0] * self.v for i in range(3)]


class EstimatorMid(Estimator):
    """navigation-frame integration with the specific force rotated by the MID-interval attitude
    (the IMU sample is a mean over the interval); fixes the lag that loses speed in fast turns"""

    def update(self, dt, acc_g, gyro_dps):
        w = [math.radians(c) * dt for c in gyro_dps]
        dR = [[1.0, -w[2], w[1]], [w[2], 1.0, -w[0]], [-w[1], w[0], 1.0]]
        R_prev = self.R
        self.R = _orthonormalize(_mat_mul(self.R, dR))
        Rm = _orthonormalize([[0.5 * (R_prev[i][j] + self.R[i][j]) for j in range(3)] for i in range(3)])
        yaw = math.degrees(math.atan2(self.R[1][0], self.R[0][0]))
        d = (yaw - self._yaw_prev + 180.0) % 360.0 - 180.0
        self.psi += d
        self._yaw_prev = yaw
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.R[2][0]))))
        a_nav = _mat_vec(Rm, [c * G for c in acc_g])
        a_nav[2] -= G
        self.vw = [self.vw[i] + a_nav[i] * dt for i in range(3)]
        self.v = math.sqrt(sum(c * c for c in self.vw))


class EstimatorLevel(EstimatorMid):
    """EstimatorMid + map knowledge: the track is flat except on the ramps / bridge.  While the car is on the
    flat (body-z specific force ~1 g, small roll / pitch rates) for LEVEL_T seconds, the attitude is re-levelled
    (roll = pitch = 0, heading kept) and the vertical velocity is zeroed.  This stops attitude errors from
    leaking gravity into the speed estimate."""
    LEVEL_T = 0.03

    def __init__(self):
        EstimatorMid.__init__(self)
        self.flat_t = 0.0

    def update(self, dt, acc_g, gyro_dps):
        EstimatorMid.update(self, dt, acc_g, gyro_dps)
        flat = abs(acc_g[2] - 1.0) < 0.15 and abs(gyro_dps[0]) < 30.0 and abs(gyro_dps[1]) < 30.0
        self.flat_t = self.flat_t + dt if flat else 0.0
        if self.flat_t >= self.LEVEL_T:
            c, s_ = self.R[0][0], self.R[1][0]
            n = math.hypot(c, s_) or 1.0
            c, s_ = c / n, s_ / n
            self.R = [[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]
            self.vw[2] = 0.0
            self.v = math.sqrt(sum(x * x for x in self.vw))
            self.pitch = 0.0


class EstimatorAlign(EstimatorLevel):
    """EstimatorLevel + non-holonomic re-alignment: while the tyres are on the track the car moves along its own
    forward axis, so the velocity vector is turned onto that axis KEEPING ITS MAGNITUDE.  Without this, a few
    degrees between the integrated velocity and the gyro heading makes the ~4 g centripetal acceleration of the
    fast curves leak into the speed (the 0.3-0.6 m/s errors seen before the 3rd-lap bridge)."""
    GND_G = 0.6

    def update(self, dt, acc_g, gyro_dps):
        EstimatorLevel.update(self, dt, acc_g, gyro_dps)
        if acc_g[2] > self.GND_G:
            fx = [self.R[i][0] for i in range(3)]
            sgn = 1.0 if sum(a * b for a, b in zip(self.vw, fx)) >= 0.0 else -1.0
            self.vw = [c * self.v * sgn for c in fx]


class EstimatorMap(EstimatorMid):
    """Speed estimator used with the short brake (est=2).  Map knowledge: the track is flat except the ramps / bridge.
      flat & tyres on the track (body-z ~1 g, small roll/pitch rates, LEVEL_T s): attitude re-levelled, and the
          speed is integrated as a SCALAR along the forward axis (v' = f_x).  Centripetal acceleration in the
          curves (~4 g) is perpendicular to it, so it can neither leak in through a heading error nor inflate the
          speed through the Euler step.
      ramps, bridge, jumps: full navigation-frame integration (mid-interval attitude)."""
    LEVEL_T = 0.03

    def __init__(self):
        EstimatorMid.__init__(self)
        self.flat_t = 0.0
        self.flat = False

    def update(self, dt, acc_g, gyro_dps):
        flat = abs(acc_g[2] - 1.0) < 0.15 and abs(gyro_dps[0]) < 30.0 and abs(gyro_dps[1]) < 30.0
        self.flat_t = self.flat_t + dt if flat else 0.0
        self.flat = self.flat_t >= self.LEVEL_T
        if not self.flat:
            EstimatorMid.update(self, dt, acc_g, gyro_dps)
            return
        w = math.radians(gyro_dps[2]) * dt                  # flat: heading only
        yaw = math.atan2(self.R[1][0], self.R[0][0]) + w
        c, s_ = math.cos(yaw), math.sin(yaw)
        self.R = [[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]
        yd = math.degrees(yaw)
        d = (yd - self._yaw_prev + 180.0) % 360.0 - 180.0
        self.psi += d
        self._yaw_prev = yd
        self.pitch = 0.0
        self.v = max(0.0, self.v + acc_g[0] * G * dt)
        self.vw = [c * self.v, s_ * self.v, 0.0]


class EstimatorHybrid(EstimatorMid):
    """navigation-frame integration (handles jumps and landings), plus the non-holonomic constraint while the
    tyres are pressed onto the track (body-z specific force > GND_G): the velocity vector is projected onto
    the forward axis, which removes the drift that wall impacts and attitude errors put into the other axes."""
    GND_G = 0.6

    def update(self, dt, acc_g, gyro_dps):
        EstimatorMid.update(self, dt, acc_g, gyro_dps)
        if acc_g[2] > self.GND_G:
            fx = [self.R[i][0] for i in range(3)]
            vf = sum(a * b for a, b in zip(self.vw, fx))
            self.vw = [c * vf for c in fx]
            self.v = abs(vf)


DEFAULT_CTRL = dict(
    v_over=2.85,       # (legacy) target speed at the foot of the up-ramp
    v_bridge=3.0,      # "virtual motor" top speed used from emu_from deg before the lane change to the bridge exit [m/s]
    kp_bridge=7.6,     # gain of that law: F = F0/vmax*(1+kp)*(v_bridge - v) = same force curve as a 3.0 m/s-geared motor
    emu_from=-1.0,     # heading before the lane change where the virtual-motor law starts; <0: only on the bridge
    a_coast=4.0,       # deceleration assumed for planning the coasting point [m/s^2] (1st crossing)
    a_coast_late=-1.0, # same for the later over crossings (car arrives much faster); <0: = a_coast
    r_eff=0.37,        # effective curve radius (heading -> distance) [m]
    l_str=0.93,        # length of the two-straight section [m]
    phi_post=35.0,     # deg after the lane-change heading (into the next curve) where full throttle resumes
    v_under=99.0,      # speed limit when crossing UNDER the bridge (99 = none)
    kp=4.0,            # proportional gain of the speed loop elsewhere
    first_over=1,      # 1: the first crossing goes over the bridge (inner-lane start)
    brake=0,           # 1: short brake available (motor terminals shorted by PWM): command may go negative
    kb=2.5,            # brake gain [1/(m/s)]: brake duty = kb * (over-speed - dead band), clipped to 1
    brake_bridge=1,    # 1: the short brake may also be used on the ramp / bridge
    brake_db=0.05,     # dead band [m/s]: brake only when the estimate exceeds the target by more than this
    brake_n=99,        # short brake only for lane-change crossings with index <= brake_n (0 = 1st lap only)
    brake_str=0,       # 1: for later crossings the short brake is also allowed on the straights before the last curves
                       #    (speed estimate still accurate there; not in the fast curves where wall hits clip the IMU)
    v_over_first=-1.0, # target speed at the foot of the up-ramp for the FIRST crossing; <0: = v_over
    air_ctrl=0,        # 1: in the air (IMU ~free fall) steer the pitch with the wheels: spin-up -> nose up, brake -> nose down
    air_pitch=0.0,     # target pitch in the air [deg, nose up +]
    air_kp=0.05,       # [1/deg]   u = kp*(target - pitch) - kd*pitch_rate ; u>0 spin up, u<0 brake
    air_kd=0.002,      # [1/(deg/s)]
    air_g=0.35,        # |specific force| below this [g] = airborne
    v_cap=99.0,        # global speed cap [m/s] everywhere (curve / roller limit for strong motors; 99 = none)
    est=0,             # speed estimator: 0 nav-frame (original), 1 mid-interval attitude, 2 map-aware (flat: forward scalar)
)


class Controller:
    def __init__(self, vmax, params=None):
        self.p = dict(DEFAULT_CTRL)
        if params:
            self.p.update({k: v for k, v in params.items() if k in DEFAULT_CTRL})
        self.vmax = vmax
        self.imu = ImuSensor()
        self.est = [Estimator, EstimatorMid, EstimatorMap][int(self.p["est"])]()
        self.n = 0              # index of the next lane-change crossing
        self.zone = False
        self.duty = 1.0
        self.max_pitch = 0.0
        self.rem = 180.0
        self.v_lim = 99.0
        self.on_bridge = False
        self.on_str = False
        self.air = False
        self.s_str = 0.5 * self.p["l_str"]     # the start line is in the middle of the straight section

    def over(self, n):
        return (n % 2 == 0) == bool(self.p["first_over"])

    def speed_duty(self, v_target, kp=None):
        if v_target >= 90.0:
            return 1.0
        kp = self.p["kp"] if kp is None else kp
        e = v_target - self.est.v
        allowed = self.n <= self.p["brake_n"] or (self.p["brake_str"] and self.on_str)
        if self.p["brake"] and allowed and e < -self.p["brake_db"] and (self.p["brake_bridge"] or not self.zone):
            return max(-1.0, self.p["kb"] * (e + self.p["brake_db"]))   # short brake ∝ over-speed
        d = v_target / self.vmax + kp * e / self.vmax
        return max(0.0, min(1.0, d))

    def distance_to_go(self, rem, h):
        """distance [m] to the foot of the next up-ramp, from the heading still to turn (+ odometry on the straight)"""
        p, d = self.p, 5.0
        self.on_str = 180.0 - d <= rem <= 180.0 + d
        if rem > 180.0 + d:                              # curves after the previous lane change
            self.s_str = 0.0
            return p["r_eff"] * math.radians(rem - 180.0) + p["l_str"] + p["r_eff"] * math.pi
        if rem >= 180.0 - d:                             # on the straights: heading constant -> integrate speed
            self.s_str += self.est.v * h
            return max(0.0, p["l_str"] - self.s_str) + p["r_eff"] * math.radians(max(0.0, rem))
        return p["r_eff"] * math.radians(max(0.0, rem))   # last four curves

    def step(self, dt, R, v_world):
        """call every physics tick; the estimator and the duty update at the IMU rate (zero-order hold between)"""
        sample = self.imu.update(dt, R, v_world)
        if sample is None:
            return self.duty, None
        h, acc, gyro = sample
        self.est.update(h, acc, gyro)
        p = self.p
        self.air = bool(p["air_ctrl"]) and math.sqrt(sum(c * c for c in acc)) < p["air_g"]
        if self.air:                                            # airborne: drive train as a reaction wheel (pitch only)
            q = -gyro[1]                                        # nose-up rate [deg/s] (body y = left)
            u = p["air_kp"] * (p["air_pitch"] - self.est.pitch) - p["air_kd"] * q
            if u >= 0.0:
                vr = min(1.0, self.est.v / self.vmax)
                self.duty = min(1.0, vr + u * (1.0 - vr))
            else:
                self.duty = max(-1.0, u) if p["brake"] else 0.0
            return self.duty, (acc, gyro)
        rem = self.est.psi - (-180.0 - 360.0 * self.n)        # heading still to turn before the n-th lane change
        if rem < -p["phi_post"]:                              # lane change passed -> next one
            self.n += 1
            rem = self.est.psi - (-180.0 - 360.0 * self.n)
            self.on_bridge = False
        over = self.over(self.n)
        self.rem = rem
        self.on_str = False
        if over and rem <= 0.0:
            self.on_bridge = True
        kp = None
        emu = p["emu_from"] >= 0.0
        if over and (self.on_bridge or (emu and rem <= p["emu_from"])):
            # virtual low-geared motor: same force-speed curve as the robust 3.0 m/s gearing
            v_lim, kp = p["v_bridge"], p["kp_bridge"]
        elif over:                                            # braking-point planning (coast only, no brake)
            s_go = self.distance_to_go(rem, h)
            ac = p["a_coast"] if (self.n == 0 or p["a_coast_late"] < 0) else p["a_coast_late"]
            if emu:                                           # reach v_bridge where the virtual motor takes over
                v_lim = math.sqrt(p["v_bridge"] ** 2 + 2.0 * ac * max(0.0, s_go - p["r_eff"] * math.radians(p["emu_from"])))
            else:
                vo = p["v_over_first"] if (self.n == 0 and p["v_over_first"] > 0.0) else p["v_over"]
                v_lim = math.sqrt(vo ** 2 + 2.0 * ac * s_go)
        else:
            v_lim = p["v_under"] if rem < 60.0 else 99.0
        v_lim = min(v_lim, p["v_cap"])
        self.zone = over and (self.on_bridge or (emu and rem <= p["emu_from"]))
        self.v_lim = v_lim
        self.duty = self.speed_duty(v_lim, kp)
        return self.duty, (acc, gyro)


# =====================================================================================================================
# Learned speed map + receding-horizon speed planning ("learning run" -> course-out speed map -> MPC)
# =====================================================================================================================
MAP_CTRL = dict(
    bin=0.10,          # map resolution along the course coordinate [m]
    v0=3.0,            # speed for bins without a map entry [m/s]
    a_brk=18.0,        # deceleration the planner may count on [m/s^2] (short brake + grip)
    horizon=1.5,       # look-ahead [m]
    l_post0=0.4,       # course coordinate length after the foot of the up-ramp until the next crossing index [m], OVER crossings
    l_post1=1.0,       # same, UNDER crossings
    post_deg=0.0,      # heading still to turn [deg] below which the coordinate switches to odometry (ramp / bridge)
    margin=0.0,        # subtracted from every map speed [m/s] (race safety margin)
    margin_rel=0.0,    # relative margin (map speed x (1 - margin_rel))
)


class MapController(Controller):
    """Speed limit map on an IMU-only course coordinate + receding-horizon speed planning.

    Course coordinate u [m] = distance still to go to the foot of the next up-ramp (heading + straight odometry,
    same as distance_to_go), negative after the foot (odometry over the ramp / bridge / S-bend) until the
    crossing index switches.  Map key = (crossing parity = lane, bin of u).

    Planning (closed-form solution of the 1-D time-optimal MPC with speed-limit constraints and a deceleration
    bound): the commanded speed is the largest one from which every map limit inside the horizon can still be met,
        v_cmd = min_b sqrt(V_b^2 + 2 a_brk d_b)       (d_b = distance to bin b, V_b its limit)
    and the inner loop tracks it with full throttle / short brake.  Logs per-bin IMU indicators for learning."""

    def __init__(self, vmax, params=None, vmap=None):
        params = dict(params or {})
        Controller.__init__(self, vmax, params)
        self.mp = dict(MAP_CTRL)
        self.mp.update({k: v for k, v in params.items() if k in MAP_CTRL})
        self.vmap = dict(vmap or {})
        self.post = False
        self.s_post = 0.0
        self.u_post0 = 0.0
        self.u = 3.0
        self.key = None
        self.log = {}           # key -> [n, v_max, roll_max, az_min, air_n, v_cmd_min]
        self.l_post_seen = []
        self.u_top = self.p["r_eff"] * math.pi * 2.0 + self.p["l_str"]
        self.v_prev = None
        self.brk_t = 0.0
        self.decel_seen = []    # measured deceleration during full short-brake on the flat [m/s^2] (learned a_brk)

    def lpost(self):
        return self.mp["l_post0"] if self.over(self.n) else self.mp["l_post1"]

    def V(self, par, ib):
        v = self.vmap.get("%d|%d" % (par, ib), self.mp["v0"])
        return max(0.5, v * (1.0 - self.mp["margin_rel"]) - self.mp["margin"])

    def coord(self, h):
        p = self.p
        rem = self.est.psi - (-180.0 - 360.0 * self.n)
        if rem < -p["phi_post"]:
            self.l_post_seen.append(self.s_post)
            self.n += 1
            self.post, self.s_post = False, 0.0
            rem = self.est.psi - (-180.0 - 360.0 * self.n)
        self.rem = rem
        if rem <= self.mp["post_deg"] and not self.post:
            # end of the last curve: the ramp / bridge / S-bend barely change the heading -> odometry from here
            self.post = True
            self.u_post0 = p["r_eff"] * math.radians(max(0.0, rem))
        if self.post:
            self.s_post += self.est.v * h
            return self.u_post0 - self.s_post
        return self.distance_to_go(rem, h)

    def plan(self):
        b, H, a = self.mp["bin"], self.mp["horizon"], self.mp["a_brk"]
        par = self.n % 2
        u = self.u
        ib0 = int(math.floor(u / b))
        v = self.V(par, ib0)
        ib = ib0 - 1
        while True:                                     # bins ahead in this crossing's coordinate (u decreasing)
            d = u - (ib + 1) * b
            if d > H or (ib + 1) * b < -self.lpost() - 0.5:
                break
            v = min(v, math.sqrt(self.V(par, ib) ** 2 + 2.0 * a * max(0.0, d)))
            ib -= 1
        d0 = max(0.0, self.lpost() - self.s_post) if self.post else (u + self.lpost())   # to the next crossing index
        if d0 < H:                                      # continue into the next crossing's coordinate
            par2 = (self.n + 1) % 2
            ib = int(math.floor(self.u_top / b))
            while True:
                d = d0 + (self.u_top - (ib + 1) * b)
                if d > H or ib < 0:
                    break
                v = min(v, math.sqrt(self.V(par2, ib) ** 2 + 2.0 * a * max(0.0, d)))
                ib -= 1
        return v

    def step(self, dt, R, v_world):
        sample = self.imu.update(dt, R, v_world)
        if sample is None:
            return self.duty, None
        h, acc, gyro = sample
        self.est.update(h, acc, gyro)
        self.u = self.coord(h)
        par, ib = self.n % 2, int(math.floor(self.u / self.mp["bin"]))
        self.key = "%d|%d" % (par, ib)
        v_cmd = self.plan()
        self.zone = False
        self.v_lim = v_cmd
        self.duty = self.speed_duty(v_cmd)
        # braking capability: forward specific force while the short brake is fully on and the tyres are loaded
        if self.duty <= -0.9 and acc[2] > 0.6:
            self.brk_t += h
            if self.brk_t > 0.01:
                self.decel_seen.append(-acc[0] * G)
        else:
            self.brk_t = 0.0
        # per-bin IMU indicators (what a real car can log)
        roll = math.degrees(math.asin(max(-1.0, min(1.0, self.est.R[2][1]))))
        anorm = math.sqrt(sum(c * c for c in acc))
        L = self.log.setdefault(self.key, [0, 0.0, 0.0, 9.0, 0, 99.0])
        L[0] += 1; L[1] = max(L[1], self.est.v); L[2] = max(L[2], abs(roll)); L[3] = min(L[3], acc[2])
        L[4] += int(anorm < 0.35); L[5] = min(L[5], v_cmd)
        return self.duty, (acc, gyro)
