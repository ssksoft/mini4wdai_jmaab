# -*- coding: utf-8 -*-
"""Python-control simulation core: XIAO MG24 Sense + KEIMYO-001 motor driver on a stock Mini 4WD.

Pure Python (no Unreal dependency): used by the Unreal runtime (ue/m4wd_drive.py, start_py).
Per physics step (= UE frame) the runtime calls

    a = car.step(dt, R, v_world, v_fwd, grounded)      # -> longitudinal acceleration [m/s^2] to apply

and this module
  1) turns the true body motion into a virtual LSM6DS3 reading (specific force [g], angular rate [dps]),
     mapped onto the sensor axes of how the XIAO is mounted, with optional noise / bias,
  2) puts the analog voltages the board would show on PC0 (VBAT_SENSE), PC1 (NTC) and PC3 (MID_SENSE),
  3) calls your Controller.step(io) at CONTROL_HZ up to the same simulation time,
  4) takes the time-averaged duty (io.duty = PC2) and turns it into wheel force with the driver model below.

KEIMYO-001 (Switch Science 11192) as modelled here
  * low-side N-ch drive, P-ch short brake across the motor, complementary from ONE pin (PC2):
        duty 1 = full drive, duty 0 = continuous short brake, in between the off-phase brakes (no free-wheel)
    while io.duty has never been set it is treated like duty 0 (gate pulled low -> brake).
  * forward only.  Battery: Ni-MH 2 cells or Li-ion 1 cell.  TPS61023 boost powers the XIAO (not modelled).
  * averaged (continuous-current) DC motor:  I = (d*Voc - e) / (R_m + d^2 * R_int),  e = ke * v,  F = kF * I
    kF, ke are chosen so that the Ni-MH preset at duty 1 gives exactly the calibrated UE motor
    (F = F0 * (1 - v / VMAX)); the battery internal resistance, the short brake and other batteries follow
    from the circuit.  R_m (motor + wiring + MOSFET) and the battery data are estimates: see the manual.
  * VBAT_SENSE = mean battery terminal voltage, MID_SENSE = mean M- voltage ((1-d)*Voc), both / 3.2 (22k:10k)
  * NTC: 10k pull-up to 3.3 V, NTCG104BH103 (10k, B=4100) to GND; board temperature from a 1st-order model.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

G = 9.80665

# ------------------------------------------------------------------ presets
BATTERIES = {
    # key: (label, open-circuit voltage [V], internal resistance incl. holder/contacts [ohm])
    "nimh2": ("ニッケル水素 2本（満充電・負荷なし約2.75 V）", 2.75, 0.06),
    "nimh2_low": ("ニッケル水素 2本（使い込み・約2.45 V）", 2.45, 0.08),
    "liion1": ("リチウムイオン 1セル（約4.0 V）", 4.00, 0.04),
}
MOTORS = {
    # key: (label, VMAX [m/s] at Ni-MH duty 1, F0 [N] stall force at the tyres with Ni-MH, R_m [ohm])
    # VMAX / F0 = the UE calibration against the real videos (A0 x 0.1259 kg); R_m is an estimate.
    "standard": ("標準モーター（キット付属）", 4.0, 8.0 * 0.1259, 1.30),
    "machdash_pro": ("マッハダッシュモーターPRO", 7.0, 13.0 * 0.1259, 0.50),
}
VBAT_DIV = 3.2          # 22k : 10k
MID_DIV = 3.2
ADC_REF = 3.3
NTC_R0, NTC_B, NTC_PULLUP = 10000.0, 4100.0, 10000.0


class Keimyo001:
    """averaged electrical model of the driver + battery + motor"""

    def __init__(self, battery="nimh2", motor="standard", t_amb=25.0, r_drv=0.01):
        self.bat_label, self.voc, self.r_int = BATTERIES[battery]
        self.mot_label, vmax, f0, r_m = MOTORS[motor]
        voc_ref, rint_ref = BATTERIES["nimh2"][1:]
        self.r_m = r_m + r_drv                       # motor + MOSFET + wiring
        self.ke = voc_ref / vmax                     # back-EMF per ground speed [V/(m/s)]
        self.kf = f0 * (self.r_m + rint_ref) / voc_ref   # wheel force per motor current [N/A]
        self.t_amb = self.temp = t_amb
        self.rth, self.tau = 60.0, 90.0              # board: K/W, s (rough)
        self.i = 0.0
        self.vb = self.voc
        self.d = 0.0

    def step(self, d, v, dt):
        """d: duty 0..1 (time-averaged), v: ground speed of the drive train [m/s] -> wheel force [N]"""
        d = min(1.0, max(0.0, d))
        e = self.ke * v
        i = (d * self.voc - e) / (self.r_m + d * d * self.r_int)
        self.d, self.i = d, i
        self.vb = self.voc - self.r_int * d * i     # mean battery terminal voltage
        p = i * i * 0.006                           # MOSFET + copper loss near the NTC [W] (tiny)
        self.temp += ((self.t_amb + self.rth * p) - self.temp) * min(1.0, dt / self.tau)
        return self.kf * i

    # ---- what the XIAO sees on its pins [V]
    def v_pc0(self):
        return self.vb / VBAT_DIV

    def v_pc3(self):
        return (1.0 - self.d) * self.vb / MID_DIV

    def v_pc1(self):
        tk = self.temp + 273.15
        r = NTC_R0 * math.exp(NTC_B * (1.0 / tk - 1.0 / 298.15))
        return ADC_REF * r / (NTC_PULLUP + r)


# ------------------------------------------------------------------ small 3x3 helpers (lists, columns = body axes)
def _mt(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(A, x):
    return [sum(A[i][k] * x[k] for k in range(3)) for i in range(3)]


def parse_axes(spec):
    """'x,y,z' / '-y,x,z' ...: sensor axis k measures body axis token k (body: x fwd, y left, z up)"""
    out = []
    for tok in spec.replace(" ", "").split(","):
        s = -1.0 if tok.startswith("-") else 1.0
        out.append((s, "xyz".index(tok.lstrip("+-").lower())))
    if len(out) != 3 or len({a for _, a in out}) != 3:
        raise ValueError("imu_axes must name x, y, z once each, e.g. 'x,y,z' or '-y,x,z': " + spec)
    return out


class VirtualLSM6DS3:
    """samples at the output data rate; each sample = mean specific force / angular rate over the period"""

    def __init__(self, odr=416.0, axes="x,y,z", noise=True, seed=0):
        self.odr = odr
        self.axes = parse_axes(axes)
        self.noise = noise
        rnd = random.Random(seed)
        self.rnd = rnd
        self.gbias = [rnd.gauss(0, 0.4) for _ in range(3)] if noise else [0.0] * 3   # residual zero-rate offset [dps]
        self.abias = [rnd.gauss(0, 0.01) for _ in range(3)] if noise else [0.0] * 3  # [g]
        self.R = self.v = None
        self.acc_t = 0.0
        self.body = ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0])   # last sample in body axes (g, dps)
        self.sensor = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    def update(self, dt, R, v):
        if self.R is None:
            self.R, self.v = R, list(v)
            self._map()
            return
        self.acc_t += dt
        if self.acc_t < 1.0 / self.odr - 1e-9:
            return
        h = self.acc_t
        a = [(v[i] - self.v[i]) / h for i in range(3)]
        a[2] += G
        Rm = [[0.5 * (self.R[i][j] + R[i][j]) for j in range(3)] for i in range(3)]
        f = _mv(_mt(Rm), a)
        M = _mm(_mt(self.R), R)
        w = [0.5 * (M[2][1] - M[1][2]) / h, 0.5 * (M[0][2] - M[2][0]) / h, 0.5 * (M[1][0] - M[0][1]) / h]
        self.R, self.v, self.acc_t = R, list(v), 0.0
        acc = [c / G for c in f]
        gyr = [math.degrees(c) for c in w]
        if self.noise:
            acc = [acc[k] + self.abias[k] + self.rnd.gauss(0, 0.0015) for k in range(3)]
            gyr = [gyr[k] + self.gbias[k] + self.rnd.gauss(0, 0.12) for k in range(3)]
        self.body = ([max(-16.0, min(16.0, c)) for c in acc], [max(-2000.0, min(2000.0, c)) for c in gyr])
        self._map()

    def _map(self):
        acc, gyr = self.body
        self.sensor = tuple([s * acc[i] for s, i in self.axes] + [s * gyr[i] for s, i in self.axes])

    def hold(self, sensor6):
        """force a reading (sensor axes) - used for the boot sequence (e.g. upside down)"""
        self.sensor = tuple(sensor6)


# ------------------------------------------------------------------ the car electronics as a whole
class CarModel:
    """driver + battery + motor + IMU + log; the 'firmware' (_advance) is supplied by the subclass"""

    def _init_common(self, battery, motor, mass, mu, roll_decel, imu_axes, imu_odr, noise, seed, t_amb, log_path, serial_path):
        self.drv = Keimyo001(battery, motor, t_amb)
        self.imu = VirtualLSM6DS3(imu_odr, imu_axes, noise, seed)
        self.mass, self.mu, self.roll_decel = mass, mu, roll_decel
        self.t = 0.0                 # simulation time [s]
        self.duty = 0.0
        self.pwm_seen = False
        self.force = 0.0
        self.lines = []              # recent io.print lines (HUD)
        self.log = open(log_path, "w", encoding="utf-8") if log_path else None
        if self.log:
            self.log.write("t_s,duty,v_mps,a_mps2,force_N,i_motor_A,v_bat_V,pc0_V,pc1_V,pc3_V,temp_C,"
                           "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,grounded\n")
        self.serial_log = open(serial_path, "w", encoding="utf-8") if serial_path else None

    def boot(self, sequence):
        """run the controller with the car held still before the physics starts.
        sequence: list of (duration_s, (ax, ay, az, gx, gy, gz) in sensor axes, or None = upright at rest)"""
        for dur, reading in sequence:
            if reading is None:
                acc = [0.0, 0.0, 1.0]
                reading = tuple([s * acc[i] for s, i in self.imu.axes] + [0.0, 0.0, 0.0])
            self.imu.hold(reading)
            n = max(1, int(round(dur / 0.002)))
            for _ in range(n):
                self.duty = self._advance(dur / n)
                self.drv.step(self.duty, 0.0, dur / n)

    def step(self, dt, R, v_world, v_fwd, grounded):
        """R: 3x3 list, columns = body forward / left / up in a right-handed, z-up world; v_world [m/s].
        returns the longitudinal acceleration [m/s^2] the simulator should apply along the car's forward axis"""
        self.imu.update(dt, R, v_world)
        self.duty = self._advance(dt)
        F = self.drv.step(self.duty, max(v_fwd, 0.0), dt)
        self.force = F
        amax = self.mu * G
        if not grounded:
            a = 0.0
        elif F > 0.0:
            a = min(F / self.mass, amax)
        else:
            a = max(F / self.mass - self.roll_decel * v_fwd, -amax)
        if self.log:
            s, d = self.imu.sensor, self.drv
            self.log.write("%.5f,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d\n" % (
                self.t, self.duty, v_fwd, a, F, d.i, d.vb, d.v_pc0(), d.v_pc1(), d.v_pc3(), d.temp, *s, int(grounded)))
        return a


# ================================================================== Python control
class ControllerIO:
    """what a Python controller sees and sets - the same signals the XIAO MG24 has on the KEIMYO-001 board.
    inputs : t [s], dt [s] (since the previous call), acc (ax, ay, az) [g], gyro (gx, gy, gz) [dps] in SENSOR axes,
             vbat() / temp_c() / v_mid() decoded like the firmware would, adc_volts("PC0"...) raw pin voltage
    output : duty (0..1, 1 = drive, 0 = short brake; None until set = brake), print(...) -> HUD / serial.txt"""

    def __init__(self, car):
        self._car = car
        self.t = 0.0
        self.dt = 0.0
        self.acc = (0.0, 0.0, 1.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.duty = None
        self.state = {}              # free scratch space shared with the HUD (shown as key=value)

    def adc_volts(self, pin):
        d = self._car.drv
        return {"PC0": d.v_pc0, "PC1": d.v_pc1, "PC3": d.v_pc3}[pin.upper()]()

    def vbat(self):
        return self.adc_volts("PC0") * VBAT_DIV

    def v_mid(self):
        return self.adc_volts("PC3") * MID_DIV

    def temp_c(self):
        v = self.adc_volts("PC1")
        r = NTC_PULLUP * v / max(1e-6, ADC_REF - v)
        return 1.0 / (math.log(r / NTC_R0) / NTC_B + 1.0 / 298.15) - 273.15

    def print(self, *args):
        self._car._emit(" ".join(str(a) for a in args))


def load_controller(path, params=None):
    """import the controller file fresh (edits are picked up on every run) and build Controller(**params)"""
    import importlib.util
    import itertools
    path = os.path.abspath(path)
    if not hasattr(load_controller, "_n"):
        load_controller._n = itertools.count()
    name = "m4wd_ctrl_%s_%d" % (os.path.splitext(os.path.basename(path))[0], next(load_controller._n))
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, "Controller"):
        raise AttributeError("%s に class Controller がありません" % path)
    return mod.Controller(**(params or {}))


class ControllerError(RuntimeError):
    pass


class PyCar(CarModel):
    """the car whose 'firmware' is a Python object:
        class Controller:
            CONTROL_HZ = 1000              # how often step() is called (simulation time)
            def setup(self, io): ...       # once at power-on
            def step(self, io): ...        # every 1/CONTROL_HZ s: read io.acc / io.gyro / io.vbat(), set io.duty"""

    def __init__(self, controller, battery="nimh2", motor="standard", mass=0.135, mu=1.0, roll_decel=0.25,
                 imu_axes="x,y,z", imu_odr=416.0, noise=True, seed=0, t_amb=25.0, log_path=None, serial_path=None,
                 params=None):
        self.ctrl = load_controller(controller, params) if isinstance(controller, str) else controller
        self.ctrl_path = controller if isinstance(controller, str) else type(controller).__name__
        self._init_common(battery, motor, mass, mu, roll_decel, imu_axes, imu_odr, noise, seed, t_amb, log_path, serial_path)
        self.hz = float(getattr(self.ctrl, "CONTROL_HZ", 1000.0))
        self.io = ControllerIO(self)
        self.t_next = 0.0
        self.t_prev_call = 0.0
        self._call("setup")

    def _emit(self, line):
        if self.serial_log:
            self.serial_log.write("%.4f %s\n" % (self.t, line))
        self.lines.append(line)
        self.lines = self.lines[-6:]

    def _call(self, fn):
        f = getattr(self.ctrl, fn, None)
        if f is None:
            return
        try:
            f(self.io)
        except Exception as e:
            import traceback
            tb = traceback.extract_tb(e.__traceback__)[-1]
            raise ControllerError("%s() で例外: %s: %s  (%s 行 %d)" % (fn, type(e).__name__, e, os.path.basename(tb.filename), tb.lineno))

    def _advance(self, dt):
        """call step() at CONTROL_HZ inside [t, t+dt]; returns the time-averaged duty"""
        io = self.io
        s = self.imu.sensor
        io.acc, io.gyro = s[:3], s[3:]
        t0, t1 = self.t, self.t + dt
        acc_d, tc = 0.0, t0
        while self.t_next <= t1 + 1e-12:
            cur = 0.0 if io.duty is None else min(1.0, max(0.0, float(io.duty)))
            acc_d += cur * (self.t_next - tc)
            tc = self.t_next
            io.t = self.t_next
            io.dt = self.t_next - self.t_prev_call
            self.t_prev_call = self.t_next
            self._call("step")
            self.t_next += 1.0 / self.hz
        if io.duty is not None:
            self.pwm_seen = True
        cur = 0.0 if io.duty is None else min(1.0, max(0.0, float(io.duty)))
        acc_d += cur * (t1 - tc)
        self.t = t1
        return acc_d / dt if dt > 0 else cur

    def close(self):
        f = getattr(self.ctrl, "close", None)
        if f:
            try:
                f()
            except Exception:
                pass
        for fh in (self.log, self.serial_log):
            if fh:
                fh.close()
        self.log = self.serial_log = None
