# -*- coding: utf-8 -*-
"""lc_brake.py - sample controller: brake before the lane-change ramp (XIAO MG24 Sense + KEIMYO-001)

1) brake-hold for START_WAIT s while measuring the gyro offset
2) run at RUN_DUTY
3) nose-up kick (gyro Y) at the foot of the lane-change ramp -> brake BRAKE_S
4) remember the heading of the ramp (gyro Z integrated); the ramp comes again every 720 deg,
   so from the 2nd time on brake EARLY_S already LEAD_DEG before it

io (see MiniAI_PySim/sil/sil_core.py, ControllerIO):
  io.t, io.dt          time [s] / time since the previous step() [s]
  io.acc, io.gyro      (x, y, z) in SENSOR axes [g] / [dps]  (default mounting: x fwd, y left, z up)
  io.vbat(), io.temp_c(), io.v_mid(), io.adc_volts("PC0")
  io.duty = 0..1       1 = full drive, 0 = full short brake (KEIMYO-001: no free-wheel, no reverse)
  io.print(...)        Serial-like output (HUD + serial.txt)
  io.state[...]        values shown on the HUD
Every value in __init__ can be overridden from outside:  start_py("lc_brake", params={"BRAKE_S": 0.15})
"""


class Controller:
    CONTROL_HZ = 1000                     # step() rate [Hz]

    def __init__(self, START_WAIT=2.0, RUN_DUTY=1.0, BRAKE_DUTY=0.0, BRAKE_S=0.12, CLIMB_DPS=120.0,
                 REARM_S=0.7, LEAD_DEG=30.0, EARLY_S=0.12, WINDOW_DEG=120.0, PRINT_S=0.1):
        # ======== tuning (edit here, or pass params=) ========
        self.START_WAIT = START_WAIT      # brake-hold before the start [s]
        self.RUN_DUTY = RUN_DUTY          # cruising duty 0..1
        self.BRAKE_DUTY = BRAKE_DUTY      # duty in a brake window (0 = full short brake)
        self.BRAKE_S = BRAKE_S            # brake length from the foot of the ramp [s]
        self.CLIMB_DPS = CLIMB_DPS        # nose-up pitch rate = ramp foot [dps]
        self.REARM_S = REARM_S            # ignore other bumps of the same ramp [s]
        self.LEAD_DEG = LEAD_DEG          # anticipation: heading before the ramp [deg]
        self.EARLY_S = EARLY_S            # length of the anticipated brake [s]
        self.WINDOW_DEG = WINDOW_DEG      # once the ramp is known, accept a kick only within this heading of it [deg]
        self.PRINT_S = PRINT_S            # log period [s]

    # ---------------------------------------------------------------- once at power-on
    def setup(self, io):
        io.duty = 0.0                     # brake while booting
        self.mode = "WAIT"
        self.t_mode = io.t
        self.gy_sum = self.gz_sum = 0.0
        self.n = 0
        self.gy_bias = self.gz_bias = 0.0
        self.psi = 0.0                    # heading [deg], left turn = +
        self.psi_ramp = None              # heading where the ramp was last met
        self.t_print = 0.0
        io.print("t,mode,duty,heading_deg,vbat_V,temp_C")

    def _set(self, io, mode):
        self.mode, self.t_mode = mode, io.t

    # ---------------------------------------------------------------- every 1/CONTROL_HZ s
    def step(self, io):
        gx, gy, gz = io.gyro
        el = io.t - self.t_mode
        pitch_rate = -(gy - self.gy_bias)          # nose up = negative gyro Y (x fwd, y left, z up)

        if self.mode == "WAIT":                    # car held: average the gyro offset
            self.gy_sum += gy; self.gz_sum += gz; self.n += 1
            io.duty = 0.0
            if el > self.START_WAIT:
                self.gy_bias, self.gz_bias = self.gy_sum / self.n, self.gz_sum / self.n
                self.psi = 0.0
                self._set(io, "RUN")
        elif self.mode == "RUN":
            io.duty = self.RUN_DUTY
            to_go = None
            if self.psi_ramp is not None:
                d = -1.0 if self.psi_ramp < 0 else 1.0          # clockwise course: heading goes negative
                to_go = d * (self.psi_ramp + d * 720.0 - self.psi)   # heading left until the ramp comes again
            # a kick only counts as the ramp where the ramp can be (bumps elsewhere are ignored)
            if pitch_rate > self.CLIMB_DPS and (to_go is None or abs(to_go) < self.WINDOW_DEG):
                self.psi_ramp = self.psi
                self._set(io, "BRAKE")
            elif to_go is not None and -90.0 < to_go < self.LEAD_DEG:
                self._set(io, "EARLY")
        elif self.mode == "EARLY":                 # anticipated brake before the ramp
            io.duty = self.BRAKE_DUTY
            if pitch_rate > self.CLIMB_DPS:
                self.psi_ramp = self.psi
                self._set(io, "BRAKE")
            elif el > self.EARLY_S:
                self._set(io, "RUN")
        elif self.mode == "BRAKE":
            io.duty = self.BRAKE_DUTY
            if el > self.BRAKE_S:
                self._set(io, "AFTER")
        elif self.mode == "AFTER":                 # full power, no new brake on the same ramp
            io.duty = self.RUN_DUTY
            if el > self.REARM_S:
                self._set(io, "RUN")

        if self.mode != "WAIT":
            self.psi += (gz - self.gz_bias) * io.dt

        io.state.update(mode=self.mode, heading="%.0f" % self.psi)
        if io.t - self.t_print >= self.PRINT_S:
            self.t_print = io.t
            io.print("%.2f,%s,%.2f,%.0f,%.2f,%.1f" % (io.t, self.mode, io.duty, self.psi, io.vbat(), io.temp_c()))
