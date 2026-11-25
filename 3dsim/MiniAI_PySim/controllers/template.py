# -*- coding: utf-8 -*-
"""template.py - copy this file to start your own controller (e.g. controllers/my_ctrl.py)

    UE : import m4wd_drive as D; D.start_py("my_ctrl")          (then Play; [R] reloads the file)

The file must define   class Controller   with
    CONTROL_HZ           step() rate [Hz] (simulation time)
    __init__(**params)   tuning values; start_py(..., params={...}) overrides them
    setup(io)            once at power-on (t = 0)
    step(io)             every 1/CONTROL_HZ s
    close()              optional, when the run ends (write your own files here)

io: io.t io.dt | io.acc io.gyro (sensor axes, g / dps) | io.vbat() io.temp_c() io.v_mid() io.adc_volts(pin)
    io.duty = 0..1 (1 drive, 0 short brake) | io.print(...) | io.state[...] (HUD)
"""


class Controller:
    CONTROL_HZ = 500

    def __init__(self, DUTY=0.6, START_DELAY=1.0):
        self.DUTY = DUTY
        self.START_DELAY = START_DELAY

    def setup(self, io):
        io.duty = 0.0                     # short brake = car held

    def step(self, io):
        # ---- your control law here -------------------------------------------
        if io.t < self.START_DELAY:
            io.duty = 0.0
        else:
            io.duty = self.DUTY
        # -----------------------------------------------------------------------
        io.state["yaw_rate"] = "%.0f" % io.gyro[2]
