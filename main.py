import os
import sys
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
import numpy as np
from time import sleep

import swd
import usb
import libusb_package
import usb.core
import usb.backend.libusb1
import random
import time

libusb1_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)

Form = uic.loadUiType(os.path.join(os.getcwd(), "Form.ui"))[0]

memory_dict = {"kpp": 0x20000004, "kip": 0x20000008, "kdp": 0x2000000c, "dp": 0x20000010,
               "kps": 0x20000154, "kis": 0x20000018, "kds": 0x2000001c, "ds": 0x20000020,
               "x_360": 0x2000012c, "x_1024": 0x40010024, "setpoint_x": 0x20000000,
               "man_out": 0x40000034, "control_x": 0x20000028, "auto_mode": 0x20000024}

kpp = 50
kip = 1
kdp = 5
dp = 50

kps = 0
kis = 50
kds = 50
ds = 50

man_out = 0

npoles = None
nzeros = None

x_360 = None
x_1024 = None
setpoint_x = 0
STM_ONLINE = True

mode = "p"
setpoint_mode = "num"

t = np.linspace(0, 2 * np.pi, 500)
x_arr = np.zeros_like(t)
x_setpoint_arr = np.zeros_like(t)
T0 = time.time()
T = time.time()
freq = 2 * np.pi / 100
A = 90
stm32_detected = False
try:
    dev = swd.Swd()
    cm = swd.CortexM(dev)
    cm.reset()
    cm.run()
    stm32_detected = True
except Exception:
    class Cm:
        def reset(self):
            pass
        def halt(self):
            pass
    class Dev:
        def set_mem32(self, add, val):
            sleep(0.001)
        def get_mem32(self, add):
            sleep(0.001)
            if mode == "m":
                return man_out
            if setpoint_mode == "sine":
                return A * np.sin(freq * (T - 0.1))
            return setpoint_x + random.randint(-2, 2)
    dev = Dev()
    cm = Cm()

class MatplotlibWindow(QMainWindow, Form):
    def __init__(self):
        super(MatplotlibWindow, self).__init__()
        self.setupUi(self)

        self.kpp_ledit.textChanged.connect(self.update_vars)
        self.kip_ledit.textChanged.connect(self.update_vars)
        self.kdp_ledit.textChanged.connect(self.update_vars)
        self.dp_ledit.textChanged.connect(self.update_vars)

        self.kps_ledit.textChanged.connect(self.update_vars)
        self.kis_ledit.textChanged.connect(self.update_vars)
        self.kds_ledit.textChanged.connect(self.update_vars)
        self.ds_ledit.textChanged.connect(self.update_vars)

        self.vertical_slider.valueChanged.connect(self.update_vars)

        self.npole_ledit.textChanged.connect(self.update_vars)
        self.nzero_ledit.textChanged.connect(self.update_vars)

        self.pos_ledit.textChanged.connect(self.update_vars)

        self.pos_button.clicked.connect(self.update_vars)
        self.speed_button.clicked.connect(self.update_vars)
        self.man_button.clicked.connect(self.update_vars)
        self.man_num_button.clicked.connect(self.update_vars)
        self.man_scroll_button.clicked.connect(self.update_vars)
        self.step_button.clicked.connect(self.update_vars)
        self.impulse_button.clicked.connect(self.update_vars)
        self.sine_button.clicked.connect(self.update_vars)


        self.kpp_ledit.setText(str(kpp))
        self.kip_ledit.setText(str(kip))
        self.kdp_ledit.setText(str(kdp))
        self.dp_ledit.setText(str(dp))
        self.kps_ledit.setText(str(kps))
        self.kis_ledit.setText(str(kis))
        self.kds_ledit.setText(str(kds))
        self.ds_ledit.setText(str(ds))
        self.pos_ledit.setText(str(0))
        self.speed_ledit.setText(str(0))

        # stm32_detected = True
        if stm32_detected:
            self.status_label.setText("STM32 Detected!")
            self.status_label.setStyleSheet("color: green")
        else:
            self.status_label.setStyleSheet("color: red;  background-color: black")

        self.thread = None

        self.fig = Figure(frameon=False)
        self.fig.patch.set_color("w")
        self.ax = self.fig.add_subplot(111, frame_on=False)
        self.canvas = FigureCanvas(self.fig)
        self.navi = NavigationToolbar(self.canvas, self)

        l = QVBoxLayout(self.matplotlib_widget)
        l.addWidget(self.canvas)
        l.addWidget(self.navi)

        x = np.linspace(0, 2 * np.pi, 100)
        y = np.zeros_like(x)
        (self.line1,) = self.ax.plot(x, y, "b", lw=2, label="setpoint")
        (self.line2,) = self.ax.plot(x, y, "r", lw=2, label="sensor")
        self.ax.grid()
        self.ax.set_ylim(-180, 180)
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), ncol=3, fancybox=True, shadow=True)

        self.thread_plot = PlotThread(self, 0.5)
        self.thread_plot.update_trigger.connect(self.update_plot)
        self.thread_plot.start()
    
        self.thread = MicroThread(self)
        self.thread.update_trigger.connect(self.update)
        self.thread.start()
    
    def update_vars(self, num):
        global kpp, kip, kdp, kps, kis, kds, dp, ds, man_out, npoles, nzeros, mode, T0, T, freq, setpoint_mode, setpoint_x
        if not isinstance(num, bool):
            try:
                int(num)
            except Exception:
                return
        if self.sender() == self.kpp_ledit:
            kpp = int(num)
            dev.set_mem32(memory_dict["kpp"], kpp)
        elif self.sender() == self.kip_ledit:
            kip = int(num)
            dev.set_mem32(memory_dict["kip"], kip)
        elif self.sender() == self.kdp_ledit:
            kdp = int(num)
            dev.set_mem32(memory_dict["kdp"], kdp)
        elif self.sender() == self.dp_ledit:
            dp = int(num)
            dev.set_mem32(memory_dict["dp"], dp)
        elif self.sender() == self.kps_ledit:
            kps = int(num)
            dev.set_mem32(memory_dict["kps"], kps)
        elif self.sender() == self.kis_ledit:
            kis = int(num)
            dev.set_mem32(memory_dict["kis"], kis)
        elif self.sender() == self.kds_ledit:
            kds = int(num)
            dev.set_mem32(memory_dict["kds"], kds)
        elif self.sender() == self.ds_ledit:
            dev.set_mem32(memory_dict["ds"], ds)
            ds = int(num)
        elif self.sender() == self.npole_ledit:
            npoles = int(num)
        elif self.sender() == self.nzero_ledit:
            nzeros = int(num)
        elif self.sender() == self.vertical_slider:
            man_out = int(num)
            if mode == "m":
                dev.set_mem32(memory_dict["man_out"], abs(man_out * 42000))
            freq_0 = freq
            if man_out != 0:
                freq = 2 * np.pi * man_out / 100
            else:
                freq = 2 * np.pi / 100
            T0 -= (freq_0 - freq) / freq * T
        elif self.sender() == self.pos_ledit:
            setpoint_x = int(num)
            if setpoint_mode == "num":
                dev.set_mem32(memory_dict["setpoint_x"], setpoint_x)
        elif self.sender() == self.pos_button:
            mode = "p"
            dev.set_mem32(memory_dict["auto_mode"], 1)
            dev.set_mem32(memory_dict["control_x"], 1)
            self.mode_label.setText("Mode: Position Control")
        elif self.sender() == self.speed_button:
            mode = "s"
            dev.set_mem32(memory_dict["auto_mode"], 1)
            dev.set_mem32(memory_dict["control_x"], 0)
            self.mode_label.setText("Mode: Speed Control")
        elif self.sender() == self.man_button:
            mode = "m"
            dev.set_mem32(memory_dict["auto_mode"], 0)
            self.min_label.setText("0 V")
            self.max_label.setText("6 V")
            self.mode_label.setText("Mode: No Controller")
        elif self.sender() == self.man_num_button:
            setpoint_mode = "num"
            if mode == "p" or mode == "m":
                temp = self.pos_ledit.text()
                self.mode_label.setText("Mode: Position Control")
            elif mode == "s":
                temp = self.speed_ledit.text()
            if temp == "":
                setpoint_x = man_out
                if mode == "p":
                    self.pos_ledit.setText(str(setpoint_x))
                elif mode == "s":
                    self.speed_ledit.setText(str(setpoint_x))
            else:
                setpoint_x = int(temp)
        elif self.sender() == self.man_scroll_button:
            setpoint_mode = "scroll"
            self.min_label.setText("0 deg")
            self.max_label.setText("100 deg")
        elif self.sender() == self.sine_button:
            setpoint_mode = "sine"
            self.min_label.setText("0.01 Hz")
            self.max_label.setText("1 Hz")

    def update(self, x):
        self.posval_label.setText(str(x))

    def update_plot(self):
        self.line1.set_data(t, x_setpoint_arr)
        self.line2.set_data(t, x_arr)
        self.canvas.draw()
    
    
    def closeEvent(self, event):
        global STM_ONLINE
        STM_ONLINE = False
        cm.reset()
        cm.halt()


class PlotThread(QtCore.QThread):
    update_trigger = QtCore.pyqtSignal()

    def __init__(self, window, decay):
        QtCore.QThread.__init__(self, parent=window)
        self.decay = decay
        self.window = window

    def run(self):
        global t, x_arr, x_setpoint_arr, setpoint_x, A, T, T0, freq, setpoint_mode
        while STM_ONLINE:
            if setpoint_mode == "scroll" and mode != "m":
                setpoint_x = man_out
            elif setpoint_mode == "sine" and mode != "m":
                T = time.time() - T0
                setpoint_x = A * np.sin(freq * T)
            
            x_360 = dev.get_mem32(memory_dict["x_360"])
            if x_360 > 180:
                x_360 -= 360
            dev.set_mem32(memory_dict["setpoint_x"], setpoint_x)
            
            x_arr = np.roll(x_arr, 1)
            x_setpoint_arr = np.roll(x_setpoint_arr, 1)
            x_setpoint_arr[0] = setpoint_x
            x_arr[0] = x_360
            self.update_trigger.emit()

class MicroThread(QtCore.QThread):
    update_trigger = QtCore.pyqtSignal(int)

    def __init__(self, window):
        QtCore.QThread.__init__(self, parent=window)
        self.window = window

    def run(self):
        global kpp, kip, kdp, kps, kis, kds, dp, ds, man_out, npoles, nzeros
        while STM_ONLINE:
            x_360 = dev.get_mem32(memory_dict["x_360"])
            if x_360 > 180:
                x_360 -= 360
            self.update_trigger.emit(int(x_360))



if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MatplotlibWindow()
    w.show()
    sys.exit(app.exec_())
