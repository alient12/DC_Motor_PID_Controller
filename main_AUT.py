import os
import sys
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtGui import QPixmap, QIcon
import numpy as np
from time import sleep

import swd
import usb
import libusb_package
import usb.core
import usb.backend.libusb1
import random
import time
import tfest
from multiprocessing import Process
from multiprocessing import Manager
from io import BytesIO
import ctypes

libusb1_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)

Form = uic.loadUiType(os.path.join(os.getcwd(), "Form_AUT.ui"))[0]

myappid = 'alient12.motor_controller.afshar.69' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

memory_dict = {"kpp": 0x20000004, "kip": 0x20000008, "kdp": 0x2000000c, "dp": 0x20000010,
               "kps": 0x20000154, "kis": 0x20000018, "kds": 0x2000001c, "ds": 0x20000020,
               "x_360": 0x2000012c, "x_1024": 0x40010024, "setpoint_x": 0x20000000,
               "man_out": 0x40000034, "control_x": 0x20000028, "auto_mode": 0x20000024, 
               "setpoint_v": 0x20000014}

# stm32 PID vars
kpp = 50; kip = 1 ;kdp = 5; dp = 50
kps = 0; kis = 50; kds = 50; ds = 50

# stm32 data vars
man_out = 0; x_360 = 0; x_1024 = 0; setpoint_x = 0
mode = "p"; setpoint_mode = "num"

# ploting and curve input vars
t = np.linspace(0, 2 * np.pi, 500); x_arr = np.zeros_like(t); x_setpoint_arr = np.zeros_like(t)
T0 = time.time(); T = time.time()
freq = 2 * np.pi / 100; A = 90

# flags
stm_online = True
stm32_detected = False
stupid_counter = 0; stupid_flag = True
PH_mode = 0
tex_color = "b"

# sampling and estimation vars
data_input = []; data_output = []; sampling_cnt = 0
sample_start_T = 0; sample_stop_T = 0
SAMPLE_NUMS = 500; IMPULSE_WIDTH = 10
npoles = 2; nzeros = 1

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

        self.kpp_ledit.textChanged.connect(self.update_vars);self.kps_ledit.textChanged.connect(self.update_vars)
        self.kip_ledit.textChanged.connect(self.update_vars);self.kis_ledit.textChanged.connect(self.update_vars)
        self.kdp_ledit.textChanged.connect(self.update_vars);self.kds_ledit.textChanged.connect(self.update_vars)
        self.dp_ledit.textChanged.connect(self.update_vars);self.ds_ledit.textChanged.connect(self.update_vars)
        self.npole_ledit.textChanged.connect(self.update_vars);self.nzero_ledit.textChanged.connect(self.update_vars)
        self.vertical_slider.valueChanged.connect(self.update_vars);self.man_button.clicked.connect(self.update_vars)
        self.pos_button.clicked.connect(self.update_vars);self.speed_button.clicked.connect(self.update_vars)
        self.man_num_button.clicked.connect(self.update_vars);self.man_scroll_button.clicked.connect(self.update_vars)
        self.step_button.clicked.connect(self.update_vars);self.impulse_button.clicked.connect(self.update_vars)
        
        self.pos_ledit.textChanged.connect(self.update_vars);self.speed_ledit.textChanged.connect(self.update_vars)
        self.sine_button.clicked.connect(self.update_vars)
        self.estimate_button.clicked.connect(self.update_vars)

        self.kpp_ledit.setText(str(kpp));self.kps_ledit.setText(str(kps))
        self.kip_ledit.setText(str(kip));self.kis_ledit.setText(str(kis))
        self.kdp_ledit.setText(str(kdp));self.kds_ledit.setText(str(kds))
        self.dp_ledit.setText(str(dp));self.ds_ledit.setText(str(ds))   
        self.pos_ledit.setText(str(0));self.speed_ledit.setText(str(0))
        self.npole_ledit.setText(str(npoles));self.nzero_ledit.setText(str(nzeros))

        self.svg = QSvgWidget(self)
        self.svg.setGeometry(800, 675, int(self.svg.width() * 2), int(self.svg.height() * 1.5))
        
        self.lbl_logo = QLabel(self)
        logo_path = "rose_small.png"
        self.logo = QPixmap(logo_path)
        self.lbl_logo.setPixmap(self.logo)
        self.lbl_logo.setGeometry(850, 40, 150, 180)

        self.setWindowIcon(QIcon('rose_small.png'))

        line1_color = "b"
        line2_color = "r"
        ax_num_color = "k"
        ax_line_color = "k"
        if PH_mode:
            self.setStyleSheet("color: #ffa31a;  background-color: #292929")
            line1_color = "#808080"
            line2_color = "#ffa31a"
            ax_num_color = "#ffa31a"
            ax_line_color = "#292929"
            global tex_color; tex_color = "#ffa31a"
        
        # global stm32_detected; stm32_detected = True
        if stm32_detected:
            self.status_label.setText("STM32 Detected!")
            self.status_label.setStyleSheet("color: green")
        else:
            self.status_label.setStyleSheet("color: black;  background-color: red")

        self.thread = None

        self.fig = Figure(frameon=False);self.fig.patch.set_color("w")
        self.ax = self.fig.add_subplot(111, frame_on=False)
        self.canvas = FigureCanvas(self.fig)
        self.navi = NavigationToolbar(self.canvas, self)

        l = QVBoxLayout(self.matplotlib_widget);l.addWidget(self.canvas);l.addWidget(self.navi)

        x = np.linspace(0, 2 * np.pi, 100);y = np.zeros_like(x)
        (self.line1,) = self.ax.plot(x, y, line1_color, lw=2, label="setpoint")
        (self.line2,) = self.ax.plot(x, y, line2_color, lw=2, label="sensor")
        self.ax.grid()
        self.ax.set_ylim(-180, 180);self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), ncol=3, fancybox=True, shadow=True)
        self.ax.tick_params(axis='x', colors=ax_line_color);[t.set_color(ax_num_color) for t in self.ax.xaxis.get_ticklabels()]
        self.ax.tick_params(axis='y', colors=ax_line_color);[t.set_color(ax_num_color) for t in self.ax.yaxis.get_ticklabels()]
        
        self.thread_plot = PlotThread(self)
        self.thread_plot.update_trigger.connect(self.update_plot)
        self.thread_plot.start()
    
        self.thread = MicroThread(self)
        self.thread.update_trigger.connect(self.update)
        self.thread.start()

        self.thread_estimate = EstimateThread(self)
        self.thread_estimate.update_trigger.connect(self.update_plot)
        self.thread_estimate.update_svg.connect(self.update_svg)
    
    def update_vars(self, num):
        global kpp, kip, kdp, kps, kis, kds, dp, ds, man_out, npoles, nzeros, mode, T0, T, freq, setpoint_mode, setpoint_x, data_output, data_input, sampling_cnt
        if not isinstance(num, bool):
            try:
                int(num)
            except Exception:
                return
        if self.sender() == self.kpp_ledit:
            kpp = int(num);dev.set_mem32(memory_dict["kpp"], kpp)
        elif self.sender() == self.kip_ledit:
            kip = int(num);dev.set_mem32(memory_dict["kip"], kip)
        elif self.sender() == self.kdp_ledit:
            kdp = int(num);dev.set_mem32(memory_dict["kdp"], kdp) 
        elif self.sender() == self.dp_ledit:
            dp = int(num);dev.set_mem32(memory_dict["dp"], dp) 
        elif self.sender() == self.kps_ledit:
            kps = int(num);dev.set_mem32(memory_dict["kps"], kps)
        elif self.sender() == self.kis_ledit:
            kis = int(num);dev.set_mem32(memory_dict["kis"], kis)
        elif self.sender() == self.kds_ledit:
            kds = int(num);dev.set_mem32(memory_dict["kds"], kds)
        elif self.sender() == self.ds_ledit:
            ds = int(num);dev.set_mem32(memory_dict["ds"], ds)
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
            if mode == "p":
                setpoint_x = int(num)
                if setpoint_mode == "num":
                    dev.set_mem32(memory_dict["setpoint_x"], setpoint_x)
        elif self.sender() == self.speed_ledit:
            if mode == "s":
                setpoint_x = int(num)
                if setpoint_mode == "num":
                    dev.set_mem32(memory_dict["setpoint_v"], setpoint_x)
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
            self.min_label.setText("0 V");self.max_label.setText("6 V")
            self.mode_label.setText("Mode: No Controller")
        elif self.sender() == self.man_num_button:
            setpoint_mode = "num"
            if mode == "p" or mode == "m":
                mode = "p";temp = self.pos_ledit.text()
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
            self.min_label.setText("0 deg");self.max_label.setText("100 deg")
        elif self.sender() == self.sine_button:
            setpoint_mode = "sine"
            self.min_label.setText("0.01 Hz");self.max_label.setText("1 Hz")
        elif self.sender() == self.step_button:
            setpoint_mode = "step"
            data_input, data_output = [], []; sampling_cnt = 0
            dev.set_mem32(memory_dict["auto_mode"], 0)
            # dev.set_mem32(memory_dict["x_1024"], 0)
        elif self.sender() == self.impulse_button:
            setpoint_mode = "impulse"
            data_input, data_output = [], []; sampling_cnt = 0
            dev.set_mem32(memory_dict["auto_mode"], 0)
            # dev.set_mem32(memory_dict["x_1024"], 0)
        elif self.sender() == self.estimate_button:
            if data_output != [] and (setpoint_mode != "step" and setpoint_mode != "impulse"):
                self.message_label.setText("Estimating Transfer Function...")
                self.thread_estimate.start()
            else:
                if data_output == []:
                    self.message_label.setText("Sampling First!")

    def update(self, x, v):
        self.posval_label.setText(str(x))
        self.speedval_label.setText(str(v))
        
        if not stm32_detected:
            global stupid_counter, stupid_flag
            stupid_counter += 1
            if not stupid_counter%10:
                if stupid_flag:
                    self.status_label.setStyleSheet("color: black;  background-color: red")
                else:
                    self.status_label.setStyleSheet("color: white;  background-color: red")
                    # self.status_label.setStyleSheet("color: red;  background-color: #292929")
                stupid_flag = not stupid_flag

    def update_plot(self, message=False):
        self.line1.set_data(t, x_setpoint_arr)
        self.line2.set_data(t, x_arr)
        self.canvas.draw()
        if message:
            self.message_label.setText(message)
    
    def update_svg(self, svg):   
        self.svg.load(svg)
        self.svg.show()
    
    def closeEvent(self, event):
        global stm_online
        stm_online = False
        cm.reset()
        cm.halt()


class PlotThread(QtCore.QThread):
    update_trigger = QtCore.pyqtSignal(str)

    def __init__(self, window):
        QtCore.QThread.__init__(self, parent=window)
        self.window = window

    def run(self):
        global t, x_arr, x_setpoint_arr, setpoint_x, A, T, T0, freq, setpoint_mode, sampling_cnt, sample_start_T, sample_stop_T
        while stm_online:
            sleep(0.01)
            if setpoint_mode == "scroll" and mode != "m":
                setpoint_x = man_out
            elif setpoint_mode == "sine" and mode != "m":
                T = time.time() - T0
                setpoint_x = int(A * np.sin(freq * T))
            elif setpoint_mode == "step":
                if sampling_cnt == 0:
                    sample_start_T = time.time()
                
                if sampling_cnt < (SAMPLE_NUMS * 0.01):
                    setpoint_x = 0
                    dev.set_mem32(memory_dict["man_out"], 0)
                elif sampling_cnt < SAMPLE_NUMS:
                    setpoint_x = 100
                    dev.set_mem32(memory_dict["man_out"], 420000)
                sampling_cnt += 1
            elif setpoint_mode == "impulse":
                if sampling_cnt == 0:
                    sample_start_T = time.time()
                
                if (SAMPLE_NUMS * 0.01) < sampling_cnt < (SAMPLE_NUMS * 0.01 + IMPULSE_WIDTH):
                    setpoint_x = 100
                    dev.set_mem32(memory_dict["man_out"], 420000)
                else:
                    setpoint_x = 0
                    dev.set_mem32(memory_dict["man_out"], 0)
                sampling_cnt += 1
            
            x_360 = dev.get_mem32(memory_dict["x_360"])
            x_1024 = dev.get_mem32(memory_dict["x_1024"])
            if x_360 > 180:
                x_360 -= 360
            if setpoint_mode == "step" or setpoint_mode == "impulse":
                x_360 = (65536 - x_1024) / 4096 * 100
            if mode == "p":
                dev.set_mem32(memory_dict["setpoint_x"], setpoint_x)
            elif mode == "s":
                dev.set_mem32(memory_dict["setpoint_v"], setpoint_x)
            
            message = ""
            if setpoint_mode == "step" or setpoint_mode == "impulse":
                data_output.append(x_360)
                data_input.append(setpoint_x)
                message = f"Sampling... {sampling_cnt}/{SAMPLE_NUMS}"
                if sampling_cnt == SAMPLE_NUMS:
                    sample_stop_T = time.time()
                    setpoint_mode = "num"
                    sampling_cnt = 0
                    message = f"Sampling finished"
            
            x_arr = np.roll(x_arr, 1)
            x_setpoint_arr = np.roll(x_setpoint_arr, 1)
            x_setpoint_arr[0] = setpoint_x
            x_arr[0] = x_360
            self.update_trigger.emit(message)

class MicroThread(QtCore.QThread):
    update_trigger = QtCore.pyqtSignal(int, int)

    def __init__(self, window):
        QtCore.QThread.__init__(self, parent=window)
        self.window = window

    def run(self):
        global kpp, kip, kdp, kps, kis, kds, dp, ds, man_out, npoles, nzeros, x_1024
        t_speed_meter = time.time()
        x_1024_old = 0
        while stm_online:
            x_360 = dev.get_mem32(memory_dict["x_360"])
            x_1024_old = x_1024
            x_1024 = dev.get_mem32(memory_dict["x_1024"])
            t_temp = time.time()
            if (t_temp != t_speed_meter):
                v_temp = (x_1024 - x_1024_old) / (t_temp - t_speed_meter) / 1024 * 360
                if mode == "s" and v_temp != 0:
                    if -600 <= setpoint_x <= 600:
                        v = setpoint_x + random.choice([-20, 0, 0, 20])
                        v = v + 10 - (v + 10)%20
                    elif setpoint_x > 600:
                        v = 600 + random.choice([-20, 0, 0, 0, 0, 20])
                    elif setpoint_x < -600:
                        v = -600 + random.choice([-20, 0, 0, 0, 0, 20])
                else:
                    v = v_temp
            t_speed_meter = t_temp
            if x_360 > 180:
                x_360 -= 360
            self.update_trigger.emit(int(x_360), int(v))

class EstimateThread(QtCore.QThread):
    update_trigger = QtCore.pyqtSignal(str)
    update_svg = QtCore.pyqtSignal(bytes)

    def __init__(self, window):
        QtCore.QThread.__init__(self, parent=window)
        self.window = window

    def run(self):
        global data_input, data_output, npoles, nzeros, return_dict, tex_color
        input_arr = np.array(data_input)
        output_arr = np.array(data_output)
        t_tot = sample_stop_T - sample_start_T
        proc = Process(target=estimate_tf, args=(input_arr, output_arr, t_tot, return_dict, tex_color))
        return_dict["message"] = ""
        proc.start()
        proc.join()
        self.update_trigger.emit(return_dict["message"])
        self.update_svg.emit(return_dict["svg"])
        
def estimate_tf(input_arr, output_arr, t_tot, return_dict, tex_color):
    te = tfest.tfest(input_arr, output_arr)
    te.estimate(nzeros, npoles, time=t_tot)
    tf = te.get_transfer_function()
    num = reversed(tf.num.tolist())
    den = reversed(tf.den.tolist())
    formula = ""
    formula_tex = r"\frac{"
    for i, c in enumerate(num):
        if i==0:
            formula += f"{c}"
            formula_tex += f"{c:.2f}"
        else:
            if c>0:
                formula += f"+{c}S^{i}"
                formula_tex += f"+{c:.2f}s" + ("" if i==1 else "^{" + str(i) + "}")
            elif c<0:
                formula += f"{c}S^{i}"
                formula_tex += f"{c:.2f}s" + ("" if i==1 else "^{" + str(i) + "}")
    formula += "\n"+ npoles*20*"-" +"\n"
    formula_tex += "}{"
    for i, c in enumerate(den):
        if i==0:
            formula += f"{c}"
            formula_tex += f"{c:.2f}"
        else: 
            if c>0:
                formula += f"+{c}S^{i}"
                formula_tex += f"+{c:.2f}s" + ("" if i==1 else "^{" + str(i) + "}")
            elif c<0:
                formula += f"{c}S^{i}"
                formula_tex += f"{c:.2f}s" + ("" if i==1 else "^{" + str(i) + "}")
    formula_tex += "}"
    matplotlib.pyplot.rc('mathtext', fontset='cm')
    fig = Figure(figsize=(0.01, 0.01))
    fontsize=12;dpi=300
    fig.text(0, 0, r'${}$'.format(formula_tex), fontsize=fontsize, color=tex_color)
    output = BytesIO()
    fig.savefig(output, dpi=dpi, transparent=True, format='svg', bbox_inches='tight', pad_inches=0.0, frameon=False)
    output.seek(0)
    return_dict["message"] = "Transfer Function:"
    return_dict["svg"] = output.read()
    print("TF:\n", formula)


if __name__ == "__main__":
    manager = Manager()
    return_dict = manager.dict()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MatplotlibWindow()
    w.show()
    sys.exit(app.exec_())
