# for more info about PySWD module visit https://github.com/cortexm/pyswd
# for more info about tfest module visit https://github.com/giuliovv/tfest/blob/main/examples/test.ipynb
import swd
import usb
import libusb_package
import usb.core
import usb.backend.libusb1

libusb1_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)

import numpy as np
from matplotlib import pyplot as plt
import time

import tfest

INPUT_CONST = int(420000 / 1)
SINE_NUM = 3
SAMPLE_NUMS = 1500 * SINE_NUM
INPUT_MODE = 2 # step_input -> 0 | impulse_input -> 1 | sine_input -> 2
N_POLES = 1
N_ZEROS = 0

# Step Input
step_input = np.ones((SAMPLE_NUMS)) * INPUT_CONST
step_input[0:100] = 0

# Impulse Input
impulse_input = np.zeros((SAMPLE_NUMS))
impulse_input[100:200] = INPUT_CONST

# Sine Input
sine_input = (np.sin(np.arange(SAMPLE_NUMS) * np.pi * 2 * SINE_NUM / SAMPLE_NUMS) + 1) * 45

if INPUT_MODE == 1:
    input_func = impulse_input
elif INPUT_MODE == 0:
    input_func = step_input
elif INPUT_MODE == 2:
    input_func = sine_input
    INPUT_CONST = 1

dev = swd.Swd()
cm = swd.CortexM(dev)
cm.reset()
cm.run()
# dev.set_mem32(0x20000008, 0) # Ki -> 0
# dev.set_mem32(0x20000004, 0) # Kp -> 0
# dev.set_mem32(0x2000000c, 0) # Kd -> 0

y_data = []
y_360_data = []

t1 = time.time()
for i in range(SAMPLE_NUMS):
    x_360 = dev.get_mem32(0x2000012c)
    x_1024 = dev.get_mem32(0x40010024)
    # input_pulse = dev.get_mem32(0x40000034)
    # dev.set_mem32(0x40000034, int(input_func[i])) # input_pulse TIM2->CCR1
    val = int(input_func[i])
    dev.set_mem32(0x20000000, val) # input_pulse setpoint_x
    # print(f"x_360:{x_360}, x_1024:{x_1024}, input_pulse:{input_pulse}")
    if x_360 > 180:
        x_360 -= 360
    y_data.append(x_1024)
    y_360_data.append(x_360)
t2 = time.time()
t_tot = t2 - t1
TS = t_tot / SAMPLE_NUMS

t_arr = np.arange(SAMPLE_NUMS) * TS
x_arr = input_func / INPUT_CONST #* y_data[-1]
x_360_arr = input_func / INPUT_CONST #* y_360_data[-1]
y_arr = np.array(y_data)
y_360_arr = np.array(y_360_data)

# plt.plot(t_arr, x_arr, label="x_1024")
plt.plot(t_arr, x_360_arr, label="x_360")
# plt.plot(t_arr, y_arr ,label="y_1024")
plt.plot(t_arr, y_360_arr, label="y_360")
plt.legend()
plt.show()

# print("Estimating Transfer function...")
# te = tfest.tfest(x_arr, y_arr)
# te.estimate(N_ZEROS, N_POLES, time=t_tot)
# te.get_transfer_function()
# print("Transfer function:")
# print(te.get_transfer_function())
# te.plot_bode()