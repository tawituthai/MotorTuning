#!/usr/bin/python3

from cmath import nan
from operator import truediv
import can
import matplotlib.pyplot as plt
import signal
import struct
import sys
from sklearn.metrics import mean_squared_error
import time
import os

### Global
# CAN bus
can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
# Set filter
can_filters = [ {"can_id": 0x502, "can_mask": 0xFFF, "extended": False},    # Speed Command
                {"can_id": 0x110, "can_mask": 0xFFF, "extended": False},    # Motor 0, Left motor
                {"can_id": 0x410, "can_mask": 0xFFF, "extended": False}]    # Motor 3, Right motor
can_bus.set_filters(can_filters)
# Define msg
msg_tx = can.Message(arbitration_id = 0x000, 
                    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 
                    is_extended_id = False, 
                    is_fd = False)

# Store data
wheelRPM_L_CMD = []
wheelRPM_R_CMD = []
wheelRPM_L_Actual = []
wheelRPM_R_Actual = []

# State machine
RUN_state = True

# RMSE output file and value
Report_filename = 'report_output.txt'
MSE_Left = nan
MSE_Right = nan

def outputReport(lineToWrite):
    if os.path.exists(Report_filename):
        os.remove(Report_filename)

    with open(Report_filename, 'w') as f:
        f.write(lineToWrite)
    f.close()
    

def usrProcess1(signum, frame):
    RUN_state = False

    MSE_Left = 9999.0
    if (len(wheelRPM_L_CMD) == len(wheelRPM_L_Actual)):
        MSE_Left = mean_squared_error(wheelRPM_L_CMD, wheelRPM_L_Actual, squared=False)

    MSE_Right = 9999.0
    if (len(wheelRPM_R_CMD) == len(wheelRPM_R_Actual)):
        MSE_Right = mean_squared_error(wheelRPM_R_CMD, wheelRPM_R_Actual, squared=False)

    # Write RMSE to report file
    txtToSend = 'RMSE_Left,' + str(MSE_Left) + ',RMSE_Right,' + str(MSE_Right) + '\n'
    outputReport(txtToSend)

    plt.figure(0)
    plt.title('Left motor')
    plt.plot(wheelRPM_L_CMD,label = "Left CMD", linestyle="-.")
    plt.plot(wheelRPM_L_Actual,label = "Left Actual", linestyle="-")
    mse_txt = 'RMSE: ' + str(MSE_Left)
    plt.text(0, 0, mse_txt, fontsize=10, color='darkred')
    plt.xlabel('X-axis')
    plt.ylabel('RPM')
    plt.legend()

    plt.figure(1)
    plt.title('Right motor')
    plt.plot(wheelRPM_R_CMD,label = "Right CMD", linestyle="-.")
    plt.plot(wheelRPM_R_Actual,label = "Right Actual", linestyle="-")
    mse_txt = 'RMSE: ' + str(MSE_Right)
    plt.text(0, 0, mse_txt, fontsize=10, color='darkred')
    plt.xlabel('X-axis')
    plt.ylabel('RPM')
    plt.legend()

    # Show plot
    plt.show()

    time.sleep(2)

    # Free CAN bus
    can_bus.shutdown()
    # End process
    sys.exit(0)

signal.signal(signal.SIGUSR1, usrProcess1)

def main():
    # print('Method 3 -- start')
    try:
        while RUN_state:
            msg = can_bus.recv(1)
            if msg is not None:
                if (msg.arbitration_id == 0x502):
                    # Use big-endian format
                    rx_data = struct.unpack('>hhhh', msg.data)
                    wheelRPM_L_CMD.append(rx_data[0])
                    wheelRPM_R_CMD.append(rx_data[3])

                elif (msg.arbitration_id == 0x110):
                    # Use big-endian format
                    rx_data = struct.unpack('>BhhhB', msg.data)
                    wheelRPM_L_Actual.append(rx_data[1])

                elif (msg.arbitration_id == 0x410):
                    # Use big-endian format
                    rx_data = struct.unpack('>BhhhB', msg.data)
                    wheelRPM_R_Actual.append(rx_data[1])

    except KeyboardInterrupt:
        pass
    finally:
        # Free CAN bus
        can_bus.shutdown()

if __name__ == "__main__":
    main()
