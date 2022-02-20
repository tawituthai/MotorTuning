#!/usr/bin/python3
from math import nan
import os
import sys
import can
import subprocess
import signal
import struct
import time

### Global
# CAN bus
can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
# Set filter
can_default_filters = [{"can_id": 0x000, "can_mask": 0x000, "extended": False}]
can_bus.set_filters(can_default_filters)
# Define msg
msg_tx = can.Message(arbitration_id = 0x000, 
                    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 
                    is_extended_id = False, 
                    is_fd = False)
# CAN ID
MotorEnable_CANID = 0x501
MotorGetPIDGain_tx_M0_CANID = 0x101
MotorGetPIDGain_tx_M1_CANID = 0x201
MotorGetPIDGain_tx_M2_CANID = 0x301
MotorGetPIDGain_tx_M3_CANID = 0x401
MotorGetPIDGain_rx_M0_CANID = 0x120
MotorGetPIDGain_rx_M1_CANID = 0x220
MotorGetPIDGain_rx_M2_CANID = 0x320
MotorGetPIDGain_rx_M3_CANID = 0x420
MotorTunePIDGain_M0_CANID = 0x102
MotorTunePIDGain_M1_CANID = 0x202
MotorTunePIDGain_M2_CANID = 0x302
MotorTunePIDGain_M3_CANID = 0x402

# RMSE output file
Report_filename = 'report_output.txt'

# Toggle Enable/Disable Motor
def toggleEnable() :
    # print('Toggle Motor Enable/Disable')
    msg_tx.arbitration_id = MotorEnable_CANID
    msg_tx.data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    can_bus.send(msg_tx)

# Send request message to get PID gain then set a filter to expected CANID and wait for reply
# After got a reply, set filter back to default value
def getPIDGain(motor_id_) :
    if ( motor_id_ == 0 ) :
        msg_tx.arbitration_id = MotorGetPIDGain_tx_M0_CANID
        can_filters = [{"can_id": 0x120, "can_mask": 0xFFF, "extended": False}]
    elif ( motor_id_ == 1 ):
        msg_tx.arbitration_id = MotorGetPIDGain_tx_M1_CANID
        can_filters = [{"can_id": 0x220, "can_mask": 0xFFF, "extended": False}]
    elif ( motor_id_ == 2 ):
        msg_tx.arbitration_id = MotorGetPIDGain_tx_M2_CANID
        can_filters = [{"can_id": 0x320, "can_mask": 0xFFF, "extended": False}]
    elif ( motor_id_ == 3 ):
        msg_tx.arbitration_id = MotorGetPIDGain_tx_M3_CANID
        can_filters = [{"can_id": 0x420, "can_mask": 0xFFF, "extended": False}]
    else:
        print('Motor id: ', motor_id_, ' not support')
        sys.exit(0)
    
    # print('getPIDGain of motor', motor_id_)
    # pack data as big-endian
    msg_tx.data = struct.pack('>hhhh', 0, 0, 0, 0)
    can_bus.send(msg_tx)

    # Set filter and wait for reply
    can_bus.set_filters(can_filters)
    msg_rx = can_bus.recv(1) # wait with 1 sec timeout
    Kp_gain = nan
    Ki_gain = nan
    if (msg_rx != None) :
        # Use big-endian format
        rx_data = struct.unpack('>hhhh', msg_rx.data)
        Kp_gain = rx_data[0]
        Ki_gain = rx_data[1]
        print('Motor id: ', motor_id_, ' has Kp: ', Kp_gain, ' and Ki: ', Ki_gain)
    else:
        print('Timeout. No reply from motor id: ', motor_id_)
    # clear filter
    # can_bus.set_filters(can_default_filters)

    # return Kp and Ki gain
    return (Kp_gain, Ki_gain)

# Tune PID gain of specific motor.
# This is a 'tune' value (not a 'set'), the value will be sum with current PID gain
def tunePIDGain(motor_id_, Kp_int_, Ki_int_) :
    if ( motor_id_ == 0 ) :
        msg_tx.arbitration_id = MotorTunePIDGain_M0_CANID
    elif ( motor_id_ == 1 ):
        msg_tx.arbitration_id = MotorTunePIDGain_M1_CANID
    elif ( motor_id_ == 2 ):
        msg_tx.arbitration_id = MotorTunePIDGain_M2_CANID
    elif ( motor_id_ == 3 ):
        msg_tx.arbitration_id = MotorTunePIDGain_M3_CANID
    else:
        print('Motor id: ', motor_id_, ' not support')
        sys.exit(0)

    print('tunePID of motor', motor_id_, ' with Kp: ', Kp_int_, ' and Ki: ', Ki_int_)
    # pack data as big-endian
    msg_tx.data = struct.pack('>hhhh', int(Kp_int_), int(Ki_int_), 0, 0)
    can_bus.send(msg_tx)

def readRMSEValue(filename_) :
    with open(filename_, 'r') as f:
        report_csv = f.readline()
    f.close()
    temp_txt = report_csv.rstrip().split(',')
    RMSE_Left = float(temp_txt[1])
    RMSE_Right = float(temp_txt[3])

    return (RMSE_Left, RMSE_Right)

def sendTestSequence() :
    # Start CAN receive and plot process
    canRecv_proc = subprocess.Popen(['python3', 'ReceiveDataAndPlot.py'], stdout=subprocess.DEVNULL)     #ignore stdout
    time.sleep(1)   # wait for child process to comeup

    # Start sending waveform
    testSeq_process = subprocess.Popen(['python3', 'SequenceTransmit.py'], stdout=subprocess.DEVNULL)    #ignore stdout
    
    # Wait until sequence finish
    testSeq_process.wait()

    # After testSeq_process is done, send signal to canRecv_proc to stop receive CAN msg and plot data
    canRecv_proc.send_signal(signal.SIGUSR1)
    time.sleep(1)   # wait for child process to end

    # Read file here
    # RMSE_Left, RMSE_Right = readRMSEValue(Report_filename)

def main() :

    doAnotherLoop = True
    numberOfLoop = 0

    print('### MotorTuning ###')
    print('--- Initialize test ---')
    #Get current PID gain
    Kp_left, Ki_left = getPIDGain(0)   # Left Motor
    Kp_right, Ki_right = getPIDGain(3)   # Right Motor
    print('---')

    # Enable motors
    toggleEnable()

    # Do test sequence
    sendTestSequence()

    # Disable motors
    toggleEnable()

    # Get RMSE value
    RMSE_Left, RMSE_Right = readRMSEValue(Report_filename)
    print('RMSE_Left: ', RMSE_Left, 'RMSE_Right: ', RMSE_Right)

    # Tune PID somehow, here

    # End first loop
    print('===================================================\n')
    time.sleep(1) 

    while (doAnotherLoop and (numberOfLoop < 2) ) :
        tunePIDGain(0, 2000, -100)
        Kp_left, Ki_left = getPIDGain(0)   # Left Motor

        #Get current PID gain
        tunePIDGain(3, 2000, -100)
        Kp_right, Ki_right = getPIDGain(3)   # Right Motor
        print(Kp_right, Ki_right)
        print('---')

        # Enable motors
        toggleEnable()

        # Do test sequence
        sendTestSequence()

        # Disable motors
        toggleEnable()

        # Get RMSE value
        RMSE_Left, RMSE_Right = readRMSEValue(Report_filename)
        print('RMSE_Left: ', RMSE_Left, 'RMSE_Right: ', RMSE_Right)

        # End loop
        print('===================================================\n')
        time.sleep(1) 

        # Then start another loop
        numberOfLoop = numberOfLoop + 1

if __name__ == "__main__" :
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        # Free CAN bus
        can_bus.shutdown()