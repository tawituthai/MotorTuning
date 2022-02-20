#!/usr/bin/python3

import signal
import can
import time
import sys
import subprocess

### Global
# CAN bus
can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
# Set filter
can_default_filters = [{"can_id": 0x000, "can_mask": 0x000, "extended": False}]
can_bus.set_filters(can_default_filters)
# Define msg
msg = can.Message(arbitration_id = 0x502, 
                  data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 
                  is_extended_id = False, 
                  is_fd = False)

def doSend(msg_in):
    msg.data = bytearray(msg_in)
    try:
        can_bus.send(msg)
    except can.CanError:
        print("Message NOT sent")

def main():
    start_time = time.time()

    run_state = True
    tx_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    while run_state:
        elapsed_time = time.time() - start_time
        if (elapsed_time < 1.0) :
            #Send vel = 0%
            # print("Speed 0%")
            tx_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        elif (elapsed_time >= 1.0 and elapsed_time < 4.0):
            #Send vel = 20%
            # print("Speed 20%")
            tx_data = [0x00, 0x32, 0xFF, 0xCE, 0x00, 0x32, 0xFF, 0xCE]
        elif (elapsed_time >= 4.0 and elapsed_time < 5.0):
            #Send vel = 0%
            # print("Speed 0%")
            tx_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        elif (elapsed_time >= 5.0 and elapsed_time < 8.0):
            #Send vel = 40%
            # print("Speed 40%")
            tx_data = [0x00, 0x64, 0xFF, 0x9C, 0x00, 0x64, 0xFF, 0x9C]
        elif (elapsed_time >= 8.0 and elapsed_time < 9.0):
            #Send vel = 0%
            # print("Speed 0%")
            tx_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        elif (elapsed_time >= 9.0 and elapsed_time < 12.0):
            #Send vel = 60%
            # print("Speed 60%")
            tx_data = [0x00, 0xC8, 0xFF, 0x38, 0x00, 0xC8, 0xFF, 0x38]
        elif (elapsed_time >= 12.0 and elapsed_time < 13.0):
            #Send vel = 0%
            # print("Speed 0%")
            tx_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            # print("Test end")
            run_state = False
        
        # Send CAN msg
        doSend(tx_data)
        time.sleep(0.01)    # cycle time 10 ms. -- 100 Hz

    can_bus.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    main()
