#!/usr/bin/env python3
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.



import sys

import numpy as np
import pythonnet
import time
import clr
from ctypes import *
import redpitaya_scpi as scpi

# a = pythonnet.set_runtime('mono')
# print(a)
# pythonnet.load().AddReference("System.Windows.Forms")




def main():
    rp_s = scpi.scpi(sys.argv[1])

    if (len(sys.argv) > 2):
        led = int(sys.argv[2])
    else:
        led = 0

    # print ("Blinking LED["+str(led)+"]")

    # period = 1 # seconds

    # rp_s.tx_txt('ANALOG:PIN AOUT0,0.5')
    #
    # while 1:
    #     time.sleep(period/2.0)
    #     rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(1))
    #     time.sleep(period/2.0)
    #     rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(0))

    AddrW = 0b11000000
    AddrR = 0b11000001

    rp_s.tx_txt('I2C:DEV' + str(AddrW >> 1) + ' "/dev/i2c-0"')
    rp_s.tx_txt('I2C:DEV?')
    print(rp_s.rx_txt())
    rp_s.tx_txt('I2C:FMODE ON')

    AddrW = 0b11000000
    AddrR = 0b11000001
    CWD = 0b01000000
    VPG = 0b10010000

    Imax = 3450

    # fat write
    dataA = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataB = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataC = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataD = 0

    CP = 0b00000000
    b1 = CP | dataA >> 8
    b2 = 0xFF & dataA
    b3 = CP | dataB >> 8
    b4 = 0xFF & dataB
    b5 = CP | dataC >> 8
    b6 = 0xFF & dataC
    b7 = CP | dataD >> 8
    b8 = 0xFF & dataD

    Vref = 0b10000000 | 0b1111  # 1 =>
    G = 0b11000000 | 0b0000
    rp_s.tx_txt('I2C:IO:W:B1 ' + str(G))
    rp_s.tx_txt('I2C:IO:W:B1 ' + str(Vref))

    send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
        b7) + ',' + str(b8)
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)


    """--------------------------------Polariméter-----------------------------"""
    # Load DLL library
    lib = cdll.LoadLibrary("C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPAX_64.dll")

    # Detect and initialize PAX1000 device
    instrumentHandle = c_ulong()
    IDQuery = True
    resetDevice = False
    resource = c_char_p(b"")
    deviceCount = c_int()

    # Check how many PAX1000 are connected
    lib.TLPAX_findRsrc(instrumentHandle, byref(deviceCount))
    if deviceCount.value < 1:
        print("No PAX1000 device found.")
        exit()
    else:
        print(deviceCount.value, "PAX1000 device(s) found.")
        print("")

    # Connect to the first available PAX1000
    lib.TLPAX_getRsrcName(instrumentHandle, 0, resource)
    if (0 == lib.TLPAX_init(resource.value, IDQuery, resetDevice, byref(instrumentHandle))):
        print("Connection to first PAX1000 initialized.")
    else:
        print("Error with initialization.")
        exit()
    print("")

    # Short break to make sure the device is correctly initialized
    time.sleep(2)

    # Make settings
    lib.TLPAX_setMeasurementMode(instrumentHandle, 9)
    lib.TLPAX_setWavelength(instrumentHandle, c_double(1550e-9))  # hullámhossz beállítás
    lib.TLPAX_setBasicScanRate(instrumentHandle, c_double(60))
    lib.TLPAX_setPowerRange(instrumentHandle, c_double(0.01));

    # Check settings
    wavelength = c_double()
    lib.TLPAX_getWavelength(instrumentHandle, byref(wavelength))
    print("Set wavelength [nm]: ", wavelength.value * 1e9)
    mode = c_int()
    lib.TLPAX_getMeasurementMode(instrumentHandle, byref(mode))
    print("Set mode: ", mode.value)
    scanrate = c_double()
    lib.TLPAX_getBasicScanRate(instrumentHandle, byref(scanrate))
    print("Set scanrate: ", scanrate.value)
    print("")

    scanID = c_int()
    lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    # S0 = c_double()  ### fontos sor
    S1 = c_double()  ### fontos sor
    S2 = c_double()  ### fontos sor
    S3 = c_double()  ### fontos sor

    ii = np.linspace(0,Imax, 1, dtype = 'int')


    for i in ii:
        lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
        lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
        lib.TLPAX_releaseScan(instrumentHandle, scanID)

        print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
        b3 = CP | i >> 8
        b4 = 0xFF & i
        send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(b7) + ',' + str(b8)
        rp_s.tx_txt('I2C:IO:W:B8 ' + send)
        time.sleep(0.25)

    lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    lib.TLPAX_releaseScan(instrumentHandle, scanID)

    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")

    """-----------Polariméter---------"""
    # Close
    lib.TLPAX_close(instrumentHandle)
    print("Connection to PAX1000 closed.")
    """-------------------------------"""

    rp_s.close()

if __name__ == "__main__":
    main()
