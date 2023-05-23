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
import matplotlib.pyplot as plt

CP = 0b00000000
IAbsMax = 3450
Imax = 1500
Tw = 0.3

lib = cdll.LoadLibrary("C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPAX_64.dll")

# a = pythonnet.set_runtime('mono')
# print(a)
# pythonnet.load().AddReference("System.Windows.Forms")

def SVec(theta,phi):
    return np.array([np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)])

def Sdiff(S1, S2):
    return np.linalg.norm(S1-S2)

def minimize(St, rng, steps, n_pc, b_vec, rp_s, instrumentHandle):
    scanID = c_int()
    S1 = c_double()
    S2 = c_double()
    S3 = c_double()
    s_vec = np.zeros((steps,3))

    k = 0
    dmin = 2.1
    for i in rng:
        b_vec[2*n_pc] = CP | i >> 8
        b_vec[2*n_pc+1] = 0xFF & i
        send =   str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
               + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ','\
               + str(b_vec[6]) + ',' + str(b_vec[7])
        rp_s.tx_txt('I2C:IO:W:B8 ' + send)
        time.sleep(Tw)

        lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
        lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
        lib.TLPAX_releaseScan(instrumentHandle, scanID)

        #   print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
        s_vec[k, :] = [S1.value, S2.value, S3.value]

        diff = Sdiff(s_vec[k, :], St)
        if diff < dmin:
            dmin = diff
            i_min = i
            k_min = k

        print(f'Sdiff[{k}] = {diff}')
        k = k + 1

    b_vec[2*n_pc] = CP | i_min >> 8
    b_vec[2*n_pc + 1] = 0xFF & i_min
    send = str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
           + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ',' \
           + str(b_vec[6]) + ',' + str(b_vec[7])
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    time.sleep(Tw)

    lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    lib.TLPAX_releaseScan(instrumentHandle, scanID)

    s = [S1.value, S2.value, S3.value]
    diff = Sdiff(s, St)
    print(f'Smin @ [k_min = {k_min}] = {diff}')
    return i_min


def LowLim(i):
    return max(i,0)

def HighLim(i):
    return min(i,Imax)

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

    CWD = 0b01000000
    VPG = 0b10010000

    # fat write
    dataA = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataB = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataC = 0  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataD = 0

    # CP = 0b00000000
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
    # lib = cdll.LoadLibrary("C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPAX_64.dll")

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
    time.sleep(5)

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

    steps =41
    rng = np.linspace(0, Imax, steps, dtype='int')
    rng_vec = np.array([rng, rng, rng])
    di = round(Imax/(steps-1))
    di_vec = np.array([di, di, di])
    # St = SVec(0.5*np.pi, 0.3*np.pi)
    St = np.array([0, -1.0, 0])
    b_vec = np.zeros(8)
    i_min = np.empty([3])
    for i in [0,1,2]:
        for n_pc in [0,1,2]:
            i_min[n_pc] = minimize(St, rng_vec[n_pc], steps, n_pc, b_vec, rp_s, instrumentHandle)

        steps = 21
        rng_vec = np.empty([3,steps],dtype='int')
        for j in [0,1,2]:
            ll = int(LowLim(i_min[j]-4*di_vec[j]))
            hl = int(HighLim(i_min[j]+4*di_vec[j]))
            rng_vec[j] = np.round(np.linspace(ll, hl, steps))
            di_vec[j] = round((hl-ll)/(steps-1))



    scanID = c_int()

    S1 = c_double()  ### fontos sor
    S2 = c_double()  ### fontos sor
    S3 = c_double()  ### fontos sor
    #
    # steps = 41
    # ii = np.linspace(0,Imax, steps, dtype='int')
    # time.sleep(0.25)
    # s_vec = np.zeros((steps,3))
    # St = SVec(0.5, 1.2)
    # # Tw = 0.3
    #
    # print("First squeezer")
    # dmin = 2.1
    # k=0
    # for i in ii:
    #     b1 = CP | i >> 8
    #     b2 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    # #    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k,:] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k,:],St)
    #     if diff<dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k+1
    #
    #
    # b1 = CP | i_min >> 8
    # b2 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #        b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)
    #
    # print('Second squeezer')
    # dmin = 2.1
    # k = 0
    # for i in ii:
    #     b3 = CP | i >> 8
    #     b4 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    #     #   print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k, :] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k, :], St)
    #     if diff < dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k + 1
    #
    # b3 = CP | i_min >> 8
    # b4 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #     b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)
    #
    # print('Third squeezer')
    # dmin = 2.1
    # k = 0
    # for i in ii:
    #     b5 = CP | i >> 8
    #     b6 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    #     #    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k, :] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k, :], St)
    #     if diff < dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k + 1
    #
    # b5 = CP | i_min >> 8
    # b6 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #     b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)

    lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    lib.TLPAX_releaseScan(instrumentHandle, scanID)

    s = np.array([S1.value, S2.value, S3.value])
    diff = Sdiff(s, St)
    print(f'Sdiff = {diff}')

    #   plt.plot(ii, SVec[:, 0])
 #   plt.plot(ii, s_vec[:,0], ii, s_vec[:,1], ii, s_vec[:,2])
 #   plt.show()

    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")

    """-----------Polariméter---------"""
    # Close
    lib.TLPAX_close(instrumentHandle)
    print("Connection to PAX1000 closed.")
    """-------------------------------"""

    rp_s.close()

if __name__ == "__main__":
    main()
