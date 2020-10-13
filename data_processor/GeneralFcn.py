import struct

# !/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import scipy.signal as signal

# IIR2Filter comes from https://github.com/poganyg/IIR-filter


class IIR2Filter(object):

    def createCoeffs(self, order, cutoff, filterType, design='butter', rp=1, rs=1, fs=0):

        # defining the acceptable inputs for the design and filterType params
        self.designs = ['butter', 'cheby1', 'cheby2']
        self.filterTypes1 = ['lowpass', 'highpass', 'Lowpass', 'Highpass', 'low', 'high']
        self.filterTypes2 = ['bandstop', 'bandpass', 'Bandstop', 'Bandpass']

        # Error handling: other errors can arise too, but those are dealt with
        # in the signal package.
        self.isThereAnError = 1  # if there was no error then it will be set to 0
        self.COEFFS = [0]  # with no error this will hold the coefficients

        if design not in self.designs:
            print('Gave wrong filter design! Remember: butter, cheby1, cheby2.')
        elif filterType not in self.filterTypes1 and filterType not in self.filterTypes2:
            print('Gave wrong filter type! Remember: lowpass, highpass',
                  ', bandpass, bandstop.')
        elif fs < 0:
            print('The sampling frequency has to be positive!')
        else:
            self.isThereAnError = 0

        # if fs was given then the given cutoffs need to be normalised to Nyquist
        if fs and self.isThereAnError == 0:
            for i in range(len(cutoff)):
                cutoff[i] = cutoff[i] / fs * 2

        if design == 'butter' and self.isThereAnError == 0:
            self.COEFFS = signal.butter(order, cutoff, filterType, output='sos')
        elif design == 'cheby1' and self.isThereAnError == 0:
            self.COEFFS = signal.cheby1(order, rp, cutoff, filterType, output='sos')
        elif design == 'cheby2' and self.isThereAnError == 0:
            self.COEFFS = signal.cheby2(order, rs, cutoff, filterType, output='sos')

        return self.COEFFS

    def __init__(self, order, cutoff, filterType, design='butter', rp=1, rs=1, fs=0):
        self.COEFFS = self.createCoeffs(order, cutoff, filterType, design, rp, rs, fs)
        self.acc_input = np.zeros(len(self.COEFFS))
        self.acc_output = np.zeros(len(self.COEFFS))
        self.buffer1 = np.zeros(len(self.COEFFS))
        self.buffer2 = np.zeros(len(self.COEFFS))
        self.input = 0
        self.output = 0

    def filter(self, input):

        # len(COEFFS[0,:] == 1 means that there was an error in the generation
        # of the coefficients and the filtering should not be used
        if len(self.COEFFS[0, :]) > 1:

            self.input = input
            self.output = 0

            # The for loop creates a chain of second order filters according to
            # the order desired. If a 10th order filter is to be created the
            # loop will iterate 5 times to create a chain of 5 second order
            # filters.
            for i in range(len(self.COEFFS)):
                self.FIRCOEFFS = self.COEFFS[i][0:3]
                self.IIRCOEFFS = self.COEFFS[i][3:6]

                # Calculating the accumulated input consisting of the input and
                # the values coming from the feedbaack loops (delay buffers
                # weighed by the IIR coefficients).
                self.acc_input[i] = (self.input + self.buffer1[i]
                                     * -self.IIRCOEFFS[1] + self.buffer2[i] * -self.IIRCOEFFS[2])

                # Calculating the accumulated output provided by the accumulated
                # input and the values from the delay bufferes weighed by the
                # FIR coefficients.
                self.acc_output[i] = (self.acc_input[i] * self.FIRCOEFFS[0]
                                      + self.buffer1[i] * self.FIRCOEFFS[1] + self.buffer2[i]
                                      * self.FIRCOEFFS[2])

                # Shifting the values on the delay line: acc_input->buffer1->
                # buffer2
                self.buffer2[i] = self.buffer1[i]
                self.buffer1[i] = self.acc_input[i]

                self.input = self.acc_output[i]

            self.output = self.acc_output[i]

        return self.output


class RealTimeProcessor(object):
    def __init__(self, ):
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.QX = 0.0
        self.QY = 0.0
        self.QZ = 0.0
        self.QW = 1.0
        self.R11 = 1.0
        self.R12 = 0.0
        self.R13 = 0.0
        self.R21 = 0.0
        self.R22 = 1.0
        self.R23 = 0.0
        self.R31 = 0.0
        self.R32 = 0.0
        self.R33 = 1.0

    def step(self, udp_data):
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
        # position
        self.X = x * 0.0005
        self.Y = y * 0.0005
        self.Z = z * 0.0005
        # qaut
        self.QX = float(qx * 0.001)
        self.QY = float(qy * 0.001)
        self.QZ = float(qz * 0.001)
        self.QW = float(qw * 0.001)
        # qaut2rotm
        yy = self.QY * self.QY
        xx = self.QX * self.QX
        zz = self.QZ * self.QZ
        xy = self.QX * self.QY
        xz = self.QX * self.QZ
        yz = self.QY * self.QZ
        wx = self.QW * self.QX
        wy = self.QW * self.QY
        wz = self.QW * self.QZ
        self.R11 = 1 - 2 * (yy + zz)
        self.R12 = 2 * (xy - wz)
        self.R13 = 2 * (xz + wy)
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx)
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2 * (xx + yy)


class Differentiator(object):
    def __init__(self, data_delayed=0):
        self.data_delayed = data_delayed

    def step(self, data, time_step):
        temp_data_d = (data - self.data_delayed)/time_step
        self.data_delayed = data
        return temp_data_d


class MyRealTimeProcessor(RealTimeProcessor):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate, sample_time, filter_on):
        RealTimeProcessor.__init__(self, )
        self.filter_on = filter_on
        self.sample_time = sample_time
        self.X_f = self.X
        self.Y_f = self.Y
        self.Z_f = self.Z
        self.FilterX = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ = IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

        self.X_d = 0
        self.Y_d = 0
        self.Z_d = 0
        self.DifferentiatorX = Differentiator()
        self.DifferentiatorY = Differentiator()
        self.DifferentiatorZ = Differentiator()

        self.R11_d = 0.0
        self.R12_d = 0.0
        self.R13_d = 0.0
        self.R21_d = 0.0
        self.R22_d = 0.0
        self.R23_d = 0.0
        self.R31_d = 0.0
        self.R32_d = 0.0
        self.R33_d = 0.0
        self.DifferentiatorR11 = Differentiator()
        self.DifferentiatorR12 = Differentiator()
        self.DifferentiatorR13 = Differentiator()
        self.DifferentiatorR21 = Differentiator()
        self.DifferentiatorR22 = Differentiator()
        self.DifferentiatorR23 = Differentiator()
        self.DifferentiatorR31 = Differentiator()
        self.DifferentiatorR32 = Differentiator()
        self.DifferentiatorR33 = Differentiator()

        self.wx = 0
        self.wy = 0
        self.wz = 0

    def step(self, udp_data):
        RealTimeProcessor.step(self, udp_data)
        self.X_f = self.FilterX.filter(self.X)
        self.Y_f = self.FilterY.filter(self.Y)
        self.Z_f = self.FilterZ.filter(self.Z)
        if self.filter_on:
            self.X_d = self.DifferentiatorX.step(self.X_f, self.sample_time)
            self.Y_d = self.DifferentiatorY.step(self.X_f, self.sample_time)
            self.Z_d = self.DifferentiatorZ.step(self.X_f, self.sample_time)
        else:
            self.X_d = self.DifferentiatorX.step(self.X, self.sample_time)
            self.Y_d = self.DifferentiatorY.step(self.X, self.sample_time)
            self.Z_d = self.DifferentiatorZ.step(self.X, self.sample_time)

        self.R11_d = self.DifferentiatorR11.step(self.R11, self.sample_time)
        self.R12_d = self.DifferentiatorR12.step(self.R12, self.sample_time)
        self.R13_d = self.DifferentiatorR13.step(self.R13, self.sample_time)
        self.R21_d = self.DifferentiatorR21.step(self.R21, self.sample_time)
        self.R22_d = self.DifferentiatorR22.step(self.R22, self.sample_time)
        self.R23_d = self.DifferentiatorR23.step(self.R23, self.sample_time)
        self.R31_d = self.DifferentiatorR31.step(self.R31, self.sample_time)
        self.R32_d = self.DifferentiatorR32.step(self.R32, self.sample_time)
        self.R33_d = self.DifferentiatorR33.step(self.R33, self.sample_time)

        self.wx = self.R13 * self.R12_d + self.R23 * self.R22_d + self.R33 * self.R32_d
        self.wy = self.R11 * self.R13_d + self.R21 * self.R23_d + self.R31 * self.R33_d
        self.wz = self.R12 * self.R11_d + self.R22 * self.R21_d + self.R32 * self.R31_d



