import numpy


class Altitude(object):
    def __init__(self, lambda0=8, lambda1=12, lambda2=6,
                 a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, a7=0, a8=0, sample_time=0.01):
        self.dt = sample_time
        # control gains
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        # model parameters
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.A6 = a6
        self.A7 = a7
        self.A8 = a8
        # reference
        self.z_desired = 0
        self.z_desired_dot = 0
        self.z_desired_ddot = 0
        self.z_desired_dddot = 0

        self.f = 0

    def update_parameters(self, a1, a2, a3, a4, a5, a6, a7, a8, ):
        self.A1 = a1
        self.A2 = a2
        self.A3 = a3
        self.A4 = a4
        self.A5 = a5
        self.A6 = a6
        self.A7 = a7
        self.A8 = a8

    def update_gains(self,  lambda0, lambda1, lambda2,):
        self.lambda0 = lambda0
        self.lambda1 = lambda1
        self.lambda2 = lambda2

    def update_reference(self, z_desired, z_desired_dot, z_desired_ddot, z_desired_dddot):
        self.z_desired = z_desired
        self.z_desired_dot = z_desired_dot
        self.z_desired_ddot = z_desired_ddot
        self.z_desired_dddot = z_desired_dddot

    def update_error(self, omega, z, z_dot, z_ddot):
        f_delay = self.f
        temp = (self.A2 * 2 * omega * numpy.sign(omega) * self.A7 + self.A3 / self.dt + self.lambda2 * self.A3)
        if temp == 0:
            f = 0
        else:
            f = (self.z_desired_dddot - self.lambda1 * (z_dot - self.z_desired_dot)
                 - self.lambda0 * (z - self.z_desired)
                 - self.lambda2 * (self.A1 * z_dot + self.A2 * omega * abs(omega) + self.A4 - self.z_desired_ddot)
                 - self.A1 * z_ddot - self.A2 * 2.0 * omega * numpy.sign(omega)
                 * (self.A5 * z_dot + self.A6 * omega * abs(omega) + self.A8)
                 + self.A3 * f_delay / self.dt) / temp
        self.f = f
        return self.f
