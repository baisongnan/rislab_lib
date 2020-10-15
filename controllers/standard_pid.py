

class PidControl(object):
    def __init__(self, sample_time, kp, ki, kd, constant):
        self.sample_time = sample_time
        self.kp = kp
        self.ki = ki
        self.ki_temp = 0
        self.kd = kd
        self.constant = constant
        self.desired_x = 0

        self.error_x = 0
        self.i_error_x = 0

    def update_reference(self, desired_x):
        self.desired_x = desired_x

    def update_error(self, x):
        delayed_error_x = self.error_x
        self.error_x = self.desired_x - x
        d_error_x = (self.error_x - delayed_error_x) / self.sample_time
        self.i_error_x = self.i_error_x + ((self.error_x + delayed_error_x) * self.sample_time / 2)
        u_x = self.kp * self.error_x + self.ki_temp * self.i_error_x + self.kd * d_error_x + self.constant
        return u_x

    def integrator_enable(self):
        self.ki_temp = self.ki

    def integrator_disable(self):
        self.ki_temp = 0
        self.i_error_x = 0
