from .standard_pid import PidControl


class Pid3D(object):
    def __init__(self, sample_time,
                 k_p_x, k_i_x, k_d_x, constant_x,
                 k_p_y, k_i_y, k_d_y, constant_y,
                 k_p_z, k_i_z, k_d_z, constant_z,):
        self.pid_x = PidControl(sample_time, k_p_x, k_i_x, k_d_x, constant_x)
        self.pid_y = PidControl(sample_time, k_p_y, k_i_y, k_d_y, constant_y)
        self.pid_z = PidControl(sample_time, k_p_z, k_i_z, k_d_z, constant_z)

    def update_reference(self, desired_x, desired_y, desired_z):
        self.pid_x.update_reference(desired_x)
        self.pid_y.update_reference(desired_y)
        self.pid_z.update_reference(desired_z)

    def update_error(self, x, y, z):
        u_x = self.pid_x.update_error(x)
        u_y = self.pid_y.update_error(y)
        u_z = self.pid_z.update_error(z)
        return u_x, u_y, u_z

    def integrator_enable(self):
        self.pid_x.integrator_enable()
        self.pid_y.integrator_enable()
        self.pid_z.integrator_enable()

    def integrator_disable(self):
        self.pid_x.integrator_disable()
        self.pid_y.integrator_disable()
        self.pid_z.integrator_disable()
