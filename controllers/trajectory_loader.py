import mat4py
import os


class Trajectory(object):
    def __init__(self, file_path):
        MAT_Trajectory = mat4py.loadmat(
            os.path.abspath(os.path.join(os.getcwd(), "..")) + file_path)

        self.flag = 1
        self.x = MAT_Trajectory['x']
        self.y = MAT_Trajectory['y']
        self.z = MAT_Trajectory['z']
        self.xd = MAT_Trajectory['xd']
        self.yd = MAT_Trajectory['yd']
        self.zd = MAT_Trajectory['zd']
        self.xdd = MAT_Trajectory['xdd']
        self.ydd = MAT_Trajectory['ydd']
        self.zdd = MAT_Trajectory['zdd']
        self.xddd = MAT_Trajectory['xddd']
        self.yddd = MAT_Trajectory['yddd']
        self.zddd = MAT_Trajectory['zddd']
        self.xdddd = MAT_Trajectory['xdddd']
        self.ydddd = MAT_Trajectory['ydddd']
        self.zdddd = MAT_Trajectory['zdddd']
        self.traj_len = len(self.x)

    def update(self, ):
        self.flag = self.flag + 1
        if self.flag >= self.traj_len - 1:
            self.flag = self.traj_len - 1

    def get_x(self):
        return self.x[self.flag], self.xd[self.flag], self.xdd[self.flag], self.xddd[self.flag], self.xdddd[self.flag]

    def get_y(self):
        return self.y[self.flag], self.yd[self.flag], self.ydd[self.flag], self.yddd[self.flag], self.ydddd[self.flag]

    def get_z(self):
        return self.z[self.flag], self.zd[self.flag], self.zdd[self.flag], self.zddd[self.flag], self.zdddd[self.flag]
