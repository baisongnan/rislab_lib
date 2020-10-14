import math
import numpy as np

mask = np.uint32((1 << 9) - 1)
i_list = [3, 2, 1, 0, ]
M_SQRT1_2 = float(math.sqrt(1/2))

# this function is for Crazyflie


def quaternion_decompress(compress_quaternion):
    # compress_quaternion = [x, y, z, w]
    comp = np.uint32(compress_quaternion)
    q = [0, 0, 0, 0, ]
    i_largest = comp >> 30
    sum_squares = 0.0
    for i in i_list:
        if i != i_largest:
            mag = np.uint(comp & mask)
            neg_bit = np.uint((comp >> 9) & 0x1)
            comp = comp >> 10
            q[i] = (M_SQRT1_2) * (float(mag)) / mask
            if neg_bit == 1:
                q[i] = -q[i]
            sum_squares += q[i] * q[i]
    q[i_largest] = math.sqrt(1.0 - sum_squares)
    return q
