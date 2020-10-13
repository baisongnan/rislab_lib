import time
from rislab_lib.mocap_udp import UdpReceiver

if __name__ == '__main__':
    receiver = UdpReceiver.UdpRigidBodies()
    receiver.start_thread()
    Controller_Start_Time = time.time()
    while True:
        # no sync
        AbsTime = time.time() - Controller_Start_Time
        time.sleep(0.5)
        print(receiver.get_data())
        if AbsTime > 5:
            break

    while True:
        # sync
        AbsTime = time.time() - Controller_Start_Time
        print(receiver.get_data_sync())
        if AbsTime > 10:
            break
    receiver.stop_thread()

