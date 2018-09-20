from __future__ import print_function

import myo as libmyo; libmyo.init()
import time
import sys
import math
import serial
import json

class Listener(libmyo.DeviceListener):
    """
    Listener implementation. Return False from any function to
    stop the Hub.
    """
    # Output only 0.5 seconds
    interval = 0.5

    def __init__(self):
        super(Listener, self).__init__()
        self.orientation = None
        self.pose = libmyo.Pose.rest
        self.emg_enabled = False
        self.locked = False
        self.rssi = None
        self.emg = None
        self.last_time = 0

    def on_connect(self, myo, timestamp, firmware_version):
        myo.vibrate('medium')
        myo.request_rssi()
        myo.request_battery_level()

    def on_rssi(self, myo, timestamp, rssi):
        self.rssi = rssi

    def on_pose(self, myo, timestamp, pose):
        if pose == libmyo.Pose.double_tap:
            myo.set_stream_emg(libmyo.StreamEmg.enabled)
            self.emg_enabled = True
        elif pose == libmyo.Pose.fingers_spread:
            myo.set_stream_emg(libmyo.StreamEmg.disabled)
            self.emg_enabled = False
            self.emg = None
        self.pose = pose

    def on_orientation_data(self, myo, timestamp, orientation):
        quat = orientation
        ctime = time.time()
        if (ctime - self.last_time) < self.interval:
            return
        self.last_time = ctime
        print("Orientation:", quat.x, quat.y, quat.z, quat.w)
        roll = math.atan2(2.0 * (quat.w * quat.x + quat.y * quat.z), 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y));
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (quat.w * quat.y - quat.z * quat.x))));
        yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
        rollW = (roll + math.pi) / (math.pi * 2.0) * 180
        pitchW = (pitch + math.pi) / (math.pi * 2.0) * 180
        yawW = (yaw + math.pi) / (math.pi * 2.0) * 180
        print("Eulers: ROLL", rollW, "PITCH", pitchW, "YAW" , yawW)
        ser = serial.Serial('COM5', 9600, timeout=120)
        data = str(rollW) + "," + str(pitchW) + "," + str(yawW)
        ser.write(str.encode(data))

    def on_accelerometor_data(self, myo, timestamp, acceleration):
        pass

    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        pass

    def on_emg_data(self, myo, timestamp, emg):
        self.emg = emg

    def on_unlock(self, myo, timestamp):
        self.locked = False

    def on_lock(self, myo, timestamp):
        self.locked = True

def main():
    print("Connecting to Myo ... Use CTRL^C to exit.")
    try:
        hub = libmyo.Hub()
    except MemoryError:
        print("Myo Hub could not be created. Make sure Myo Connect is running.")
        return

    hub.set_locking_policy(libmyo.LockingPolicy.none)
    hub.run(1000, Listener())

    # Listen to keyboard interrupts and stop the hub in that case.
    try:
        while hub.running:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nQuitting ...")
    finally:
        print("Shutting down hub...")
        hub.shutdown()


if __name__ == '__main__':
    main()
