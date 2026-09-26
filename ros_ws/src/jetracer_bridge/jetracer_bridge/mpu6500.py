# -*- coding: utf-8 -*-
"""
SY-151 (MPU-6500 互換。シルクは MPU-6050) の最小ドライバ。6 軸 + 温度をバースト読み出し。

  WHO_AM_I: 0x70 = MPU-6500, 0x68 = MPU-6050 (雑音特性が違うので起動時に必ず記録する)
  レンジ・DLPF は imu_sim.yaml の range / dlpf と **必ず一致させる** (sim と実機の飽和・群遅延を揃える)
  python3 -m jetracer_bridge.mpu6500 --bus 1   # 接続とレートの確認 (ROS 無し)
"""
import argparse
import math
import struct
import time

REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG2 = 0x1D
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75
G = 9.80665

# DLPF_CFG (ジャイロ: CONFIG[2:0] / 加速度: ACCEL_CONFIG2[2:0]) と帯域 [Hz] の対応 (MPU-6500 データシート ★要確認)
GYRO_DLPF = {250: 0, 184: 1, 92: 2, 41: 3, 20: 4, 10: 5, 5: 6}
ACCEL_DLPF = {460: 0, 184: 1, 92: 2, 41: 3, 20: 4, 10: 5, 5: 6}


def _nearest(table, bw):
    return table[min(table, key=lambda k: abs(k - bw))]


class MPU6500:

    def __init__(self, bus, addr=0x68, accel_g=4, gyro_dps=500, accel_bw_hz=92, gyro_bw_hz=92, rate_hz=100):
        self.bus = bus
        self.addr = addr
        self.who = self.bus.read_byte_data(addr, REG_WHO_AM_I)
        self.bus.write_byte_data(addr, REG_PWR_MGMT_1, 0x01)     # PLL, wake
        time.sleep(0.05)
        ar = {2: 0, 4: 1, 8: 2, 16: 3}[int(accel_g)]
        gr = {250: 0, 500: 1, 1000: 2, 2000: 3}[int(gyro_dps)]
        self.bus.write_byte_data(addr, REG_ACCEL_CONFIG, ar << 3)
        self.bus.write_byte_data(addr, REG_GYRO_CONFIG, gr << 3)
        self.bus.write_byte_data(addr, REG_CONFIG, _nearest(GYRO_DLPF, gyro_bw_hz))
        self.bus.write_byte_data(addr, REG_ACCEL_CONFIG2, _nearest(ACCEL_DLPF, accel_bw_hz))
        # 内部サンプルレート 1 kHz / (1 + div)
        div = max(0, min(255, int(round(1000.0 / float(rate_hz))) - 1))
        self.bus.write_byte_data(addr, REG_SMPLRT_DIV, div)
        self.acc_scale = float(accel_g) * G / 32768.0
        self.gyr_scale = math.radians(float(gyro_dps)) / 32768.0
        self.acc_range = float(accel_g) * G
        self.gyr_range = math.radians(float(gyro_dps))

    @property
    def model(self):
        return {0x70: 'MPU-6500', 0x68: 'MPU-6050', 0x71: 'MPU-9250'}.get(self.who, f'unknown(0x{self.who:02X})')

    def read(self):
        """(accel[3] m/s², gyro[3] rad/s, temp °C, saturated)。センサ軸そのまま。"""
        raw = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 14)
        ax, ay, az, t, gx, gy, gz = struct.unpack('>hhhhhhh', bytes(raw))
        sat = any(abs(v) >= 32767 for v in (ax, ay, az, gx, gy, gz))
        acc = [ax * self.acc_scale, ay * self.acc_scale, az * self.acc_scale]
        gyr = [gx * self.gyr_scale, gy * self.gyr_scale, gz * self.gyr_scale]
        temp = t / 333.87 + 21.0
        return acc, gyr, temp, sat


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=0x68)
    ap.add_argument('--seconds', type=float, default=3.0)
    a = ap.parse_args(argv)
    from smbus2 import SMBus
    with SMBus(a.bus) as bus:
        imu = MPU6500(bus, a.addr)
        print(f'bus {a.bus} addr 0x{a.addr:02X}: WHO_AM_I=0x{imu.who:02X} ({imu.model})')
        n, t0 = 0, time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            acc, gyr, temp, sat = imu.read()
            n += 1
            if n % 50 == 0:
                print(f'acc={acc[0]:+.2f} {acc[1]:+.2f} {acc[2]:+.2f}  gyr={gyr[0]:+.3f} {gyr[1]:+.3f} {gyr[2]:+.3f}  T={temp:.1f}')
            time.sleep(0.01)
        print(f'{n / a.seconds:.0f} Hz')


if __name__ == '__main__':
    main()
