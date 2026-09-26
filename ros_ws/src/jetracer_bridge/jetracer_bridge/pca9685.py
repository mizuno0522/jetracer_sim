# -*- coding: utf-8 -*-
"""
PCA9685 (16ch PWM) を smbus2 で直接叩く最小ドライバ。パルス幅 [µs] で書く。

NvidiaRacecar の ServoKit (gain/offset) を使わないのは、[-1,1] → 750〜2250 µs の既定写像に
依存すると誰かが既定値を変えた瞬間に舵角が変わるため (設計 パラメータ タブ)。
"""
import time

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA
OSC_HZ = 25_000_000.0


class PCA9685:

    def __init__(self, bus, addr=0x40, freq_hz=50.0):
        self.bus = bus
        self.addr = addr
        self.freq = float(freq_hz)
        self.bus.write_byte_data(self.addr, MODE1, 0x00)        # reset
        time.sleep(0.005)
        prescale = int(round(OSC_HZ / (4096.0 * self.freq) - 1.0))
        old = self.bus.read_byte_data(self.addr, MODE1)
        self.bus.write_byte_data(self.addr, MODE1, (old & 0x7F) | 0x10)   # sleep
        self.bus.write_byte_data(self.addr, PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, MODE1, old)
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, MODE1, old | 0xA0)  # restart + auto-increment
        self.bus.write_byte_data(self.addr, MODE2, 0x04)        # totem pole
        self.period_us = 1e6 / self.freq

    def set_pulse_us(self, ch, us):
        ticks = int(round(max(0.0, min(self.period_us, float(us))) / self.period_us * 4096.0))
        ticks = max(0, min(4095, ticks))
        reg = LED0_ON_L + 4 * int(ch)
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F])

    def off(self, ch):
        reg = LED0_ON_L + 4 * int(ch)
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, 0, 0x10])   # full off

    def all_off(self):
        self.bus.write_i2c_block_data(self.addr, ALL_LED_ON_L, [0, 0, 0, 0x10])
