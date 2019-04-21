#!/usr/bin/python3
# Accel and Magnetometer
# MIT License

import smbus
import struct


FXOS8700_ADDRESS                  = 0x1F     # 0011111
FXOS8700_ID                       = 0xC7     # 1100 0111

FXOS8700_REGISTER_STATUS          = 0x00
FXOS8700_REGISTER_OUT_X_MSB       = 0x01
FXOS8700_REGISTER_OUT_X_LSB       = 0x02
FXOS8700_REGISTER_OUT_Y_MSB       = 0x03
FXOS8700_REGISTER_OUT_Y_LSB       = 0x04
FXOS8700_REGISTER_OUT_Z_MSB       = 0x05
FXOS8700_REGISTER_OUT_Z_LSB       = 0x06
FXOS8700_REGISTER_WHO_AM_I        = 0x0D   # 11000111   r
FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E
FXOS8700_REGISTER_CTRL_REG1       = 0x2A   # 00000000   r/w
FXOS8700_REGISTER_CTRL_REG2       = 0x2B   # 00000000   r/w
FXOS8700_REGISTER_CTRL_REG3       = 0x2C   # 00000000   r/w
FXOS8700_REGISTER_CTRL_REG4       = 0x2D   # 00000000   r/w
FXOS8700_REGISTER_CTRL_REG5       = 0x2E   # 00000000   r/w
FXOS8700_REGISTER_MSTATUS         = 0x32
FXOS8700_REGISTER_MOUT_X_MSB      = 0x33
FXOS8700_REGISTER_MOUT_X_LSB      = 0x34
FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35
FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36
FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37
FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38
FXOS8700_REGISTER_TEMPERATURE     = 0x51
FXOS8700_REGISTER_MCTRL_REG1      = 0x5B   # 00000000   r/w
FXOS8700_REGISTER_MCTRL_REG2      = 0x5C   # 00000000   r/w
FXOS8700_REGISTER_MCTRL_REG3      = 0x5D   # 00000000   r/w

ACCEL_RANGE_2G                    = 0x00
ACCEL_RANGE_4G                    = 0x01
ACCEL_RANGE_8G                    = 0x02

ACCEL_MG_LSB_2G                   = 0.000244
ACCEL_MG_LSB_4G                   = 0.000488
ACCEL_MG_LSB_8G                   = 0.000976
MAG_UT_LSB                        = 0.1


# def twos_comp(val, bits):
#     """compute the 2's complement of int value val"""
#     if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
#         val = val - (1 << bits)        # compute negative value
#     return val                         # return positive value as is


class FXOS8700():
    scale = None  # turn readings into accelerations: 2, 4, 8 G

    def __init__(self, gs=None, bus=1):
        """
        Args
            gs: accel range: 2, 4, or 8 G's
            bus: i2c bus to use, default is 1
            verbose: print out some info at start
        """
        self.gs = gs
        self.bus = bus
        self.address = FXOS8700_ADDRESS

    def begin(self):
        self.bus = smbus.SMBus(self.bus)

        if self.readBytes(FXOS8700_REGISTER_WHO_AM_I)[0] != FXOS8700_ID:
            time.sleep(1)
            if self.readBytes(FXOS8700_REGISTER_WHO_AM_I)[0] != FXOS8700_ID:
                return False

        # Set to standby mode (required to make changes to this register)
        self.writeBytes(FXOS8700_REGISTER_CTRL_REG1, [0])

        # Configure the accelerometer
        # _range = None
        if self.gs == 2 or self.gs == None:
            self.writeBytes(FXOS8700_REGISTER_XYZ_DATA_CFG, [ACCEL_RANGE_2G])
            self.scale = ACCEL_MG_LSB_2G
            # _range = '2G'
        elif self.gs == 4:
            self.writeBytes(FXOS8700_REGISTER_XYZ_DATA_CFG, [ACCEL_RANGE_4G])
            self.scale = ACCEL_MG_LSB_4G
            # _range = '4G'
        elif self.gs == 8:
            self.writeBytes(FXOS8700_REGISTER_XYZ_DATA_CFG, [ACCEL_RANGE_8G])
            self.scale = ACCEL_MG_LSB_8G
            # _range = '8G'
        else:
            return False

        # High resolution
        self.writeBytes(FXOS8700_REGISTER_CTRL_REG2, [0x02])

        # Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode
        self.writeBytes(FXOS8700_REGISTER_CTRL_REG1, [0x15])

        # Configure the magnetometer
        # Hybrid Mode, Over Sampling Rate = 16
        self.writeBytes(FXOS8700_REGISTER_MCTRL_REG1, [0x1F])

        # Jump to reg 0x33 after reading 0x06
        self.writeBytes(FXOS8700_REGISTER_MCTRL_REG2, [0x20])

        return True


    def readBytes(self, register, numBytes=1):
        return self.bus.read_i2c_block_data(self.address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self.bus.write_i2c_block_data(self.address, register, byteVals)

    def __del__(self):
        return

    def temperature(self):
        """
        Return temperature in C
        Range is -128 to 127 C ... should i worry about the negative?
        """
        data = self.readBytes(FXOS8700_REGISTER_TEMPERATURE)[0]
        # print('intermediate tmp:', data)
        # return self.twos_comp(t, 8)
        return data

    def get(self):
        data = self.readBytes(FXOS8700_REGISTER_OUT_X_MSB, 12)  # do this???

        data = bytearray(data)
        data = struct.unpack('>hhhhhh', data)

        d = data[:3]
        accel = ([(x >> 2) * self.scale for x in d])

        d = data[3:]
        mag = ([x * MAG_UT_LSB for x in d])

        return tuple(accel), tuple(mag)
