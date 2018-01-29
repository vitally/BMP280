# Copyright (c) 2016
# Author: Vitally Tezhe
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The base for the code is taken from:
# https://github.com/gradymorgan/node-BMP280.git
# https://github.com/adafruit/Adafruit_Python_BMP.git
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import logging
import time


# BMP280 default address.
BMP280_I2CADDR           = 0x77

# Operating Modes
BMP280_ULTRALOWPOWER     = 0
BMP280_STANDARD          = 1
BMP280_HIGHRES           = 2
BMP280_ULTRAHIGHRES      = 3

# BMP280 Temperature Registers
BMP280_REGISTER_DIG_T1 = 0x88
BMP280_REGISTER_DIG_T2 = 0x8A
BMP280_REGISTER_DIG_T3 = 0x8C
# BMP280 Pressure Registers
BMP280_REGISTER_DIG_P1 = 0x8E
BMP280_REGISTER_DIG_P2 = 0x90
BMP280_REGISTER_DIG_P3 = 0x92
BMP280_REGISTER_DIG_P4 = 0x94
BMP280_REGISTER_DIG_P5 = 0x96
BMP280_REGISTER_DIG_P6 = 0x98
BMP280_REGISTER_DIG_P7 = 0x9A
BMP280_REGISTER_DIG_P8 = 0x9C
BMP280_REGISTER_DIG_P9 = 0x9E

BMP280_REGISTER_CONTROL = 0xF4
#Pressure measurments
BMP280_REGISTER_PRESSUREDATA_MSB = 0xF7
BMP280_REGISTER_PRESSUREDATA_LSB = 0xF8
BMP280_REGISTER_PRESSUREDATA_XLSB = 0xF9
#Temperature measurments
BMP280_REGISTER_TEMPDATA_MSB = 0xFA
BMP280_REGISTER_TEMPDATA_LSB = 0xFB
BMP280_REGISTER_TEMPDATA_XLSB = 0xFC

# Commands
BMP280_READCMD = 0x3F



class BMP280(object):
    def __init__(self, mode=BMP280_STANDARD, address=BMP280_I2CADDR, i2c=None, **kwargs):
        self._logger = logging.getLogger('BMP280')
        # Check that mode is valid.
        if mode not in [BMP280_ULTRALOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, BMP280_ULTRAHIGHRES]:
            raise ValueError('Unexpected mode value {0}.  Set mode to one of BMP280_ULTRALOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, or BMP280_ULTRAHIGHRES'.format(mode))
        self._mode = mode
        # Create I2C device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        # Load calibration values.
        self._load_calibration()
        self._tfine = 0
    #reading two bytes of data from each address as signed or unsigned, based on the Bosch docs
    def _load_calibration(self):
        self.cal_REGISTER_DIG_T1 = self._device.readU16LE(BMP280_REGISTER_DIG_T1)   # UINT16
        self.cal_REGISTER_DIG_T2 = self._device.readS16LE(BMP280_REGISTER_DIG_T2)   # INT16
        self.cal_REGISTER_DIG_T3 = self._device.readS16LE(BMP280_REGISTER_DIG_T3)   # INT16
        self.cal_REGISTER_DIG_P1 = self._device.readU16LE(BMP280_REGISTER_DIG_P1)   # UINT16
        self.cal_REGISTER_DIG_P2 = self._device.readS16LE(BMP280_REGISTER_DIG_P2)   # INT16
        self.cal_REGISTER_DIG_P3 = self._device.readS16LE(BMP280_REGISTER_DIG_P3)   # INT16
        self.cal_REGISTER_DIG_P4 = self._device.readS16LE(BMP280_REGISTER_DIG_P4)   # INT16
        self.cal_REGISTER_DIG_P5 = self._device.readS16LE(BMP280_REGISTER_DIG_P5)   # INT16
        self.cal_REGISTER_DIG_P6 = self._device.readS16LE(BMP280_REGISTER_DIG_P6)   # INT16
        self.cal_REGISTER_DIG_P7 = self._device.readS16LE(BMP280_REGISTER_DIG_P7)   # INT16
        self.cal_REGISTER_DIG_P8 = self._device.readS16LE(BMP280_REGISTER_DIG_P8)   # INT16
        self.cal_REGISTER_DIG_P9 = self._device.readS16LE(BMP280_REGISTER_DIG_P9)   # INT16

        self._logger.debug('T1 = {0:6d}'.format(self.cal_REGISTER_DIG_T1))
        self._logger.debug('T2 = {0:6d}'.format(self.cal_REGISTER_DIG_T2))
        self._logger.debug('T3 = {0:6d}'.format(self.cal_REGISTER_DIG_T3))
        self._logger.debug('P1 = {0:6d}'.format(self.cal_REGISTER_DIG_P1))
        self._logger.debug('P2 = {0:6d}'.format(self.cal_REGISTER_DIG_P2))
        self._logger.debug('P3 = {0:6d}'.format(self.cal_REGISTER_DIG_P3))
        self._logger.debug('P4 = {0:6d}'.format(self.cal_REGISTER_DIG_P4))
        self._logger.debug('P5 = {0:6d}'.format(self.cal_REGISTER_DIG_P5))
        self._logger.debug('P6 = {0:6d}'.format(self.cal_REGISTER_DIG_P6))
        self._logger.debug('P7 = {0:6d}'.format(self.cal_REGISTER_DIG_P7))
        self._logger.debug('P8 = {0:6d}'.format(self.cal_REGISTER_DIG_P8))
        self._logger.debug('P9 = {0:6d}'.format(self.cal_REGISTER_DIG_P9))
    #data from the datasheet example, useful for debug
    def _load_datasheet_calibration(self):
        self.cal_REGISTER_DIG_T1 = 27504
        self.cal_REGISTER_DIG_T2 = 26435
        self.cal_REGISTER_DIG_T3 = -1000
        self.cal_REGISTER_DIG_P1 = 36477
        self.cal_REGISTER_DIG_P2 = -10685
        self.cal_REGISTER_DIG_P3 = 3024
        self.cal_REGISTER_DIG_P4 = 2855
        self.cal_REGISTER_DIG_P5 = 140
        self.cal_REGISTER_DIG_P6 = -7
        self.cal_REGISTER_DIG_P7 = 15500
        self.cal_REGISTER_DIG_P8 = -14600
        self.cal_REGISTER_DIG_P9 = 6000 
    #reading raw data from registers, and combining into one raw measurment
    def read_raw_temp(self):
            """Reads the raw (uncompensated) temperature from the sensor."""
            self._device.write8(BMP280_REGISTER_CONTROL, BMP280_READCMD + (self._mode << 6))
            if self._mode == BMP280_ULTRALOWPOWER:
                    time.sleep(0.005)
            elif self._mode == BMP280_HIGHRES:
                    time.sleep(0.014)
            elif self._mode == BMP280_ULTRAHIGHRES:
                    time.sleep(0.026)
            else:
                    time.sleep(0.008)
            msb = self._device.readU8(BMP280_REGISTER_TEMPDATA_MSB)
            lsb = self._device.readU8(BMP280_REGISTER_TEMPDATA_LSB)
            xlsb = self._device.readU8(BMP280_REGISTER_TEMPDATA_XLSB)
            raw = ((msb << 8 | lsb) << 8 | xlsb) >> 4
            self._logger.debug('Raw temperature 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
            return raw
    #reading raw data from registers, and combining into one raw measurment
    def read_raw_pressure(self):
            """Reads the raw (uncompensated) pressure level from the sensor."""
            self._device.write8(BMP280_REGISTER_CONTROL, BMP280_READCMD + (self._mode << 6))
            if self._mode == BMP280_ULTRALOWPOWER:
                    time.sleep(0.005)
            elif self._mode == BMP280_HIGHRES:
                    time.sleep(0.014)
            elif self._mode == BMP280_ULTRAHIGHRES:
                    time.sleep(0.026)
            else:
                    time.sleep(0.008)
            msb = self._device.readU8(BMP280_REGISTER_PRESSUREDATA_MSB)
            lsb = self._device.readU8(BMP280_REGISTER_PRESSUREDATA_LSB)
            xlsb = self._device.readU8(BMP280_REGISTER_PRESSUREDATA_XLSB)
            raw = ((msb << 8 | lsb) << 8 | xlsb) >> 4
            self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
            return raw
    #applying calibration data to the raw reading
    def read_temperature(self):
        """Gets the compensated temperature in degrees celsius."""
        adc_T = self.read_raw_temp()
        TMP_PART1 = (((adc_T>>3) - (self.cal_REGISTER_DIG_T1<<1)) * self.cal_REGISTER_DIG_T2) >> 11
        TMP_PART2 = (((((adc_T>>4) - (self.cal_REGISTER_DIG_T1)) * ((adc_T>>4) - (self.cal_REGISTER_DIG_T1))) >> 12) * (self.cal_REGISTER_DIG_T3)) >> 14
        TMP_FINE = TMP_PART1 + TMP_PART2
        self._tfine = TMP_FINE
        temp = ((TMP_FINE*5+128)>>8)/100.0
        self._logger.debug('Calibrated temperature {0} C'.format(temp))
        return temp
    #applying calibration data to the raw reading
    def read_pressure(self):
        """Gets the compensated pressure in Pascals."""
        #for pressure calculation we need a temperature, checking if we have one, and reading data if not
        if self._tfine == 0:
            self.read_temperature()
            
        adc_P = self.read_raw_pressure()
        var1 = self._tfine - 128000
        var2 = var1 * var1 * self.cal_REGISTER_DIG_P6
        var2 = var2 + ((var1*self.cal_REGISTER_DIG_P5)<<17);
        var2 = var2 + ((self.cal_REGISTER_DIG_P4)<<35);
        var1 = ((var1 * var1 * self.cal_REGISTER_DIG_P3)>>8) + ((var1 * self.cal_REGISTER_DIG_P2)<<12);
        var1 = ((((1)<<47)+var1))*(self.cal_REGISTER_DIG_P1)>>33;

        if var1 == 0:
            return 0

        p = 1048576 - adc_P;
        p = int((((p<<31) - var2)*3125) / var1);
        var1 = ((self.cal_REGISTER_DIG_P9) * (p>>13) * (p>>13)) >> 25;
        var2 = ((self.cal_REGISTER_DIG_P8) * p) >> 19;

        p = ((p + var1 + var2) >> 8) + ((self.cal_REGISTER_DIG_P7)<<4);
        return p / 256.0;

    def read_altitude(self, sealevel_pa=101325.0):
        """Calculates the altitude in meters."""
        pressure = float(self.read_pressure())
        altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
        self._logger.debug('Altitude {0} m'.format(altitude))
        return altitude

    def read_sealevel_pressure(self, altitude_m=0.0):
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""
        pressure = float(self.read_pressure())
        p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
        self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
        return p0
