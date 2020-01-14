# Copyright (c) 2020 REV Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of REV Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from enum import Enum
from wpilib import I2C
from wpilib import DriverStation
from wpilib import Color
from wpilib import ColorShim
from hal import SimDevice
from hal import SimDouble

#
# REV Robotics Color Sensor V3
#
class ColorSensorV3:
    __K_ADDRESS = 0x52
    __K_PART_ID = 0xC2

    # This is a transformation matrix given by the chip
    # manufacturer to transform the raw RGB to CIE XYZ
    __MATRIX = (
        (0.048112847, 0.289453437, -0.084950826),
        (0.030754752, 0.339680186, -0.071569905),
        (-0.093947499, 0.072838494, 0.34024948)
    )

    class RawColor:
        def __init__(self, r: int, g: int, b: int, ir: int):
            self.red = r
            self.green = g
            self.blue = b
            self.ir = ir

    """
    Constructs a ColorSensor.

    :param port: The I2C port the color sensor is attached to
    """
    def __init__(self, port: I2C.Port):
        self.__m_i2c = I2C(port, self.__K_ADDRESS)
        self.__m_sim_device = SimDevice.create("REV Color Sensor V3", port.value, self.__K_ADDRESS)
        if self.__m_sim_device is not None:
            self.__m_sim_r = self.__m_sim_device.createDouble("Red", False, 0.0)
            self.__m_sim_g = self.__m_sim_device.createDouble("Green", False, 0.0)
            self.__m_sim_b = self.__m_sim_device.createDouble("Blue", False, 0.0)
            self.__m_sim_ir = self.__m_sim_device.createDouble("IR", False, 0.0)
            self.__m_sim_prox = self.__m_sim_device.createDouble("Proximity", False, 0.0)
            return
        if not self.check_device_id():
            return
        self.initialize_device()
        # Clear the reset flag
        self.has_reset()


    class Register(Enum):
        K_MAIN_CTRL = 0x00
        K_PROXIMITY_SENSOR_LED = 0x01
        K_PROXIMITY_SENSOR_PULSES = 0x02
        K_PROXIMITY_SENSOR_RATE = 0x03
        K_LIGHT_SENSOR_MEASUREMENT_RATE = 0x04
        K_LIGHT_SENSOR_GAIN = 0x05
        K_PART_ID = 0x06
        K_MAIN_STATUS = 0x07
        K_PROXIMITY_DATA = 0x08
        K_DATA_INFRARED = 0x0A
        K_DATA_GREEN = 0x0D
        K_DATA_BLUE = 0x10
        K_DATA_RED = 0x13

    class MainControl(Enum):
        K_RGB_MODE = 0x04 # If bit is set to 1, color channels are activated
        K_LIGHT_SENSOR_ENABLE = 0x02 # Enable light sensor
        K_PROXIMITY_SENSOR_ENABLE = 0x01 # Proximity sensor active
        OFF = 0x00 # Nothing on

    class GainFactor(Enum):
        K_GAIN_1X = 0x00
        K_GAIN_3X = 0x01
        K_GAIN_6X = 0x02
        K_GAIN_9X = 0x03
        K_GAIN_18X = 0x04

    class LEDCurrent(Enum):
        K_PULSE_2MA = 0x00
        K_PULSE_5MA = 0x01
        K_PULSE_10MA = 0x02
        K_PULSE_25MA = 0x03
        K_PULSE_50MA = 0x04
        K_PULSE_75MA = 0x05
        K_PULSE_100MA = 0x06 # Default value
        K_PULSE_125MA = 0x07

    class LEDPulseFrequency:
        K_FREQ_60KHZ = 0x18 # Default value
        K_FREQ_70KHZ = 0x40
        K_FREQ_80KHZ = 0x28
        K_FREQ_90KHZ = 0x30
        K_FREQ_100KHZ = 0x38

    class ProximitySensorResolution:
        K_PROX_RES_8BIT = 0x00
        K_PROX_RES_9BIT = 0x08
        K_PROX_RES_10BIT = 0x10
        K_PROX_RES_11BIT = 0x18

    class ProximitySensorMeasurementRate:
        K_PROX_RATE_6MS = 0x01
        K_PROX_RATE_12MS = 0x02
        K_PROX_RATE_25MS = 0x03
        K_PROX_RATE_50MS = 0x04
        K_PROX_RATE_100MS = 0x05 # Default value
        K_PROX_RATE_200MS = 0x06
        K_PROX_RATE_400MS = 0x07

    class ColorSensorResolution:
        K_COLOR_SENSOR_RES_20BIT = 0x00
        K_COLOR_SENSOR_RES_19BIT = 0x08
        K_COLOR_SENSOR_RES_18BIT = 0x10
        K_COLOR_SENSOR_RES_17BIT = 0x18
        K_COLOR_SENSOR_RES_16BIT = 0x20
        K_COLOR_SENSOR_RES_13BIT = 0x28

    class ColorSensorMeasurementRate:
        K_COLOR_RATE_25MS = 0
        K_COLOR_RATE_50MS = 1
        K_COLOR_RATE_100MS = 2
        K_COLOR_RATE_200MS = 3
        K_COLOR_RATE_500MS = 4
        K_COLOR_RATE_1000MS = 5
        K_COLOR_RATE_2000MS = 7

    """
    Configure the the IR LED used by the proximity sensor.

    These settings are only needed for advanced users, the defaults
    will work fine for most teams. Consult the APDS-9151 for more
    information on these configuration settings and how they will affect
    proximity sensor measurements.

    :param freq: The pulse modulation frequency for the proximity
                 sensor LED
    :param curr: The pulse current for the proximity sensor LED
    :param pulses: The number of pulses per measurement of the
                   proximity sensor LED (0-255)
    """
    def configure_proximity_sensor_led(self, freq: LEDPulseFrequency, curr: LEDCurrent, pulses: int) -> None:
        self.write8(Register.K_PROXIMITY_SENSOR_LED, freq | curr)
        self.write8(Register.K_PROXIMITY_SENSOR_PULSES, byte(pulses))

    """
    Configure the proximity sensor.

    These settings are only needed for advanced users, the defaults
    will work fine for most teams. Consult the APDS-9151 for more
    information on these configuration settings and how they will affect
    proximity sensor measurements.

    :param res: Bit resolution output by the proximity sensor ADC.
    :param rate: Measurement rate of the proximity sensor
    """
    def configure_proximity_sensor(self, res: ProximitySensorResolution, rate: ProximitySensorMeasurementRate) -> None:
        self.write8(Register.K_PROXIMITY_SENSOR_RATE, res | rate)

    """
    Configure the color sensor.

    These settings are only needed for advanced users, the defaults
    will work fine for most teams. Consult the APDS-9151 for more
    information on these configuration settings and how they will affect
    color sensor measurements.

    :param res: Bit resolution output by the respective light sensor ADCs
    :param rate: Measurement rate of the light sensor
    :param gain: Gain factor applied to light sensor (color) outputs
    """
    def configure_color_sensor(self, res: ColorSensorResolution, rate: ColorSensorMeasurementRate, gain: GainFactor) -> None:
        self.write8(Register.K_LIGHT_SENSOR_MEASUREMENT_RATE, res | rate)
        self.write8(Register.K_LIGHT_SENSOR_GAIN, gain)

    """
    Get the most likely color. Works best when within 2 inches and
    perpendicular to surface of interest.

    :returns: Color enum of the most likely color, including unknown if
              the minimum threshold is not met
    """
    def get_color(self) -> Color:
        r = float(self.get_red())
        g = float(self.get_green())
        b = float(self.get_blue())
        mag = r + g + b
        return ColorShim(r / mag, g / mag, b / mag)

======== END OF PROGRESS ========

    /**
     * Get the raw proximity value from the sensor ADC (11 bit). This value 
     * is largest when an object is close to the sensor and smallest when 
     * far away.
     * 
     * @return  Proximity measurement value, ranging from 0 to 2047
     */
    public int getProximity() {
        if (m_simDevice != null) {
            return (int)m_simProx.get();
        }
        return read11BitRegister(Register.kProximityData);
    }

    /**
     * Get the raw color values from their respective ADCs (20-bit).
     * 
     * @return  ColorValues struct containing red, green, blue and IR values
     */
    public RawColor getRawColor() {
        return new RawColor(getRed(), getGreen(), getBlue(), getIR());
    }

    /**
     * Get the raw color value from the red ADC
     * 
     * @return  Red ADC value
     */
    public int getRed() {
        if (m_simDevice != null) {
            return (int)m_simR.get();
        }
        return read20BitRegister(Register.kDataRed);
    }

    /**
     * Get the raw color value from the green ADC
     * 
     * @return  Green ADC value
     */
    public int getGreen() {
        if (m_simDevice != null) {
            return (int)m_simG.get();
        }
        return read20BitRegister(Register.kDataGreen);
    }

    /**
     * Get the raw color value from the blue ADC
     * 
     * @return  Blue ADC value
     */
    public int getBlue() {
        if (m_simDevice != null) {
            return (int)m_simB.get();
        }
        return read20BitRegister(Register.kDataBlue);
    }

    /**
     * Get the raw color value from the IR ADC
     * 
     * @return  IR ADC value
     */
    public int getIR() {
        if (m_simDevice != null) {
            return (int)m_simIR.get();
        }
        return read20BitRegister(Register.kDataInfrared);
    }

    /**
     * Indicates if the device reset. Based on the power on status flag in the
     * status register. Per the datasheet:
     * 
     * Part went through a power-up event, either because the part was turned
     * on or because there was power supply voltage disturbance (default at 
     * first register read).
     * 
     * This flag is self clearing
     * 
     * @return  bool indicating if the device was reset
     */
    public boolean hasReset() {
        byte[] raw = new byte[1];
    
        m_i2c.read(Register.kMainStatus.bVal, 1, raw);
    
        return (raw[0] & 0x20) != 0;
    }
    
    private boolean checkDeviceID() {
        byte[] raw = new byte[1];
        if(m_i2c.read(Register.kPartID.bVal, 1, raw)) {
            DriverStation.reportError("Could not find REV color sensor", false);
            return false;
        }

        if(kPartID != raw[0]) {
            DriverStation.reportError("Unknown device found with same I2C addres as REV color sensor", false);
            return false;
        }

        return true;
    }

    private void initializeDevice() {
        write8(Register.kMainCtrl, 
            MainControl.kRGBMode.bVal | 
            MainControl.kLightSensorEnable.bVal | 
            MainControl.kProximitySensorEnable.bVal);

        write8(Register.kProximitySensorRate, 
            ProximitySensorResolution.kProxRes11bit.bVal | 
            ProximitySensorMeasurementRate.kProxRate100ms.bVal);

        write8(Register.kProximitySensorPulses, (byte) 32);
    }

    private int read11BitRegister(Register reg) {
        byte[] raw = new byte[2];
    
        m_i2c.read(reg.bVal, 2, raw);
    
        return (((int)raw[0] & 0xFF) | (((int)raw[1] & 0xFF) << 8)) & 0x7FF;
    }

    private int read20BitRegister(Register reg) {
        byte[] raw = new byte[3];
    
        m_i2c.read(reg.bVal, 3, raw);

        return (((int)raw[0] & 0xFF) | (((int)raw[1] & 0xFF) << 8) |
                (((int)raw[2] & 0xFF) << 16)) & 0x03FFFF;
    }

    private void write8(Register reg, int data) {
        m_i2c.write(reg.bVal, data);
    }
}
