/*

Support for the AllSensors DLV Series Low Voltage Digital Pressure Sensors

See the following datasheet:

https://www.allsensors.com/datasheets/DS-0336_Rev_E.pdf

The sensors are sold in a few varieties, with the following options:
  * Pressure range options from 5 to 60 PSI.
  * Type options of differential, gage, or absolute.

For simplicity, pre-configured subclasses of AllSensors_DLV which align with
the purchaseable model numbers are sensors can be found at the bottom of this
file.

* * *

This software is licensed under the Revised (3-clause) BSD license as follows:

Copyright (c) 2018, Jeremy Cole <jeremy@jcole.us>

All rights reserved.

See the LICENSE file for more details.

*/

#ifndef ALLSENSORS_DLV_H
#define ALLSENSORS_DLV_H

#include <stdint.h>

#include <Wire.h>

class AllSensors_DLV {
public:

  // The default I2C address from the datasheet.
  static const uint8_t I2C_ADDRESS = 0x28;
  
  // The sensor type, where part numbers:
  //   * DLV-xxxG-* are GAGE sensors.
  //   * DLV-xxxD-* are DIFFERENTIAL sensors.
  //   * DLV-xxxA-* are ABSOLUTE sensors.
  enum SensorType {
    GAGE         = 'G',
    DIFFERENTIAL = 'D',
    ABSOLUTE     = 'A',
  };
  
  enum Status {
    CURRENT     = 0,
    RESERVED    = 1,
    STALE_DATA  = 2,
    ERROR       = 3,
  };

  enum PressureUnit {
    PSI    = 'L',
    IN_H2O = 'H',
    PASCAL = 'P',
  };

  enum TemperatureUnit {
    CELCIUS    = 'C',
    FAHRENHEIT = 'F',
    KELVIN     = 'K',
  };

private:

  static const uint8_t READ_LENGTH = 4;
  
  static constexpr uint16_t FULL_SCALE_REF = (uint16_t) 1 << 14;
  
  TwoWire *bus;
  SensorType type;

  uint8_t raw_data[4] = {0, 0, 0, 0};
  
  uint16_t raw_p = 0;
  uint16_t raw_t = 0;

  float pressure_max;
  float pressure_range;
  float pressure_zero_ref;

  PressureUnit pressure_unit;
  TemperatureUnit temperature_unit;

  // Extract the 2-bit status field, bits 0-1.
  Status extractStatus() {
    return (Status)((raw_data[0] & 0b11000000) >> 6);
  }

  // Extract the 14-bit pressure field, bits 2-16.
  uint16_t extractIntegerPressure() {
    return ((uint16_t)(raw_data[0] & 0b00111111) << 8) | (uint16_t)(raw_data[1]);
  }
  
  // Extract the 11-bit temperature field, bits 17-28.
  uint16_t extractIntegerTemperature() {
    return ((uint16_t)(raw_data[2]) << 3) | ((uint16_t)(raw_data[3] & 0b11100000) >> 5);
  }
  
  // Convert a raw digital pressure read from the sensor to a floating point value in PSI.
  float transferPressure(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Pressure(psi) = 1.25 x ((P_out_dig - OS_dig) / 2^14) x FSS(psi)
    return 1.25 * (((float)raw_value - pressure_zero_ref) / FULL_SCALE_REF) * pressure_range;
  }
  
  // Convert a raw digital temperature read from the sensor to a floating point value in Celcius.
  float transferTemperature(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Temperature(degC) = T_out_dig x (200 / 2^11 - 1) - 50
    return (float)raw_value * (200.0 / 2047.0) - 50.0;
  }

  // Convert the input in PSI to the configured pressure output unit.
  float convertPressure(float psi) {
    switch(pressure_unit) {
      case PASCAL:
        return psi * 6894.75729;
      case IN_H2O:
        return psi * 27.679904;
      case PSI:
      default:
        return psi;
    }
  }

  // Convert the input in Celcius to the configured temperature output unit.
  float convertTemperature(float degree_c) {
    switch(temperature_unit) {
      case FAHRENHEIT:
        return degree_c * 1.8 + 32.0;
      case KELVIN:
        return degree_c + 273.15;
      case CELCIUS:
      default:
        return degree_c;
    }    
  }

public:

  Status status;
  float pressure;
  float temperature;

  AllSensors_DLV(TwoWire *bus, SensorType type, float pressure_max);

  // Set the configured pressure unit for data output (the default is inH2O).
  void setPressureUnit(PressureUnit pressure_unit) {
    this->pressure_unit = pressure_unit;
  }

  // Set the configured temperature unit for data output (the default is Celcius).
  void setTemperatureUnit(TemperatureUnit temperature_unit) {
    this->temperature_unit = temperature_unit;
  }

  bool readData();
};

class AllSensors_DLV_005D : public AllSensors_DLV {
public:
  AllSensors_DLV_005D(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::DIFFERENTIAL, 5.0) {}
};

class AllSensors_DLV_015D : public AllSensors_DLV {
public:
  AllSensors_DLV_015D(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::DIFFERENTIAL, 15.0) {}
};

class AllSensors_DLV_030D : public AllSensors_DLV {
public:
  AllSensors_DLV_030D(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::DIFFERENTIAL, 30.0) {}
};

class AllSensors_DLV_060D : public AllSensors_DLV {
public:
  AllSensors_DLV_060D(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::DIFFERENTIAL, 60.0) {}
};

class AllSensors_DLV_005G : public AllSensors_DLV {
public:
  AllSensors_DLV_005G(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::GAGE, 5.0) {}
};

class AllSensors_DLV_015G : public AllSensors_DLV {
public:
  AllSensors_DLV_015G(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::GAGE, 15.0) {}
};

class AllSensors_DLV_030G : public AllSensors_DLV {
public:
  AllSensors_DLV_030G(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::GAGE, 30.0) {}
};

class AllSensors_DLV_060G : public AllSensors_DLV {
public:
  AllSensors_DLV_060G(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::GAGE, 60.0) {}
};

class AllSensors_DLV_015A : public AllSensors_DLV {
public:
  AllSensors_DLV_015A(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::ABSOLUTE, 15.0) {}
};

class AllSensors_DLV_030A : public AllSensors_DLV {
public:
  AllSensors_DLV_030A(TwoWire *bus) : AllSensors_DLV(bus, AllSensors_DLV::SensorType::ABSOLUTE, 30.0) {}
};

#endif // ALLSENSORS_DLV_H