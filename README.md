[![license-badge][]][license] ![version] [![stars][]][stargazers] ![hit-count] [![github-issues][]][issues]

<h1>This library is no longer supported. New library here - https://github.com/enjoyneering/AHTxx</h1>

# Aosong ASAIR AHT10, AHT15 & AHT20

This is an Arduino library for Aosong ASAIR AHT10, AHT15 & AHT20 Digital Humidity & Temperature Sensor

- Supply voltage:               1.8v - 3.6v for AHT10, AHT15 & 2.0v - 5.5v for AHT20
- Temperature range:            -40°C..+85°C
- Temperature resolution:       0.01°C
- Temperature accuracy:         ±0.3°C
- Relative humidity range:      0%..100%
- Relative humidity resolution: 0.024%
- Relative humidity accuracy:   ±2%**
- I²C bus speed:                0Hz - 400KHz
- Recomended polling frequency: 8sec - 30sec***


Supports all sensors features:

- read humidity****
- read temperature****
- soft reset with sensor initialization

Tested on:
- Arduino AVR
- Arduino ESP8266
- Arduino ESP32
- Arduino STM32

**Prolonged exposure for 60 hours at humidity > 80% can lead to a temporary drift of the signal +3%. Sensor slowly returns to the calibrated state at normal operating conditions.

***If sampling rate of the measurement is too high, the sensor overheats. To prevent the temperature of the sensor from rising > 0.1°C, read sensor once every 2 seconds.

****The library returns 255 if a communication error occurs or if the calibration coefficient is off.

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[version]:       https://img.shields.io/badge/Version-1.1.0-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/AHT10.svg
[stargazers]:    https://github.com/enjoyneering/AHT10/stargazers
[hit-count]:     https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fenjoyneering%2FAHT10&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false
[github-issues]: https://img.shields.io/github/issues/enjoyneering/AHT10.svg
[issues]:        https://github.com/enjoyneering/AHT10/issues/
