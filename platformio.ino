; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:wiscore_rak4631]
platform = nordicnrf52
board = wiscore_rak4631
framework = arduino
monitor_speed = 115200

lib_deps = 
	beegee-tokyo/SX126x-Arduino@^2.0.14
	adafruit/Adafruit Unified Sensor
	olikraus/U8g2@^2.34.5
	adafruit/Adafruit BME680 Library@^2.0.2
build_flags = 
	
	-DRAK4631=1
