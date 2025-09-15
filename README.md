# embedded_oled_radar
a radar system that maps an environment onto an embedded OLED screen. This project uses an HC-SR04 and moves it 180 degrees on an SG92R. The distance data is mapped to a small OLED screen attached to the circuit

## Objective:
The HC-SR04 sensor will rotate continuously on a 180 degrees axis back and forth, scanning the environment within some range limit and output changes onto a radar image on an SSD1306 OLED screen, which receives data through I2C
