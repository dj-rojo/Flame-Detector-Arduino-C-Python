# Flame Detector Project using Arduino Nano

This project leverages the distinct light characteristics of flames to differentiate between a flame and an LED light source. To determine whether a detected light source is a flame from a candle or a white LED light (such as a phone torch), the project measures and compares light intensities across the visible and infrared spectrums. The emission spectrum of a flame typically peaks in the infrared region, whereas an LED light emits stronger intensities within the visible light spectrum.

For this project, a TMD4903 sensor is utilized to measure light intensity across various areas of visible light, specifically green, red, and blue. These measurements are analyzed in combination. Additionally, the sensor measures infrared radiation from four different directions (north, south, east, and west).

## Arduino and Arduino Code

The Arduino code interfaces with the TMD4903 light sensor to gather measurements for the different types of light: red, clear, blue, green, and infrared at each measurement point. Although a second sensor is included in the code, it was not used in the final stages of the project. Two servos are employed to move the sensor across a two-dimensional grid, enabling the detection and location of the flame or LED light source.

## Python Analysis

In this phase of the project, the data collected from the sensors are accessed directly via the USB port and imported into a Python script. The data are then displayed as a heat map, showcasing the comparison between red and clear light almost in real time. The example image provided shows a heat map of clear light on the left and infrared light on the right.
