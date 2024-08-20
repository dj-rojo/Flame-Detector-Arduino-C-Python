# Flame Detector Project 

This project utlilizes the distinct light characteristics of flames to differentiate between a flame and an LED light source. To determine whether a detected light source is a flame from a candle or a white LED light (such as a phone flashlight), the project measures and compares light intensities across the visible and infrared spectrums. The emission spectrum of a flame typically peaks in the infrared region, whereas an LED light emits stronger intensities within the visible light spectrum.

For this project, a TMD4903 sensor is utilized to measure light intensity across various areas of visible light, specifically green, red, and blue, as well as "clear", the combination of the red, blue, and green spectrum. By comparing the Additionally, the sensor measures infrared radiation from four different directions (north, south, east, and west).

## Arduino Code: Servos and Sensors 

The Arduino code interfaces with the TMD4903 light sensor to gather measurements for the different types of light: red, clear, blue, green, clear, and infrared at each measurement point. Although a second sensor is included in the code, it was not used in the final stages of the project. Two servos are employed to move the sensor across a two-dimensional grid, enabling the detection and location of the flame or LED light source.

## Python Analysis: Flame or LED?

In this phase of the project, the data collected from the sensors are accessed directly via the USB port and imported into a Python script. The data are then displayed as a heat map, showcasing the comparison between red and clear light almost in real time. By calculating the ratio of infrared to clear light, it is possible to differentiate between a flame and LED light source. In the experiments performed for this project, even the comparison of just the red and blue spectrum of visible light sufficed to identify a test light source correctly. 

