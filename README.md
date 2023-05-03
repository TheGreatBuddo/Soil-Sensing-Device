# Soil-Sensing-Device Readme

Welcome to Twin Cities Engineering's project on soil health monitoring 
through gas-based measurement! Soil health plays a critical role in
agriculture and environmental sustainability. Measuring gases in soil pore 
spaces is an effective way to monitor soil health, providing valuable 
information about the soil's nutrient content and microorganism activity. 
However, commercially available gas-based soil health measurement devices 
are prohibitively expensive, limiting their accessibility to farmers and 
soil scientists. To address this issue, our engineering project team is 
developing a low-cost DIY soil gas sensing unit capable of 
measuring oxygen, carbon dioxide, and methane concentrations at 
four soil depths every hour. Our goal is to create a prototype 
that is affordable, portable, and easy to use, 
empowering more people to measure and monitor soil health.

Our project is centered around a device that attaches to four soil 
probes of varying lengths (0.25, 0.5, 0.75, and 1 meter) to extract 
soil gases. These probes are placed in a 1-meter squared area of soil, 
assuming that the soil gas concentrations will remain constant. 
The device has four ports for tubes that connect from the probes 
to the device, and each port connects to a solenoid valve inside the 
device. These valves can be opened and closed by a Mayfly microcontroller, 
allowing the device to be exposed to one probe at a time. 
A sensor chamber houses four sensors, including 
an Alphasense IRC-AT carbon dioxide sensor, Alphasense O2-A3 oxygen sensor,
and TGS2611-E00 methane sensor, along with a bme280 temperature, pressure,
and humidity sensor. The final component in the sensor chamber is a 
heating element, which is designed to prevent condensation from forming 
and potentially damaging the sensors. If the bme280 detects a dew point, 
the heating element is activated.

All work accomplished can be found in the Final Design Document and more
discussion about the project. A poster has been developed for this project
and is also uploaded in the repository. Lastly STL files for the sensor chamber
and the program to control the device are included.
