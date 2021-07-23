# Centipede-Robot-Script

This script is for a robot I built and programmed in the game Space Engineers. The game has a built in C# API that allows scripts to control in-game vehicles and vehicle components. The robot is a giant centipede-like walker that was designed to be able to cross difficult terrain. 

### How it Works
- The centipede is made up of 16 segments, each with 2 legs and each joined to the next by joints in 3 axes
- Each leg has a hard-coded 'walking animation', the speed and direction of which can be chosen
- When the centipede walks, the front legs will take alternating steps at regular intervals
- Part way through the step of each leg, it will trigger the leg behind it to start its step
- The pitch and roll joints between segments will move freely with some resistance
- The yaw joints between segments will move in the direction the user is steering
- There is also a system for tracking the passing of time in the simulation (the game will simulate in slower than real-time if there is too much activity to manage). This system monitors a rotor that rotates at a constant rate to calculate the exact amount of time simulated since it was last run. This is necessary because the method used to periodically run the program is not exactly regular.

### Links
Centipede Robot: https://steamcommunity.com/sharedfiles/filedetails/?id=2148426014  
API Index (fan-made): https://github.com/malware-dev/MDK-SE/wiki/Api-Index
