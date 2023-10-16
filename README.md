﻿# UTOPH-Capacitor_Touch-Sensor

## Goal

The goal of this project was to create a retro acrade type video game. The ultimate end thgeme was a **gun shooting drumming rhythem game**.

## My role
My role in this project was to create an interactive user input system to allow interative gamplay for the player

What I ended up decided to create was **4 capacitor touch sensors** which were acticated by human touch.

I utilized an arduino and metal plates to create these touch sensors and then hooked them up to unity

I then built a stand to place the capacitor touch sensors on that would be eventually attached to the hysical machine

## Connecting the Arduino to Unity

I used SerialPort Read and Write to send data inbetween the programs. I then multithreaded it in the arduino so that the signals sent would be smoother and consistant.

Unity Specific Settings:
 - Go under edit -> player -> api compatibility level -> .Net 4.x
 - Figure out which serial port the arduino is in by Going to device manager -> port -> see which port the Arduino is under.  Now set Unity to the same serial port, go to assets -> scripts -> level control -> PlayerInputController -> change serial port to "COM **insert number**"
