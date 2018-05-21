# Beaker Robot

## Brief Compute Breakdown

### Raspberry Pi.

This is used for Beaker's "high level" control, in particular any neural network control algorithms are stored here.

### Arduino Mega

This is used for Beaker's "low level" control. It can take commands from the "high level" Raspberry Pi. It handles all the details of Beaker's physical components, like input sensors, telemetry transmission, and motor control.

## Software Breakdown

Since Beaker utilizes both an Arduino and a Raspberry Pi, code is broken up into two main directories named (drum roll...) arduino and raspberry_pi.

### Arduino

#### arduino/lib

The bulk of Beaker's Arduino code is housed in the `arduino/lib` directory. Here you will find literally all functionality wrapped in classes. Each lib has a header comment explaining the libs role.

#### arduino/controllers

The controllers realize balance and basic functionality. There are some oldies but goodies, like pid_control. Each controller
has a header-comment explaining what it does. Of note is the `rPI_control`, which is the controller that acts as a middle-man 
between the Raspberry Pi and Beaker's body. This is the controller used for any neural-network control.

When working with any of the non-neural network controllers, such as PID, you can easily interact with Beaker wirelessly via the 
`raspberry_pi/terminal/arduino.py` script. Once the arduino app is loaded and running, ssh into the pi, run the script and type
`H` to see a list of options for how to interact (and tune) the controller you are playing with. 

#### arduino/system_checks

The system checks directory holds a bunch of arduino scripts that allow the developer to
interactively test out the functionality of Beaker as well as the low-level libraries that makes Beaker 
work. For example, there is an 'encoders' script that lets the developer turn the wheels (by hand)
and see in the serial output that the encoders are working. It's important that the 
underlying code works before attempting to lay on more complicated algorithms. The README
in System_checks has some suggestions about the ordering in which someone performs the checks.
Note that these are not needed to run before each use. They are there in case things go wrong,
and you want to find out where exactly in hardware/software things are failing.

### Raspberry_Pi

This directory holds the python-based raspberry pi scripts. These are primarily Tensorflow-based neural-networks
that usilize the `rPI_control` arduino app mentioned above.

