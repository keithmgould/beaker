## Beaker Robot

### Brief Compute Breakdown

#### Raspberry Pi.

This is used for Beaker's "high level" control, in particular any neural network control algorithms are stored here.

#### Arduino Mega

This is used for Beaker's "low level" control. It takes commands from the "high level" Raspberry Pi. It handles all the details of Beaker's physical components.

### Software Breakdown

#### CPP_LIB

As mentioned, the Arduino Mega controls all aspects of Beaker's body.
As the Arduino is programmed in C++, all of the low-level functionality for Beaker are in C++,
and housed in this directory.

#### control_algorithms/XXX_control

There are a few different control methods here, each named xxx_control (ex: p4_control)

0. PID Control. This is the classic.
0. P4 Control. This is like PID control, but only using the P, on all four members of the state.
0. Raspberry Pi Control. Used for Reinforcement Learning algorithms, housed on rPi.

### System Checks

The system checks directory holds a bunch of arduino scripts that allow the developer to
interactively test out the functionality of Beaker as well as the code that makes Beaker 
work. For example, there is an 'encoders' script that lets the developer turn the wheels
and see in the serial output that the encoders are working. It's important that the 
underlying code works before attempting to lay on more complicated algorithms. The README
in SystemChecks has some suggestions about the ordering in which someone performs the checks.
Note that these are not needed to run before each use. They are there in case things go wrong,
and you want to find out where exactly in hardware/software things are failing.