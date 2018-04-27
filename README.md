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

#### XXX_control

There are a few different control methods here, each named xxx_control (ex: p4_control)

0. PID Control. This is the classic.
0. P4 Control. This is like PID control, but only using the P, on all four members of the state.
0. Neural Network Control. Trained Via Reinforcement Learning.
