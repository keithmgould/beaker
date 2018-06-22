# Classic Control

Taking the below concept and applying PID to each of the states, rather than just P on each state.

In short, given four element vector that defines the state:

theta, thetaDot, phi, phiDot,

We can apply a PID controller to each of these elements, and sum them together:

outTheta = thetaPid(theta)
outThetaDot = thetaDotPid(theta)
outphi = phiPid(phi)
outphiDot = phiDotPid(phiDot)

then final command is a sum of these:

outTheta + outThetaDot + outPhi + outPhiDot


This command is interpreted as an acceleration, aka a change in desired wheel velocity.


# Original P4 Concept

http://www.geology.smu.edu/~dpa-www/robo/nbot/bal2.txt

David Anderson wrote the following:

---

The Algorithm for two-wheel balancing robot nBot.

The balancing algorithm measures two outputs from the
robot and calculates the torque forces from the motors
needed for balance.  Here is an ascii-art block diagram:


               +---------------------+--------------+
    +--------< | Motor shaft encoder |              |
    |          +---------------------+   Motor PWM  | <-----+
    |    +---< | Angle sensor        |              |       |
    |    |     +---------------------+--------------+       |
    |    |                                                  |
    |    |     +-------+      |\ angle                      |
    |    |     |       |      | \ velocity                  |
    |    +-----| Deriv |------|K1-----+                     |
    |    |     |       |      | /      \                    |
    |    |     +-------+      |/        \                   |
    |    |                               +-----+            |
    |    |                    |\ angle   |      \           |
    |    |                    | \ pos    |       \          |
    |    +--------------------|K2--------+        \         |
    |                         | /        |         \        |
    |                         |/         |          \       |
    |                                    |  SUM      +------+
    |          +-------+      |\ wheel   |          /
    |          |       |      | \velocity|         /
    +----------| Deriv |------|K3--------+        /
    |          |       |      | /        |       /
    |          +-------+      |/         |      /
    |                                    +-----+
    |                         |\ wheel  /
    |                         | \ pos  /
    +-------------------------|K4-----+
                              | /
                              |/


The boxes labled "Deriv" calculate the derivative of the inputs
by subtracting the last sample from the current sample.  For the
shaft encoders this gives the wheel velocity, and for the angle
sensor this gives the angle velocity.  The four triangles labeled
K1 through K4 are the "knobs" that apply gain to the four feedback
signals.  They are then summed together and fed back to the robot
as the PWM motor voltage. 
