Want to use a "realistic" Beaker Bot in simulation with the help of the Bullet Physics engine,
and OpenAI Gym? Great!

##### Primary Files

0. beaker.urdf - the Universal Robot Description File. describes the actual model of Beaker.
0. beaker_bot.py - the BeakerBot class that wraps the URDF file.
0. beaker_env.py - the gym environment that makes use of the BeakerBot class
0. client.py - holds your control algorithm

#### Using it

```
$python3 client.py
```