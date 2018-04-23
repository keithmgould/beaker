/*
  Holds the state of Beaker, as determined by the sensors.
*/

class State {
  private:

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

  Wheels* wheels;

  float timestep = 0;   // milliseconds
  float xPos = 0;       // meters
  float lastXPos = 0;   // meters
  float xVel = 0;       // meters / sec
  float phi = 0;        // rads
  float lastPhi = 0;    // rads
  float phiDot = 0;     // rads / sec
  float theta = 0;      // rads
  float lastTheta = 0;  // rads
  float thetaDot = 0;   // rads / sec

  float degToRadians(float deg) {
    return deg * PI_OVER_ONE_EIGHTY;
  }

  // in radians. Average of both wheels
  float rawPhi() {
    return wheels->getPhi();
  }

  // in meters. Average of both wheels
  float rawX() {
    return wheels->getX();
  }

  // in radians
  float rawTheta() {
    imu::Quaternion quat;
    imu::Vector<3> axis;

    quat = bno.getQuat();
    axis = quat.toEuler();
    return axis.z();
  }

  // calculates and stores theta and thetaDot
  // rads and rads/sec
  void updateTheta(float dt){
    theta = rawTheta() + THETA_OFFSET;
    thetaDot = (1000.0 / dt) * (theta - lastTheta) / (dt / timestep);
    lastTheta = theta;
  }

  // calculates and stores x and xDot
  // meters and meters/sec
  void updateX(float dt){
    xPos = rawX();
    xVel = (1000.0 / dt) * (xPos - lastXPos) / (dt / timestep);
    lastXPos = xPos;
  }

  // calculates and stores phi and phiDot
  // rads and rads/sec
  void updatePhi(float dt){
    phi = rawPhi();
    phiDot = (1000.0 / dt) * (phi - lastPhi) / (dt / timestep);
    lastPhi = phi;
  }

  public:

  // constructor
  State(long ts, Wheels* whls) {
    timestep = ts;
    wheels = whls;
  }

  // initializer
  bool initialize(){
    if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) { return false; }
    bno.setExtCrystalUse(true);
    return true;
  }

  // accessors
  float getTheta() { return theta; }
  float getThetaDot() { return thetaDot; }
  float getPhi() { return phi; }
  float getPhiDot() { return phiDot; }
  float getX() { return xPos; }
  float getXDot() { return xVel; }

  void updateState(float dt){
    updateTheta(dt);
    updatePhi(dt);
    updateX(dt);
  }
};
