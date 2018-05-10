#ifndef __BEAKER_P4__
#define __BEAKER_P4__

/*
    This class encapsulates a simple control algorithm which I named P4.

    The 'P' comes from 'Proportional' and the '4' comes from the fact that
    there are four parameters for the state (theta, thetaDot, x-position, phiDot).

    This algorithm simply takes the state, and for each parameter in the state,
    it multiplies by an associated constant, then adds all terms together:

    command = theta * Ktheta + thetaDot * KthetaDot + x * Kx + phiDot * kPhiDot.

    The command could theoretically be simply a voltage, however in our case the 
    output will be a rotational velocity (rads/sec), which will be passed to
    Beaker's Wheels object.
*/

class P4 {
  private:

  // P4 control values
  int kTheta, kThetaDot, kXPos, kPhiDot;

  // presentation strings
  String paramString, termString;

  bool sameSign(float theta, float thetaDot){
    return (theta >= 0 && thetaDot >= 0) || (theta <= 0 && thetaDot <= 0);
  }

  float kThetaTerm(float theta){
    float results = theta * kTheta;
    termString = String(results, 4);

    return results;
  }

  float kThetaDotTerm(float theta, float thetaDot) {
    float results;

    // if(sameSign(theta, thetaDot)){
    //   results = thetaDot * kThetaDot;
    // }else{
    //   results = 0;
    // }
    
    results = thetaDot * kThetaDot;

    termString += "," + String(results, 4);

    return results;
  }
  float kXPosTerm(float xPos) {
    float results = xPos * kXPos;
    termString += "," + String(results, 4);

    return results;
  }

  float kPhiDotTerm(float phiDot) {
    float results = phiDot * kPhiDot;
    termString += "," + String(results, 4);

    return results;
  }

  void buildParamString() {
    paramString = String(kTheta);
    paramString += "," + String(kThetaDot);
    paramString += "," + String(kXPos);
    paramString += "," + String(kPhiDot);
  }

  public:

  P4(){
    kTheta = kThetaDot = kXPos = kPhiDot = 0;
    buildParamString();
  }

  float getKTheta(){ return kTheta; }
  float getKThetaDot(){ return kThetaDot; }
  float getKXPos(){ return kXPos; }
  float getKPhiDot(){ return kPhiDot; }
  String getParamString() { return paramString; }
  String getTermString() { return termString; }

  void updateParameters(int newKTheta, int newKThetaDot, int newKXPos, int newKPhiDot){
    kTheta = newKTheta;
    kThetaDot = newKThetaDot;
    kXPos = newKXPos;
    kPhiDot = newKPhiDot;
    buildParamString();
  }

  float computeNewRadsPerSec(float theta, float thetaDot, float xPos, float phiDot){
    float newRadPerSec = kThetaTerm(theta);
    newRadPerSec += kThetaDotTerm(theta, thetaDot);
    newRadPerSec += kXPosTerm(xPos);
    newRadPerSec += kPhiDotTerm(phiDot);

    return newRadPerSec;
  }

};

#endif
