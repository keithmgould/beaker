#ifndef __BEAKER_P4__
#define __BEAKER_P4__

/*
    This class encapsulates a simple control algorithm which I named P4.

    The 'P' comes from 'Proportional' and the '4' comes from the fact that
    there are four parameters for the state (theta, thetaDot, phi, phiDot).

    This algorithm simply takes the state, and for each parameter in the state,
    it multiplies by an associated constant, then adds all terms together:

    command = theta * Ktheta + thetaDot * KthetaDot + phi * Kphi + phiDot * kPhiDot.

    The command could theoretically be simply a voltage, however in our case the 
    output will be a rotational velocity (rads/sec), which will be passed to
    Beaker's Wheels object.
*/

class P4 {
  private:

  // P4 control values
  int kTheta, kThetaDot, kPhi, kPhiDot;

  // presentation strings
  String paramString, termString;

  float kThetaTerm(float theta){
    float results = theta * kTheta;
    termString = String(results, 4);

    return results;
  }

  float kThetaDotTerm(float thetaDot) {
    float results = thetaDot * kThetaDot;
    termString += "," + String(results, 4);

    return results;
  }
  float kPhiTerm(float phi) {
    float results = phi * kPhi;
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
    paramString += "," + String(kPhi);
    paramString += "," + String(kPhiDot);
  }

  public:

  P4(){
    kTheta = kThetaDot = kPhi = kPhiDot = 0;
    buildParamString();
  }

  float getKTheta(){ return kTheta; }
  float getKThetaDot(){ return kThetaDot; }
  float getKPhi(){ return kPhi; }
  float getKPhiDot(){ return kPhiDot; }
  String getParamString() { return paramString; }
  String getTermString() { return termString; }

  void updateParameters(int newKTheta, int newKThetaDot, int newKPhi, int newKPhiDot){
    kTheta = newKTheta;
    kThetaDot = newKThetaDot;
    kPhi = newKPhi;
    kPhiDot = newKPhiDot;
    buildParamString();
  }

  float computeNewRadsPerSec(float theta, float thetaDot, float phi, float phiDot){
    float newRadPerSec = kThetaTerm(theta);
    newRadPerSec += kThetaDotTerm(thetaDot);
    newRadPerSec += kPhiTerm(phi);
    newRadPerSec += kPhiDotTerm(phiDot);

    return newRadPerSec;
  }

};

#endif
