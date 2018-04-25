#ifndef __BEAKER_P4__
#define __BEAKER_P4__

class P4 {
  private:

  // P4 control values
  int kTheta, kThetaDot, kPhi, kPhiDot;

  float kThetaTerm(float theta){ return theta * kTheta * -1; }
  float kThetaDotTerm(float thetaDot) { return thetaDot * kThetaDot * -1; }
  float kPhiTerm(float phi) { return phi * kPhi; }
  float kPhiDotTerm(float phiDot) { return phiDot * kPhiDot; }

  public:

  P4(){
    kTheta = kThetaDot = kPhi = kPhiDot = 0;
  }

  float getKTheta(){ return kTheta; }
  float getKThetaDot(){ return kThetaDot; }
  float getKPhi(){ return kPhi; }
  float getKPhiDot(){ return kPhiDot; }

  String getKString() {
    String ks = String(kTheta);
    ks += "," + String(kThetaDot);
    ks += "," + String(kPhi);
    ks += "," + String(kPhiDot);

    return ks;
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
