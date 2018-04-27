#ifndef __BEAKER_P4__
#define __BEAKER_P4__

class P4 {
  private:

  // P4 control values
  int kTheta, kThetaDot, kPhi, kPhiDot;

  // presentation strings
  String paramString, termString;

  float kThetaTerm(float theta){
    float results = pow(theta,3) * kTheta;
    termString = String(results, 4);

    return results;
  }

  float kThetaDotTerm(float thetaDot) {
    float results = pow(thetaDot,3) * kThetaDot;
    termString += "," + String(results, 4);

    return results;
  }
  float kPhiTerm(float phi) {
    float results = phi * kPhi;
    termString += "," + String(results, 4);

    return results;
  }

  float kPhiDotTerm(float phiDot) {
    float results = pow(phiDot,3) * kPhiDot;
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
