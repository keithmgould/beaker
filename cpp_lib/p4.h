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

  float maxRadPerSec = 15;

  // P4 control values
  float kTheta, kThetaDot, kXPos, kPhiDot;

  // presentation strings
  String paramString, termString;

  float thetaDotAccumulator;

  bool sameSign(float theta, float thetaDot){
    return (theta >= 0 && thetaDot >= 0) || (theta < 0 && thetaDot < 0);
  }

  float kThetaTerm(float theta){
    float results;
    float fabsTheta = fabs(theta);

    if(fabsTheta > 0.025){
      results = theta * kTheta * 4;
    }else if(fabsTheta > 0.02){
      results = theta * kTheta * 3;
    }else if(fabsTheta > 0.01){
      results = theta * kTheta * 2;
    }else if(fabsTheta > 0.005){
      results = theta * kTheta * 1;
    }else{
      results = 0;
    }

    termString = String(results, 4);
    return results;
  }

  float kThetaDotTerm(float thetaDot, float theta) {
    // float results;
    // float fabsThetaDot = fabs(thetaDot);

    if(sameSign(theta, thetaDot)){
      // results = thetaDot * kThetaDot;
      thetaDotAccumulator += thetaDot;
    }else{
      // results = 0;
    }


    thetaDotAccumulator = constrain(thetaDotAccumulator, -1, 1);
    float results = thetaDotAccumulator * kThetaDot;



    termString += "," + String(results, 4);

    return results;
  }




  float kXPosTerm(float xPos) {
    float results = xPos * kXPos;
    termString += "," + String(results, 4);

    return results;
  }

  float kPhiDotTerm(float phiDot) {
    float results;
    if(fabs(phiDot) > 1){
      results = phiDot * kPhiDot;
    } else{
      results = 0;
    }

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
    kTheta = kThetaDot = kXPos = kPhiDot = thetaDotAccumulator = 0;
    buildParamString();
  }

  float getKTheta(){ return kTheta; }
  float getKThetaDot(){ return kThetaDot; }
  float getKXPos(){ return kXPos; }
  float getKPhiDot(){ return kPhiDot; }
  String getParamString() { return paramString; }
  String getTermString() { return termString; }

  void updateParameters(float newKTheta, float newKThetaDot, float newKXPos, float newKPhiDot){
    kTheta = newKTheta;
    kThetaDot = newKThetaDot;
    kXPos = newKXPos;
    kPhiDot = newKPhiDot;
    buildParamString();
  }

  float computeNewRadsPerSec(float theta, float thetaDot, float xPos, float phiDot){
    float newRadPerSec = kThetaTerm(theta);
    newRadPerSec += kThetaDotTerm(thetaDot, theta);
    newRadPerSec += kXPosTerm(xPos);
    newRadPerSec += kPhiDotTerm(phiDot);

    newRadPerSec = constrain(newRadPerSec, -maxRadPerSec, maxRadPerSec);

    return newRadPerSec;
  }

};

#endif
