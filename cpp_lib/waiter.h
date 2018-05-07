#ifndef __BEAKER_WAITER__
#define __BEAKER_WAITER__

/*
    For dynamic robots, having precise loop times is very important. 
    Furthermore, there might be more than one loop in the main loop, such as an outer
    slower loop, and an inner faster loop.  These inner/outer loops facilitate what is 
    commonly called cascading control.

    The Waiter class facilitates easy loop management.
*/

class Waiter {
  private:
    long timestep;
    long marker;

  public:

  Waiter(int ts) {
    timestep = ts;
    marker = millis();
  }

  // wait is used when you want blocking behavior for a loop.
  long wait() {
    long nowish = 0;
    long timeDelta = 0;

    while(true){
      nowish = millis();
      timeDelta = nowish - marker;
      if(timeDelta < timestep){ continue; }
      marker = nowish;
      break;
    }

    return timeDelta;
  }

  // isTime and starting are used when you want non-blocking behavior
  // for a loop.
  bool isTime() {
    long dt = millis() - marker;
    return dt >= timestep;
  }

  long starting() {
    long dt = millis() - marker;
    marker = millis();

    return dt;
  }
};

#endif
