class Waiter {
  private:
    long timestep;
    long marker;

  public:

  Waiter(int ts) {
    timestep = ts;
    marker = millis();
  }

  // wait is used when you want blocking behavior for a loop,
  // such as when testing for an inner loop.
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
  // for a loop, such as when testing for an outer loop.
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
