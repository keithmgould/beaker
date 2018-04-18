class Waiter {
  private:
    long timestep;
    long marker;

  public:

  Waiter(int ts) {
    timestep = ts;
    marker = millis();
  }

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
};
