#if !defined(Debounce_h)
#define Devounce_h

class Debounce {
  public:
    Debounce(uint8_t pin) {
      pinMode(pin, INPUT_PULLUP);
      pin_ = pin;
      ontime_ = 0;
      clicked_ = false;
      long_ = false;
      down_ = false;
    }

    uint32_t ontime_;
    uint8_t pin_;
    bool clicked_;
    bool long_;
    bool down_;

    void update() {
      bool down = digitalRead(pin_) == LOW;
      uint32_t now = millis();
      clicked_ = false;
      if (down) {
        if (!down_) {
          down_ = true;
          ontime_ = now;
        }
        if (now - ontime_ > 800) {
          long_ = true;
          clicked_ = true;
          ontime_ = now - 500;
        }
      } else if (down_) {
        down_ = false;
        long_ = false;
        if (now - ontime_ > 25) {
          clicked_ = true;
        }
      }
    }

    bool down() {
      return down_;
    }
    
    bool clicked() {
      return clicked_;
    }

    bool longclick() {
      return long_;
    }
};

#endif  //  Debounce_h

