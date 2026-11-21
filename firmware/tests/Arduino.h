#pragma once

// Host-only Arduino shim: no USB, GPIO, motor or board access.
// Host integer widths/timing do not emulate an AVR or certify hardware safety.
#include <array>
#include <cstdint>
#include <deque>
#include <sstream>
#include <vector>

#define F(value) value
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define constrain(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

struct MockWrite {
  uint8_t pin;
  int value;
};

inline unsigned long mockMillis = 0;
inline std::array<int, 32> mockOutputs{};
inline std::array<int, 32> mockInputs{};
inline std::vector<MockWrite> mockWrites;
inline std::vector<std::array<int, 32>> mockReversalSnapshots;
inline std::vector<unsigned int> mockDelays;

inline void analogWrite(uint8_t pin, int value) {
  mockOutputs.at(pin) = value;
  mockWrites.push_back({pin, value});
}
inline void digitalWrite(uint8_t pin, int value) {
  mockOutputs.at(pin) = value;
  mockWrites.push_back({pin, value});
}
inline int digitalRead(uint8_t pin) {
  return mockInputs.at(pin);
}
inline unsigned long millis() {
  return mockMillis;
}
inline unsigned long mockMicros = 0;
inline unsigned long micros() {
  return mockMicros;
}
inline void pinMode(uint8_t, int) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(uint8_t pin) {
  return pin;
}
inline void attachInterrupt(int, void (*)(), int) {}
inline void delayMicroseconds(unsigned int duration) {
  mockDelays.push_back(duration);
  mockReversalSnapshots.push_back(mockOutputs);
}

struct MockSerial {
  std::ostringstream output;
  std::deque<char> input;
  void begin(unsigned long) {}
  int available() {
    return static_cast<int>(input.size());
  }
  char read() {
    char next = input.front();
    input.pop_front();
    return next;
  }
  template <class T>
  void print(T value) {
    output << value;
  }
  template <class T>
  void println(T value) {
    output << value << '\n';
  }
  void clearOutput() {
    output.str("");
    output.clear();
  }
};

inline MockSerial Serial;
