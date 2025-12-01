// ETHERNET FSM Code

// define and initialize states
enum State {IDLE, RECEIVING, SENDING, JAM, BACKOFF};
State currentState = IDLE;
unsigned long lastTransition = 0;
const unsigned long transitionInterval = 250; // 0.25 seconds

// pulse reading and sending signals
boolean receivedPulseStart = false;
boolean receivedPulseEnd = false;
boolean receivedPulseMessage = false;
boolean receivedPulseJam = false;
int index = 0;
int count = 0;
int countReceived = 0;
boolean done = false;
unsigned long pulseStartTime = 0;
unsigned long measuredPulseDuration = 0;
boolean pulseInProgress = false;
boolean pulseDetected = false;
int sendStep = 0;
unsigned long sendStart = 0;

boolean conflict = false;

int message[] = {5, 6}; // Change this to represent the msg on this Arduino
int msgLength = sizeof(message) / sizeof(message[0]);

// jam signals
boolean jam_start = false;
unsigned long jam_time_start = 0;
unsigned long jam_time_elapsed = 0;

// backoff signals
unsigned long backoff_time_start = 0;
unsigned long backoff_time_elapsed = 0;
boolean backoff_time_completed = false;
boolean backoff_start = false;
float backoff_time = 0;
int backoff_max;
boolean exp_backoff = false;

// wire signals
const int bus = 7;
const int red = 6;
const int green = 13;
const int blue = 12;
const int yellow = 11;

const int segmentPins[7] = {5, 2, 3, 4, 8, 9, 10}; // A–G

// Segment patterns for digits 0–9 (gfedcba)
const byte digitPatterns[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

void setup() {
  lastTransition = millis();
  Serial.begin(9600);

  pinMode(A0, INPUT);   // read bus wire
  pinMode(bus, OUTPUT); // write bus wire
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(yellow, OUTPUT);

  pinMode(A5, OUTPUT); // used for green exp backoff LED

  // initialize LEDs off
  digitalWrite(bus, LOW);
  digitalWrite(red, LOW);
  digitalWrite(green, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(yellow, LOW);

  // initalize seven-segment displays off
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
    digitalWrite(segmentPins[i], HIGH);
  }

  randomSeed(analogRead(A1)); // comment out this line to force exponential backoff

  backoff_max = 10000;

  Serial.println("System initialized.");
}

void loop() {
  unsigned long now = millis();

  // conflict voltage detection
  float busVoltage = (analogRead(A0) / 1023.0) * 5.0;
  conflict = (busVoltage > 2.5);

  // Pulse detection (via analog voltage > 1)
  int state = (busVoltage > 1) ? HIGH : LOW;

  if (!pulseInProgress && state == HIGH) {
    pulseStartTime = micros();
    pulseInProgress = true;
  }

  if (pulseInProgress && state == LOW) {
    measuredPulseDuration = micros() - pulseStartTime;
    pulseInProgress = false;
    pulseDetected = true;
  }

  if (pulseDetected && currentState != SENDING) {
    // detect start pulse
    if (measuredPulseDuration >= 450000 && measuredPulseDuration <= 550000) {
      receivedPulseStart = true;
      receivedPulseEnd = false;
      receivedPulseMessage = false;
      receivedPulseJam = false;
    } 
    // detect end pulse
    else if (measuredPulseDuration >= 250000 && measuredPulseDuration <= 350000) {
      receivedPulseStart = false;
      receivedPulseEnd = true;
      receivedPulseMessage = false;
      receivedPulseJam = false;
    } 
    //  detect msg pulse
    else if (measuredPulseDuration >= 950000 && measuredPulseDuration <= 1050000) {
      receivedPulseStart = false;
      receivedPulseEnd = false;
      receivedPulseMessage = true;
      receivedPulseJam = false;
      countReceived++;
    } 
    // detect jam pulse
    else if (measuredPulseDuration >= 1950000) {
      receivedPulseStart = false;
      receivedPulseEnd = false;
      receivedPulseMessage = false;
      receivedPulseJam = true;
    } 
    // default 
    else {
      receivedPulseStart = false;
      receivedPulseMessage = false;
      receivedPulseJam = false;
    }
    pulseDetected = false;
  }

  // clock period = 250ms
  if (now - lastTransition >= transitionInterval) {
    State nextState = getNextState(currentState);
    currentState = nextState;
    lastTransition = now;
  }

  // FSM output logic
  switch (currentState) {
    case IDLE:
      exp_backoff = false;
      countReceived = 0;
      digitalWrite(blue, LOW);
      break;

    case RECEIVING:
      exp_backoff = false;
      digitalWrite(blue, HIGH);
      if (receivedPulseEnd) {
        if (countReceived != 0) {
          displayDigit(countReceived);
        }
      }
      break;

    case SENDING:
      if (!done) {
        switch (sendStep) {
          case 0: // start pulse
            digitalWrite(bus, HIGH);
            digitalWrite(green, HIGH);
            sendStart = millis();
            sendStep = 1;
            break;

          case 1: // end start pulse
            if (millis() - sendStart >= 500) {
              digitalWrite(bus, LOW);
              digitalWrite(green, LOW);
              sendStart = millis();
              sendStep = 2;
            }
            break;

          case 2: // msg pulse high
            if (millis() - sendStart >= 500) {
              if (count < message[index]) {
                digitalWrite(bus, HIGH);
                digitalWrite(red, HIGH);
                displayDigit(message[index]);
                sendStart = millis();
                sendStep = 3;
              } else {
                sendStep = 5;
              }
            }
            break;

          case 3: // msg pulse low
            if (millis() - sendStart >= 1000) {
              digitalWrite(bus, LOW);
              digitalWrite(red, LOW);
              sendStart = millis();
              count++;
              sendStep = 2;
            }
            break;

          case 5: // end pulse high
            digitalWrite(bus, HIGH);
            digitalWrite(green, HIGH);
            sendStart = millis();
            sendStep = 6;
            break;

          case 6: // end pulse low
            if (millis() - sendStart >= 300) {
              digitalWrite(bus, LOW);
              digitalWrite(green, LOW);
              index++;
              count = 0;
              done = true;
              sendStep = 0;
            }
            break;
        }
      }
      break;

    case JAM: 
      digitalWrite(yellow, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(red, LOW);
      countReceived = 0; // if rec then jam, rst to 0

      if (jam_start) { 
        jam_time_start = millis();
        jam_start = false;
      }
      jam_time_elapsed = millis() - jam_time_start;

      digitalWrite(bus, HIGH);
      break;

    case BACKOFF:
      digitalWrite(yellow, LOW);

      if (backoff_start) {
        backoff_time_start = millis();
        if (exp_backoff) {
          backoff_max = backoff_max * 2;
          Serial.print("exp_backoff");
          digitalWrite(A5, HIGH); // green led on
        }
        Serial.print("backoff_max = ");
        Serial.print(backoff_max);

        backoff_time = random(1000, backoff_max); // random delay
        backoff_start = false;
      }
      backoff_time_elapsed = millis() - backoff_time_start;
      backoff_time_completed = (backoff_time_elapsed > backoff_time);

      digitalWrite(bus, LOW);
      // set to true so that if we backoff before rec again, we enter exp_backoff case
      exp_backoff = true; 
      break;
  }
}

// get next state logic
State getNextState(State current) {
  switch (current) {
    case IDLE:
      if (receivedPulseStart) 
        return RECEIVING;
      // try to send if bus is clear and we have msgs to send still
      else if (!conflict && (index < msgLength)) {
        sendStep = 0;
        return SENDING;
      }
      else return IDLE;

    case RECEIVING:
      if (receivedPulseEnd || receivedPulseJam) 
        return IDLE;
      else 
        return RECEIVING;

    case SENDING:
      if (conflict) {
        done = false;
        jam_start = true;
        return JAM;
      } else if (!conflict && done) {
        done = false;
        return IDLE;
      } else return SENDING;

    case JAM:
      if (jam_time_elapsed >= 1950) { // jam = 2s
        backoff_start = true;
        return BACKOFF;
      } else return JAM;

    case BACKOFF:
      if (receivedPulseStart) {
        return RECEIVING;
      }
      else if (backoff_time_completed && !conflict) {
        sendStep = 0;
        return SENDING;
      }
      else return BACKOFF;
  }
  return IDLE; // fallback
}

// writes the seven segment display
void displayDigit(int num) {
  byte pattern = digitPatterns[num];
  for (int i = 0; i < 7; i++) {
    // display each segment
    bool bit_on = (pattern >> i) & 0x01;
    digitalWrite(segmentPins[i], bit_on ? LOW : HIGH); 
  }
}
