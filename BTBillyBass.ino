/*This is my crack at a state-based approach to automating a Big Mouth Billy Bass.
 This code was built on work done by both Donald Bell and github user jswett77. 
 See links below for more information on their previous work.

 In this code you'll find reference to the MX1508 library, which is a simple 
 library I wrote to interface with the extremely cheap 2-channel H-bridges that
 use the MX1508 driver chip. It may also work with other H-bridges that use different
 chips (such as the L298N), so long as you can PWM the inputs.

 This code watches for a voltage increase on input A0, and when sound rises above a
 set threshold it opens the mouth of the fish. When the voltage falls below the threshold,
 the mouth closes.The result is the appearance of the mouth "riding the wave" of audio
 amplitude, and reacting to each voltage spike by opening again. There is also some code
 which adds body movements for a bit more personality while talking.

 Most of this work was based on the code written by jswett77, and can be found here:
 https://github.com/jswett77/big_mouth/blob/master/billy.ino

 Donald Bell wrote the initial code for getting a Billy Bass to react to audio input,
 and his project can be found on Instructables here:
 https://www.instructables.com/id/Animate-a-Billy-Bass-Mouth-With-Any-Audio-Source/

 Author: Jordan Bunker <jordan@hierotechnics.com> 2019
 License: MIT License (https://opensource.org/licenses/MIT)
*/

#include <MX1508.h>

// --- Pin assignments (Nano) ---
constexpr uint8_t kPinBodyMotorA = 6;
constexpr uint8_t kPinBodyMotorB = 9;
constexpr uint8_t kPinMouthMotorA = 3;
constexpr uint8_t kPinMouthMotorB = 5;
constexpr uint8_t kPinSoundAnalog = A0;
// LED: D2 -> 220–330 ohm -> LED -> GND
constexpr uint8_t kPinMouthLed = 2;

// --- Serial ---
constexpr long kSerialBaud = 9600;

// --- Audio (analogRead 0..1023) ---
constexpr int kSilenceThreshold = 250;
constexpr int kPrevSoundVolumeUnset = -1;

// --- Mouth motor PWM speeds (0..255) ---
constexpr int kMouthOpenSpeed = 220;
constexpr int kMouthCloseSpeed = 180;

// --- Body motor PWM speeds (0..255) ---
constexpr int kBodyMotorSpeedStop = 0;
constexpr int kBodyMotorSpeedSlow = 150;
constexpr int kBodyMotorSpeedMedium = 200;
constexpr int kBodyMotorSpeedFull = 255;
constexpr int kFlapBodySpeed = 180;

// --- Timing (ms) ---
constexpr unsigned long kMouthLedBlinkMs = 120;
constexpr unsigned long kMouthTalkPhaseMs = 100;
constexpr unsigned long kWaitStateMotorHaltAfterMouthMs = 100;
constexpr unsigned long kBoredIdleMs = 1500;
constexpr unsigned long kFlapHoldMs = 500;
constexpr unsigned long kBodyActionScheduleMinMs = 500;
constexpr unsigned long kBodyActionScheduleMaxMs = 1000;
constexpr unsigned long kBodyTailActionMinMs = 900;
constexpr unsigned long kBodyTailActionMaxMs = 1200;
constexpr unsigned long kBodyLongActionMinMs = 1500;
constexpr unsigned long kBodyLongActionMaxMs = 3000;
constexpr unsigned long kBodyRestMinMs = 20;
constexpr unsigned long kBodyRestMaxMs = 50;

// --- Random ranges ---
constexpr int kBodyArticulationRandomMax = 8;
constexpr long kFlapCooldownMinS = 30;
constexpr long kFlapCooldownMaxS = 60;
constexpr long kMsPerSecond = 1000L;

enum FishState : uint8_t {
  kFishStateWait = 0,
  kFishStateTalking = 1,
  kFishStateFlap = 2
};

MX1508 bodyMotor(kPinBodyMotorA, kPinBodyMotorB);
MX1508 mouthMotor(kPinMouthMotorA, kPinMouthMotorB);

int soundPin = kPinSoundAnalog;

unsigned long mouthLedLastToggle = 0;
bool mouthLedOn = false;

int bodySpeed = kBodyMotorSpeedStop;
int soundVolume = 0;
int prevSoundVolume = kPrevSoundVolumeUnset;
FishState fishState = kFishStateWait;

bool talking = false;

long currentTime;
long mouthActionTime;
long bodyActionTime;
long lastActionTime;

enum MotorMode : uint8_t { M_HALT = 0, M_FWD = 1, M_REV = 2 };

MotorMode bodyModeLog = M_HALT;
MotorMode mouthModeLog = M_HALT;

void logMotorMode(const __FlashStringHelper* tag, MotorMode& prev, MotorMode next) {
  if (prev != next) {
    prev = next;
    Serial.print(tag);
    Serial.print(' ');
    if (next == M_HALT) {
      Serial.println(F("halt"));
    } else if (next == M_FWD) {
      Serial.println(F("forward"));
    } else {
      Serial.println(F("backward"));
    }
  }
}

void bodyHalt() {
  bodyMotor.halt();
  logMotorMode(F("body"), bodyModeLog, M_HALT);
}
void bodyFwd() {
  bodyMotor.forward();
  logMotorMode(F("body"), bodyModeLog, M_FWD);
}
void bodyRev() {
  bodyMotor.backward();
  logMotorMode(F("body"), bodyModeLog, M_REV);
}
void mouthHalt() {
  mouthMotor.halt();
  logMotorMode(F("mouth"), mouthModeLog, M_HALT);
}
void mouthFwd() {
  mouthMotor.forward();
  logMotorMode(F("mouth"), mouthModeLog, M_FWD);
}
void mouthRev() {
  mouthMotor.backward();
  logMotorMode(F("mouth"), mouthModeLog, M_REV);
}

void setup() {
  bodyMotor.setSpeed(kBodyMotorSpeedStop);
  mouthMotor.setSpeed(kBodyMotorSpeedStop);

  pinMode(soundPin, INPUT);

  pinMode(kPinMouthLed, OUTPUT);
  digitalWrite(kPinMouthLed, LOW);

  Serial.begin(kSerialBaud);
}

void loop() {
  currentTime = millis();
  updateSoundInput();
  SMBillyBass();
  updateMouthLed();
}

void updateMouthLed() {
  if (fishState == kFishStateTalking) {
    if (currentTime - mouthLedLastToggle >= kMouthLedBlinkMs) {
      mouthLedLastToggle = currentTime;
      mouthLedOn = !mouthLedOn;
      digitalWrite(kPinMouthLed, mouthLedOn ? HIGH : LOW);
    }
  } else {
    if (mouthLedOn) {
      mouthLedOn = false;
      digitalWrite(kPinMouthLed, LOW);
    }
  }
}

void SMBillyBass() {
  switch (fishState) {
    case kFishStateWait:
      if (soundVolume > kSilenceThreshold) {
        if (currentTime > mouthActionTime) {
          talking = true;
          mouthActionTime = currentTime + kMouthTalkPhaseMs;
          fishState = kFishStateTalking;
        }
      } else if (currentTime > mouthActionTime + kWaitStateMotorHaltAfterMouthMs) {
        bodyHalt();
        mouthHalt();
      }
      if (currentTime - lastActionTime > kBoredIdleMs) {
        lastActionTime = currentTime + floor(random(kFlapCooldownMinS, kFlapCooldownMaxS)) * kMsPerSecond;
        fishState = kFishStateFlap;
      }
      break;

    case kFishStateTalking:
      if (currentTime < mouthActionTime) {
        if (talking) {
          openMouth();
          lastActionTime = currentTime;
          articulateBody(true);
        }
      }
      else {
        closeMouth();
        articulateBody(false);
        talking = false;
        fishState = kFishStateWait;
      }
      break;

    case kFishStateFlap:
      flap();
      fishState = kFishStateWait;
      break;
  }
}

void updateSoundInput() {
  soundVolume = analogRead(soundPin);
  if (soundVolume != prevSoundVolume) {
    prevSoundVolume = soundVolume;
    Serial.println(soundVolume);
  }
}

void openMouth() {
  mouthMotor.halt();
  mouthMotor.setSpeed(kMouthOpenSpeed);
  mouthMotor.forward();
  logMotorMode(F("mouth"), mouthModeLog, M_FWD);
}

void closeMouth() {
  mouthMotor.halt();
  mouthMotor.setSpeed(kMouthCloseSpeed);
  mouthMotor.backward();
  logMotorMode(F("mouth"), mouthModeLog, M_REV);
}

void articulateBody(bool talking) {
  if (talking) {
    if (currentTime > bodyActionTime) {
      int r = floor(random(0, kBodyArticulationRandomMax));
      if (r < 1) {
        bodySpeed = kBodyMotorSpeedStop;
        bodyActionTime = currentTime + floor(random(kBodyActionScheduleMinMs, kBodyActionScheduleMaxMs));
        bodyFwd();

      } else if (r < 3) {
        bodySpeed = kBodyMotorSpeedSlow;
        bodyActionTime = currentTime + floor(random(kBodyActionScheduleMinMs, kBodyActionScheduleMaxMs));
        bodyFwd();

      } else if (r == 4) {
        bodySpeed = kBodyMotorSpeedMedium;
        bodyActionTime = currentTime + floor(random(kBodyActionScheduleMinMs, kBodyActionScheduleMaxMs));
        bodyFwd();

      } else if (r == 5) {
        bodySpeed = kBodyMotorSpeedStop;
        bodyHalt();
        bodyMotor.setSpeed(kBodyMotorSpeedFull);
        bodyRev();
        bodyActionTime = currentTime + floor(random(kBodyTailActionMinMs, kBodyTailActionMaxMs));
      }
      else {
        bodySpeed = kBodyMotorSpeedFull;
        bodyFwd();
        bodyActionTime = currentTime + floor(random(kBodyLongActionMinMs, kBodyLongActionMaxMs));
      }
    }

    bodyMotor.setSpeed(bodySpeed);
  } else {
    if (currentTime > bodyActionTime) {
      bodyHalt();
      bodyActionTime = currentTime + floor(random(kBodyRestMinMs, kBodyRestMaxMs));
    }
  }
}


void flap() {
  bodyMotor.setSpeed(kFlapBodySpeed);
  bodyRev();
  delay(kFlapHoldMs);
  bodyHalt();
}
