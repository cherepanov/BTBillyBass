/*This is my crack at a state-based approach to automating a Big Mouth Billy Bass.
 This code was built on work done by both Donald Bell and github user jswett77. 
 See links below for more information on their previous work.

 In this code you'll find reference to the MX1508 library, which is a simple 
 library I wrote to interface with the extremely cheap 2-channel H-bridges that
 use the MX1508 driver chip. It may also work with other H-bridges that use different
 chips (such as the L298N), so long as you can PWM the inputs.

 This code reads stereo audio on A0 (left) and A1 (right), mixes them with equal weights
 (adjustable) for detection. Detection uses a speech-weighted band (~300 Hz–3 kHz) plus an
 envelope follower so hum and deep bass do not drive the mouth. When the level rises above
 a threshold it opens the mouth; when it falls below, the mouth closes.
 The result is the appearance of the mouth "riding the wave" of audio amplitude, and
 reacting to each spike by opening again. There is also some code which adds body
 movements for a bit more personality while talking.

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
constexpr uint8_t kPinMouthMotorA = 3; //5
constexpr uint8_t kPinMouthMotorB = 5; //3
constexpr uint8_t kPinSoundAnalogL = A0;
constexpr uint8_t kPinSoundAnalogR = A1;
// LED: D2 -> 220–330 ohm -> LED -> GND
constexpr uint8_t kPinMouthLed = 2;
// SPST toggle: one leg to GND, other to this pin. LOW = motors disabled (pull-up inside).
constexpr uint8_t kPinMotorDisableSwitch = 4;

// --- Serial ---
constexpr long kSerialBaud = 9600;

// --- Audio (analogRead 0..1023) ---
// Speech-band envelope is scaled to 0..1023; typical silence/speech split is far below
// raw ADC thresholds—start ~60–120 and tune (lower = more sensitive).
constexpr int kSilenceThreshold = 85;
constexpr int kPrevSoundVolumeUnset = -1;
// Mix L/R for detection: combined = (L * wL + R * wR) / (wL + wR). Use 1,1 for equal
// average; increase one weight if that channel is consistently quieter in hardware.
constexpr int kAudioChannelWeightL = 1;
constexpr int kAudioChannelWeightR = 1;

// Speech-band path: sample period sets effective Fs (~1e6 / us); coefficients match
// ~4.8 kHz (two analogReads per tick). If you change kSpeechSamplePeriodUs, retune alphas.
constexpr unsigned long kSpeechSamplePeriodUs = 208;
// One-pole high-pass ~300 Hz (removes hum/fundamentals); low-pass ~3 kHz (limits hash).
constexpr float kSpeechHpAlpha = 0.718f;
constexpr float kSpeechLpAlpha = 0.797f;
// Envelope smoother ~60 Hz; higher alpha = faster mouth tracking, more ripple.
constexpr float kSpeechEnvAlpha = 0.073f;
constexpr float kDcBlockR = 0.992f;
// Post-envelope gain into 0..1023; lower if the mouth is too eager, raise if it ignores speech.
constexpr float kSpeechEnvGain = 18.f;

// --- Mouth motor PWM speeds (0..255) ---
constexpr int kMouthOpenSpeed = 220;
constexpr int kMouthCloseSpeed = 180;

// --- Body motor PWM speeds (0..255) ---
constexpr int kBodyMotorSpeedStop = 0;
constexpr int kBodyMotorSpeedSlow = 115;
constexpr int kBodyMotorSpeedMedium = 155;
constexpr int kBodyMotorSpeedFull = 195;
constexpr int kFlapBodySpeed = 135;
// --- Timing (ms) ---
// Shorter = return to wait sooner so the next open can trigger more often on changing level.
constexpr unsigned long kMouthTalkPhaseMs =  170;
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

bool mouthLedOn = false;

int bodySpeed = kBodyMotorSpeedStop;
int soundVolume = 0;
int prevSoundVolume = kPrevSoundVolumeUnset;
FishState fishState = kFishStateWait;

// Speech-band filter state (updateSoundInput)
static unsigned long sNextSpeechSampleUs = 0;
static bool sSpeechFilterPrimed = false;
static float sDcXm1 = 0, sDcYm1 = 0;
static float sHpXm1 = 0, sHpYm1 = 0;
static float sLpY = 0;
static float sEnv = 0;

bool talking = false;

long currentTime;
long mouthActionTime;
long bodyActionTime;
long lastActionTime;

enum MotorMode : uint8_t { M_HALT = 0, M_FWD = 1, M_REV = 2 };

MotorMode bodyModeLog = M_HALT;
MotorMode mouthModeLog = M_HALT;

bool motorsMovementEnabled() {
  return digitalRead(kPinMotorDisableSwitch) == HIGH;
}

// Tracks kPinMotorDisableSwitch so we can re-arm timers and refresh drivers when
// movement is turned back on (avoids getting stuck with stale mouth/body times
// and leaves MX1508 outputs in a known state after many halt() cycles).
static bool sPrevMotorsMovementEnabled = true;

static void onMotorsMovementJustEnabled() {
  mouthActionTime = currentTime - 1;
  bodyActionTime = currentTime - 1;
  // Avoid treating "never updated" as ages-old idle (lastActionTime was 0), which
  // forced kFishStateFlap on the same pass as entering kFishStateTalking.
  lastActionTime = currentTime;
}

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
  if (!motorsMovementEnabled()) {
    return;
  }
  bodyMotor.forward();
  logMotorMode(F("body"), bodyModeLog, M_FWD);
}
void bodyRev() {
  if (!motorsMovementEnabled()) {
    return;
  }
  bodyMotor.backward();
  logMotorMode(F("body"), bodyModeLog, M_REV);
}
void mouthHalt() {
  mouthMotor.halt();
  logMotorMode(F("mouth"), mouthModeLog, M_HALT);
}
void mouthFwd() {
  if (!motorsMovementEnabled()) {
    return;
  }
  mouthMotor.forward();
  logMotorMode(F("mouth"), mouthModeLog, M_FWD);
}
void mouthRev() {
  if (!motorsMovementEnabled()) {
    return;
  }
  mouthMotor.backward();
  logMotorMode(F("mouth"), mouthModeLog, M_REV);
}

void setup() {
  bodyMotor.setSpeed(kBodyMotorSpeedStop);
  mouthMotor.setSpeed(kBodyMotorSpeedStop);

  pinMode(kPinSoundAnalogL, INPUT);
  pinMode(kPinSoundAnalogR, INPUT);

  pinMode(kPinMouthLed, OUTPUT);
  digitalWrite(kPinMouthLed, LOW);

  pinMode(kPinMotorDisableSwitch, INPUT_PULLUP);
  sPrevMotorsMovementEnabled = motorsMovementEnabled();
  lastActionTime = millis();

  Serial.begin(kSerialBaud);
}

void loop() {
  currentTime = millis();
  updateSoundInput();
  SMBillyBass();
  updateMouthLed();
}

void updateMouthLed() {
  const bool inputActive = soundVolume > kSilenceThreshold;
  if (mouthLedOn != inputActive) {
    mouthLedOn = inputActive;
    digitalWrite(kPinMouthLed, mouthLedOn ? HIGH : LOW);
  }
}

void SMBillyBass() {
  const bool movementEnabled = motorsMovementEnabled();
  if (!movementEnabled) {
    bodyHalt();
    mouthHalt();
    fishState = kFishStateWait;
    talking = false;
    sPrevMotorsMovementEnabled = false;
    return;
  }
  if (!sPrevMotorsMovementEnabled) {
    sPrevMotorsMovementEnabled = true;
    onMotorsMovementJustEnabled();
  }

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
      // Do not run bored-flap in the same pass that transitioned to talking (would
      // overwrite fishState while lastActionTime was still at its initial value).
      if (fishState == kFishStateWait && currentTime - lastActionTime > kBoredIdleMs) {
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

// One new mixed sample (0..1023); updates sEnv and maps to soundVolume.
static void processSpeechBandSample(int mixed) {
  const float x = (float)mixed - 512.f;

  if (!sSpeechFilterPrimed) {
    sDcXm1 = x;
    sDcYm1 = 0.f;
    sHpXm1 = x;
    sHpYm1 = 0.f;
    sLpY = 0.f;
    sEnv = 0.f;
    sSpeechFilterPrimed = true;
  }

  const float dc = x - sDcXm1 + kDcBlockR * sDcYm1;
  sDcXm1 = x;
  sDcYm1 = dc;

  const float hp = kSpeechHpAlpha * (sHpYm1 + dc - sHpXm1);
  sHpXm1 = dc;
  sHpYm1 = hp;

  sLpY += kSpeechLpAlpha * (hp - sLpY);

  const float rect = fabsf(sLpY);
  sEnv += kSpeechEnvAlpha * (rect - sEnv);

  const float scaled = sEnv * kSpeechEnvGain;
  long v = (long)(scaled + 0.5f);
  if (v < 0) {
    v = 0;
  } else if (v > 1023) {
    v = 1023;
  }
  soundVolume = (int)v;
}

void updateSoundInput() {
  const unsigned long now = micros();
  if (sNextSpeechSampleUs == 0) {
    sNextSpeechSampleUs = now;
  }

  static int sDbgLeft = 0;
  static int sDbgRight = 0;

  if ((unsigned long)(now - sNextSpeechSampleUs) >= kSpeechSamplePeriodUs) {
    sNextSpeechSampleUs += kSpeechSamplePeriodUs;
    if ((unsigned long)(now - sNextSpeechSampleUs) >= kSpeechSamplePeriodUs * 4) {
      sNextSpeechSampleUs = now;
    }
    sDbgLeft = analogRead(kPinSoundAnalogL);
    sDbgRight = analogRead(kPinSoundAnalogR);
    const int wSum = kAudioChannelWeightL + kAudioChannelWeightR;
    const int mixed =
        (sDbgLeft * kAudioChannelWeightL + sDbgRight * kAudioChannelWeightR) / wSum;
    processSpeechBandSample(mixed);
  }

  if (soundVolume != prevSoundVolume) {
    prevSoundVolume = soundVolume;
    Serial.print(sDbgLeft);
    Serial.print(' ');
    Serial.print(sDbgRight);
    Serial.print(' ');
    Serial.println(soundVolume);
  }
}

void openMouth() {
  if (!motorsMovementEnabled()) {
    return;
  }
  mouthMotor.halt();
  mouthMotor.setSpeed(kMouthOpenSpeed);
  mouthMotor.forward();
  logMotorMode(F("mouth"), mouthModeLog, M_FWD);
}

void closeMouth() {
  if (!motorsMovementEnabled()) {
    return;
  }
  mouthMotor.halt();
  mouthMotor.setSpeed(kMouthCloseSpeed);
  mouthMotor.backward();
  logMotorMode(F("mouth"), mouthModeLog, M_REV);
}

void articulateBody(bool talking) {
  if (!motorsMovementEnabled()) {
    return;
  }
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

      } else if (r < 6) {
        bodySpeed = kBodyMotorSpeedMedium;
        bodyActionTime = currentTime + floor(random(kBodyLongActionMinMs, kBodyLongActionMaxMs));
        bodyFwd();

      } else {
        bodySpeed = kBodyMotorSpeedStop;
        bodyHalt();
        bodyMotor.setSpeed(kBodyMotorSpeedFull);
        bodyRev();
        bodyActionTime = currentTime + floor(random(kBodyTailActionMinMs, kBodyTailActionMaxMs));
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
  if (!motorsMovementEnabled()) {
    return;
  }
  bodyMotor.setSpeed(kFlapBodySpeed);
  bodyRev();
  delay(kFlapHoldMs);
  bodyHalt();
}
