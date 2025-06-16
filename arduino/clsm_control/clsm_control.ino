#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

// For galvo scanner (x–y control)
Adafruit_MCP4725 dac;      // x-axis via MCP4725 DAC
const int pwmPin = A0;     // y-axis via PWM

// For stepper motor (z control)
const int STEP_PIN = 6;    // STEP signal
const int DIR_PIN  = 7;    // DIR signal

// For photodiode measurement (autofocus)
const int PD_PIN = A1;     // Photodiode input
const uint16_t PD_SAMPLE_COUNT = 4; // samples to average

// Ramp parameters
const uint16_t RAMP_DEC_US     = 10;   // subtract 10 µs per ramp step
const uint16_t RAMP_START_FREQ = 480;  // start ramp at 480 Hz

// ─────────────────────────────────────────────────────────────────────────────

// Pick a safe upper bound on the number of (z,PD) pairs any scan can return.
// For example, ±1500 with inc=5 is 601 points. If you expect up to ±2000 with
// inc=1, you’d need 4001 points; so adjust MAX_POINTS accordingly. Here:
#define MAX_POINTS 3000

// One global buffer (shared by Z-scan and Autofocus)
static int16_t   z_buffer[MAX_POINTS];
static uint16_t pd_buffer[MAX_POINTS];

// ─────────────────────────────────────────────────────────────────────────────
// For galvo-scan, we need a per‐column buffer.  Using up to 4096 points per column
// ⇒ each point is 6 bytes (x:int16, y:int16, pd:uint16) ⇒ buffer = 4096×6 = 24576 bytes.
// On an R4 Minima (32 KB SRAM), that leaves ~8 KB for everything else.
// If the user requests >4096 points in a column, we’ll abort that scan (header=0).
#define MAX_COLUMN_POINTS 1001
static uint8_t columnBuf[MAX_COLUMN_POINTS * 2];
// ─────────────────────────────────────────────────────────────────────────────

// Track current stepper position (in raw units)
static int16_t current_z = 0;

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(1000000);

  // Galvo outputs
  analogWriteResolution(12);
  pinMode(pwmPin, OUTPUT);
  Wire.setClock(1000000UL); // 1 MHz
  dac.begin(0b01100000);

  // Stepper outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Photodiode ADC
  analogReadResolution(14);

}

void loop() {
  if (Serial.available() < 1) return;

  // Peek the first byte (control)
  byte ctrlByte = Serial.peek();
  uint8_t top2 = (ctrlByte & 0b11000000);

  // ────────────────────────────────────────────────────────────────────────────
  // Case 1: top2 == 0b10000000 → Galvo “point command” (existing code)
  if (top2 == 0b10000000) {
    if (Serial.available() >= 4) {
      byte pkt[4];
      Serial.readBytes(pkt, 4);
      processGalvoCommand(pkt);
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 2: top2 == 0b11000000 → Stepper single-move (existing code)
  else if (top2 == 0b11000000) {
    if (Serial.available() >= 4) {
      byte pkt[4];
      Serial.readBytes(pkt, 4);
      processStepperCommand(pkt);
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 3: ctrlByte == 0x40 → Z‐scan (existing, relative‐mode code)
  else if (ctrlByte == 0x40) {
    if (Serial.available() >= 9) {
      Serial.read();  // consume 0x40
      byte buf[8];
      Serial.readBytes(buf, 8);
      int16_t neg_lim   = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t pos_lim   = (int16_t)((buf[2] << 8) | buf[3]);
      uint16_t inc      = (uint16_t)((buf[4] << 8) | buf[5]);
      uint16_t freq_fld = (uint16_t)((buf[6] << 8) | buf[7]);
      processZScanCommand(neg_lim, pos_lim, inc, freq_fld);
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 4: ctrlByte == 0x41 → Autofocus (existing, relative‐mode code)
  else if (ctrlByte == 0x41) {
    if (Serial.available() >= 9) {
      Serial.read();  // consume 0x41
      byte buf[8];
      Serial.readBytes(buf, 8);
      int16_t neg_lim   = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t pos_lim   = (int16_t)((buf[2] << 8) | buf[3]);
      uint16_t inc      = (uint16_t)((buf[4] << 8) | buf[5]);
      uint16_t freq_fld = (uint16_t)((buf[6] << 8) | buf[7]);
      processAutofocusCommand(neg_lim, pos_lim, inc, freq_fld);
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 5: ctrlByte == 0x42 → Galvo‐Scan raster 
  else if (ctrlByte == 0x42) {
    // We expect 11 bytes: [ 0x42 | x_neg:int16 | x_pos:int16 | y_neg:int16 | y_pos:int16 | inc:uint16 ]
    if (Serial.available() >= 11) {
      Serial.read();  // consume 0x42
      byte buf[10];
      Serial.readBytes(buf, 10);
      int16_t x_neg  = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t x_pos  = (int16_t)((buf[2] << 8) | buf[3]);
      int16_t y_neg  = (int16_t)((buf[4] << 8) | buf[5]);
      int16_t y_pos  = (int16_t)((buf[6] << 8) | buf[7]);
      uint16_t inc   = (uint16_t)((buf[8] << 8) | buf[9]);
      processGalvoScanCommand(x_neg, x_pos, y_neg, y_pos, inc);
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 6: ctrlByte == 0x43 → 3D‐Layered Scan 
  else if (ctrlByte == 0x43) {
    // We expect 1 (control) + 18 bytes payload = total 19 bytes
    if (Serial.available() >= 19) {
      Serial.read();  // consume 0x43
      byte buf[18];
      Serial.readBytes(buf, 18);
      // Unpack exactly as: >hhhhHhhHH
      int16_t x_neg   = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t x_pos   = (int16_t)((buf[2] << 8) | buf[3]);
      int16_t y_neg   = (int16_t)((buf[4] << 8) | buf[5]);
      int16_t y_pos   = (int16_t)((buf[6] << 8) | buf[7]);
      uint16_t xy_inc = (uint16_t)((buf[8] << 8) | buf[9]);
      int16_t z_neg   = (int16_t)((buf[10] << 8) | buf[11]);
      int16_t z_pos   = (int16_t)((buf[12] << 8) | buf[13]);
      uint16_t z_inc  = (uint16_t)((buf[14] << 8) | buf[15]);
      uint16_t freq_fld = (uint16_t)((buf[16] << 8) | buf[17]);

      process3DLayerScanCommand(
        x_neg, x_pos, y_neg, y_pos,
        xy_inc, z_neg, z_pos, z_inc,
        freq_fld
      );
    }
  }
  // ────────────────────────────────────────────────────────────────────────────
  // Case 7: ctrlByte == 0x44  → Show-Area outline
  else if (ctrlByte == 0x44) {
    if (Serial.available() >= 13) {
      Serial.read();           // consume 0x44
      byte buf[12];
      Serial.readBytes(buf, 12);

      int16_t x_neg   = (int16_t)((buf[0] << 8) | buf[1]);
      int16_t x_pos   = (int16_t)((buf[2] << 8) | buf[3]);
      int16_t y_neg   = (int16_t)((buf[4] << 8) | buf[5]);
      int16_t y_pos   = (int16_t)((buf[6] << 8) | buf[7]);
      uint16_t inc    = (uint16_t)((buf[8] << 8) | buf[9]);
      uint16_t loops  = (uint16_t)((buf[10] << 8) | buf[11]);

      processShowAreaCommand(x_neg, x_pos, y_neg, y_pos, inc, loops);
    }
  }

  else {
    // Unknown byte → discard
    Serial.read();
  }
}



// ─────────────────────────────────────────────────────────────────────────────
// Galvo: writes DAC/PWM, then returns a PD reading
// ─────────────────────────────────────────────────────────────────────────────
void processGalvoCommand(byte* packet) {
  // validate
  if ((packet[0] & 0b11000000) != 0b10000000) return;
  if ((packet[2] & 0b11000000) != 0b00000000 ||
      (packet[3] & 0b11000000) != 0b00000000) return;

  // decode X
  uint16_t x_val = (((uint16_t)(packet[0] & 0b00111111)) << 6)
                   | (packet[1] & 0b00111111);
  // decode Y
  uint16_t y_val = (((uint16_t)packet[2]) << 6)
                   | (packet[3] & 0b00111111);

  dac.setVoltage(x_val, false);
  analogWrite(pwmPin, y_val);

  // PD measurement
  uint32_t sum = 0;
  for (uint16_t i = 0; i < PD_SAMPLE_COUNT; i++) {
    sum += analogRead(PD_PIN);
  }
  uint16_t avg = sum / PD_SAMPLE_COUNT;

  // send 14-bit in 2 bytes
  byte hb = 0b10000000 | ((avg >> 7) & 0b01111111);
  byte lb = avg & 0b01111111;
  Serial.write(hb);
  Serial.write(lb);
}

// ─────────────────────────────────────────────────────────────────────────────
// Stepper: with linear ramp, then returns a PD reading
// ─────────────────────────────────────────────────────────────────────────────
void processStepperCommand(byte* packet) {
  // validate
  if ((packet[0] & 0b11000000) != 0b11000000) return;

  // decode steps (12-bit)
  uint16_t raw_steps = (((uint16_t)(packet[0] & 0b00111111)) << 6)
                       | (packet[1] & 0b00111111);
  uint32_t total_steps = raw_steps;

  // decode frequency field (11-bit) and rebuild Hz
  uint16_t freq_field = (((uint16_t)(packet[2] & 0b00011111)) << 6)
                        | (packet[3] & 0b00111111);
  uint32_t step_frequency = (uint32_t)freq_field * 24;  // Hz

  // compute target (end) period
  uint32_t period_end_us = 1000000UL / step_frequency;

  // decide ramp start frequency
  uint32_t start_freq = (step_frequency >= RAMP_START_FREQ)
                        ? RAMP_START_FREQ
                        : step_frequency;
  uint32_t period_start_us = 1000000UL / start_freq;

  // delta and number of ramp pulses
  uint32_t delta_us    = (period_start_us > period_end_us)
                         ? (period_start_us - period_end_us)
                         : 0;
  uint32_t ramp_steps  = (RAMP_DEC_US > 0)
                         ? (delta_us / RAMP_DEC_US)
                         : 0;

  // decode direction
  uint8_t dir = (packet[2] >> 5) & 0x01;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);

  // generate pulses with ramp
  for (uint32_t i = 0; i < total_steps; i++) {
    // compute current period
    uint32_t current_us;
    if (i < ramp_steps) {
      current_us = period_start_us - (uint32_t)i * RAMP_DEC_US;
    } else {
      current_us = period_end_us;
    }
    // pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    if (current_us > 10) {
      delayMicroseconds(current_us - 10);
    }
  }

  // Update Arduino’s bookkeeping of Z
  if (dir) {
    current_z += total_steps;   // DOWN = increase
  }
  else {
    current_z -= total_steps;   // UP   = decrease
  }
  // ─────────────────────────────────────────────────────────────────────────


  // after motion, take PD reading
  uint32_t sum = 0;
  for (uint16_t i = 0; i < PD_SAMPLE_COUNT; i++) {
    sum += analogRead(PD_PIN);
    delay(1);
  }
  uint16_t avg = sum / PD_SAMPLE_COUNT;

  // send 14-bit back
  byte hb = 0b10000000 | ((avg >> 7) & 0b01111111);
  byte lb = avg & 0b01111111;
  Serial.write(hb);
  Serial.write(lb);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: readSinglePD()
// ─────────────────────────────────────────────────────────────────────────────
uint16_t readSinglePD() {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < PD_SAMPLE_COUNT; i++) {
    sum += analogRead(PD_PIN);
  }
  return (uint16_t)(sum / PD_SAMPLE_COUNT);
}

// ─────────────────────────────────────────────────────────────────────────────
// processZScanCommand
//   • neg_limit   (signed 16-bit, e.g. -1500) 
//   • pos_limit   (signed 16-bit, e.g. +1500) 
//   • increment   (unsigned 16-bit, e.g. 5) 
//   • freq_field  (unsigned 16-bit) => step_frequency = freq_field * 24 Hz
//
//  This version does:
//    1) Move 0 → neg_limit (with ramp, discard first PD)
//    2) Sweep neg_limit → pos_limit, reading PD at each step into z_buffer/pd_buffer
//    3) Send back [count (uint16)] + count×[(z_i int16) , (pd_i uint16)]
//    4) Move pos_limit → 0 (silent, no PD output)
// ─────────────────────────────────────────────────────────────────────────────
void processZScanCommand(int16_t neg_rel, int16_t pos_rel,
                         uint16_t increment, uint16_t freq_field) {
  // Reconstruct actual stepping frequency:
  uint32_t step_frequency = (uint32_t)freq_field * 24UL;

  // 1) Remember where we started
  int16_t start_z = current_z;

  // 2) Compute absolute endpoints of the scan
  int16_t neg_abs = start_z + neg_rel;   // e.g. if start_z = –300, neg_rel = –1500 → neg_abs = –1800
  int16_t pos_abs = start_z + pos_rel;   // e.g.  –300 + (+1500) = +1200

  // 3) Step from current_z → neg_abs:
  {
    int32_t delta0 = (int32_t)neg_abs - current_z;
    if (delta0 != 0) {
      bool dirBit = (delta0 > 0) ? 1 : 0;
      digitalWrite(DIR_PIN, dirBit ? HIGH : LOW);
      uint16_t steps_to_move = (uint16_t)abs(delta0);

      // Compute ramp parameters for “step_frequency”
      uint32_t period_end_us = 1000000UL / step_frequency;
      uint32_t start_freq    = (step_frequency >= RAMP_START_FREQ)
                                ? RAMP_START_FREQ
                                : step_frequency;
      uint32_t period_start_us = 1000000UL / start_freq;
      uint32_t delta_us     = (period_start_us > period_end_us)
                               ? (period_start_us - period_end_us)
                               : 0;
      uint32_t ramp_steps   = (RAMP_DEC_US > 0) ? (delta_us / RAMP_DEC_US) : 0;

      for (uint32_t i = 0; i < steps_to_move; i++) {
        uint32_t this_period_us = (i < ramp_steps)
                                  ? (period_start_us - i * RAMP_DEC_US)
                                  : period_end_us;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        if (this_period_us > 10) {
          delayMicroseconds(this_period_us - 10);
        }
      }
      // Discard first PD reading
      (void)readSinglePD();
      current_z = neg_abs;
    }
  }

  // 4) Sweep from neg_abs → pos_abs in “increment” steps, storing PD at each:
  int32_t span = (int32_t)pos_abs - (int32_t)neg_abs;
  if (span < 0 || increment == 0) {
    // Invalid relative range → return count=0
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    // Don’t move anywhere else, leave current_z alone.
    return;
  }
  uint16_t num_points = (span / increment) + 1;  // inclusive endpoints

  // Check buffer size:
  if (num_points > MAX_POINTS) {
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    return;
  }

  // Point 0 at neg_abs:
  z_buffer[0]  = (int16_t)(neg_abs - start_z);  // = neg_rel
  pd_buffer[0] = readSinglePD();
  uint16_t idx = 1;

  // Now climb in “increment” steps:
  for (int32_t z_target = (int32_t)neg_abs + increment;
       z_target <= (int32_t)pos_abs;
       z_target += increment) {
    int32_t delta = z_target - current_z;
    if (delta != 0) {
      bool dirBit = (delta > 0) ? 1 : 0;
      digitalWrite(DIR_PIN, dirBit ? HIGH : LOW);
      uint16_t move_steps = (uint16_t)abs(delta);

      // Ramp logic
      uint32_t period_end_us = 1000000UL / step_frequency;
      uint32_t start_freq    = (step_frequency >= RAMP_START_FREQ)
                                ? RAMP_START_FREQ
                                : step_frequency;
      uint32_t period_start_us = 1000000UL / start_freq;
      uint32_t delta_us     = (period_start_us > period_end_us)
                               ? (period_start_us - period_end_us)
                               : 0;
      uint32_t ramp_steps   = (RAMP_DEC_US > 0) ? (delta_us / RAMP_DEC_US) : 0;

      for (uint32_t i = 0; i < move_steps; i++) {
        uint32_t this_period_us = (i < ramp_steps)
                                  ? (period_start_us - i * RAMP_DEC_US)
                                  : period_end_us;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        if (this_period_us > 10) {
          delayMicroseconds(this_period_us - 10);
        }
      }
      current_z = (int16_t)z_target;
    }
    // Store PD at absolute z_target, but subtract start_z so that z_buffer is “relative”:
    z_buffer[idx]  = (int16_t)(current_z - start_z);
    pd_buffer[idx] = readSinglePD();
    idx++;
  }

  // 5) Send back [num_points] + all (z_rel_i, pd_i) pairs:
  Serial.write((uint8_t)(num_points >> 8));
  Serial.write((uint8_t)(num_points & 0xFF));
  for (uint16_t i = 0; i < num_points; i++) {
    uint16_t zraw = (uint16_t)z_buffer[i];  // already signed→unsigned conversion
    Serial.write((uint8_t)(zraw >> 8));
    Serial.write((uint8_t)(zraw & 0xFF));

    uint16_t pdv = pd_buffer[i];
    Serial.write((uint8_t)(pdv >> 8));
    Serial.write((uint8_t)(pdv & 0xFF));
  }

  // 6) Finally, move silently from pos_abs → start_z:
  if (current_z != start_z) {
    int32_t back_delta = (int32_t)start_z - (int32_t)current_z;
    bool dirBit = (back_delta > 0) ? 1 : 0;
    digitalWrite(DIR_PIN, dirBit ? HIGH : LOW);
    uint16_t steps_back = (uint16_t)abs(back_delta);

    uint32_t period_end_us = 1000000UL / step_frequency;
    uint32_t start_freq    = (step_frequency >= RAMP_START_FREQ)
                              ? RAMP_START_FREQ
                              : step_frequency;
    uint32_t period_start_us = 1000000UL / start_freq;
    uint32_t delta_us       = (period_start_us > period_end_us)
                               ? (period_start_us - period_end_us)
                               : 0;
    uint32_t ramp_steps     = (RAMP_DEC_US > 0) ? (delta_us / RAMP_DEC_US) : 0;

    for (uint32_t i = 0; i < steps_back; i++) {
      uint32_t this_period_us = (i < ramp_steps)
                                ? (period_start_us - i * RAMP_DEC_US)
                                : period_end_us;
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN, LOW);
      if (this_period_us > 10) {
        delayMicroseconds(this_period_us - 10);
      }
    }
    current_z = start_z;
  }
  // Now the stepper is back at start_z, and the data we sent are already “relative” z = neg_rel..pos_rel.
}

// =============================================================================
// processAutofocusCommand (relative)
//   • neg_rel    (signed 16-bit ≤ 0)   // raw offset below current_z
//   • pos_rel    (signed 16-bit ≥ 0)   // raw offset above current_z
//   • increment  (unsigned 16-bit > 0)
//   • freq_field (unsigned 16-bit = step_frequency/24)
// 
// Behavior:
//   1) Remember start_z = current_z.  
//   2) Compute absolute endpoints: 
//         neg_abs = start_z + neg_rel  
//         pos_abs = start_z + pos_rel  
//   3) Move from current_z → neg_abs (ramp, discard first PD).  
//   4) Sweep from neg_abs → pos_abs in "increment" steps, storing:  
//         z_buffer[i] = (absolute_z – start_z)   // so returned Z runs from neg_rel..pos_rel  
//         pd_buffer[i] = PD_at_that_point  
//   5) Find best index (where pd_buffer[] is max), set best_rel = z_buffer[best_i],  
//      best_abs = start_z + best_rel.  
//   6) Send back:  
//         2 bytes = num_points (uint16)  
//         num_points × [ (z_rel as int16 → 2 bytes) , (pd as uint16 → 2 bytes ) ]  
//   7) Move silently (ramp) from pos_abs → best_abs (no Serial, no PD).  
//   8) Move back to 0. 
// =============================================================================
void processAutofocusCommand(int16_t neg_rel, int16_t pos_rel,
                             uint16_t increment, uint16_t freq_field)
{
  // 1) Reconstruct actual stepping frequency:
  uint32_t step_frequency = (uint32_t)freq_field * 24UL;

  // 2) Remember where the motor is right now:
  int16_t start_z = current_z;

  // 3) Compute absolute endpoints for the sweep:
  int16_t neg_abs = start_z + neg_rel;
  int16_t pos_abs = start_z + pos_rel;

  // ── Step A: Move from start_z → neg_abs (ramp) ─────────────────────────────
  {
    int32_t delta0 = (int32_t)neg_abs - current_z;
    if (delta0 != 0) {
      bool dirBit0 = (delta0 > 0) ? 1 : 0;  // 1 = DOWN, 0 = UP
      digitalWrite(DIR_PIN, dirBit0 ? HIGH : LOW);
      uint16_t steps_to_move0 = (uint16_t)abs(delta0);

      // Compute ramp parameters for “step_frequency”
      uint32_t period_end_us0 = 1000000UL / step_frequency;
      uint32_t start_freq0    = (step_frequency >= RAMP_START_FREQ)
                                 ? RAMP_START_FREQ
                                 : step_frequency;
      uint32_t period_start_us0 = 1000000UL / start_freq0;
      uint32_t delta_us0 = (period_start_us0 > period_end_us0)
                            ? (period_start_us0 - period_end_us0)
                            : 0;
      uint32_t ramp_steps0 = (RAMP_DEC_US > 0)
                              ? (delta_us0 / RAMP_DEC_US)
                              : 0;

      for (uint32_t i = 0; i < steps_to_move0; i++) {
        uint32_t this_period_us = (i < ramp_steps0)
                                  ? (period_start_us0 - i * RAMP_DEC_US)
                                  : period_end_us0;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        if (this_period_us > 10) {
          delayMicroseconds(this_period_us - 10);
        }
      }
      // Discard first PD reading (settle)
      (void)readSinglePD();
      current_z = neg_abs;
    }
  }

  // ── Step B: Sweep from neg_abs → pos_abs in “increment” steps ─────────────
  {
    int32_t span = (int32_t)pos_abs - (int32_t)neg_abs;
    if (span < 0 || increment == 0) {
      // Invalid range → send back count = 0
      uint16_t zero = 0;
      Serial.write((uint8_t)(zero >> 8));
      Serial.write((uint8_t)(zero & 0xFF));
      return;
    }
    uint16_t num_points = (span / increment) + 1;  // inclusive both ends

    // If it doesn’t fit in our buffer, abort
    if (num_points > MAX_POINTS) {
      uint16_t zero = 0;
      Serial.write((uint8_t)(zero >> 8));
      Serial.write((uint8_t)(zero & 0xFF));
      return;
    }

    // Point 0 at neg_abs:
    z_buffer[0]  = (int16_t)(neg_abs - start_z);  // = neg_rel
    pd_buffer[0] = readSinglePD();
    uint16_t idx = 1;

    // Now step upward in “increment” steps:
    for (int32_t z_target = (int32_t)neg_abs + increment;
         z_target <= (int32_t)pos_abs;
         z_target += increment)
    {
      int32_t delta = z_target - current_z;
      if (delta != 0) {
        bool dirBit = (delta > 0) ? 1 : 0;  // 1 = DOWN, 0 = UP
        digitalWrite(DIR_PIN, dirBit ? HIGH : LOW);
        uint16_t move_steps = (uint16_t)abs(delta);

        // Recompute ramp parameters at each step:
        uint32_t period_end_us   = 1000000UL / step_frequency;
        uint32_t start_freq      = (step_frequency >= RAMP_START_FREQ)
                                    ? RAMP_START_FREQ
                                    : step_frequency;
        uint32_t period_start_us = 1000000UL / start_freq;
        uint32_t delta_us        = (period_start_us > period_end_us)
                                    ? (period_start_us - period_end_us)
                                    : 0;
        uint32_t ramp_steps      = (RAMP_DEC_US > 0)
                                    ? (delta_us / RAMP_DEC_US)
                                    : 0;

        for (uint32_t i = 0; i < move_steps; i++) {
          uint32_t this_period_us = (i < ramp_steps)
                                     ? (period_start_us - i * RAMP_DEC_US)
                                     : period_end_us;
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(STEP_PIN, LOW);
          if (this_period_us > 10) {
            delayMicroseconds(this_period_us - 10);
          }
        }
        current_z = (int16_t)z_target;
      }
      // Store PD at z_target (relative to start_z):
      z_buffer[idx]  = (int16_t)(current_z - start_z);
      pd_buffer[idx] = readSinglePD();
      idx++;
    }

    // ── Step C: (Optionally find best_z on Arduino, but we won’t use it here) ──
    // We compute best_z in Python now. Leaving this in place is optional.
    // uint16_t best_i = 0;
    // for (uint16_t i = 1; i < num_points; i++) {
    //   if (pd_buffer[i] > pd_buffer[best_i]) {
    //     best_i = i;
    //   }
    // }
    // int16_t best_rel = z_buffer[best_i];
    // int16_t best_abs = start_z + best_rel;

    // ── Step D: Send back [num_points] + each (z_rel, pd) pair ─────────────
    Serial.write((uint8_t)(num_points >> 8));
    Serial.write((uint8_t)(num_points & 0xFF));
    for (uint16_t i = 0; i < num_points; i++) {
      uint16_t zraw = (uint16_t)z_buffer[i];  // “relative Z” (big‐endian)
      Serial.write((uint8_t)(zraw >> 8));
      Serial.write((uint8_t)(zraw & 0xFF));

      uint16_t pdv = pd_buffer[i];
      Serial.write((uint8_t)(pdv >> 8));
      Serial.write((uint8_t)(pdv & 0xFF));
    }

    // ── Step E (NEW): After streaming all (z_rel,PD), return to start_z ─────
    if (current_z != start_z) {
      int32_t back_delta = (int32_t)start_z - (int32_t)current_z;
      bool dirBack = (back_delta > 0) ? 1 : 0;   // 1 = DOWN, 0 = UP
      digitalWrite(DIR_PIN, dirBack ? HIGH : LOW);
      uint16_t steps_back = (uint16_t)abs(back_delta);

      // Recompute ramp parameters (same freq used during sweep)
      uint32_t period_end_us = 1000000UL / step_frequency;
      uint32_t start_freq   = (step_frequency >= RAMP_START_FREQ)
                                ? RAMP_START_FREQ
                                : step_frequency;
      uint32_t period_start_us = 1000000UL / start_freq;
      uint32_t delta_us    = (period_start_us > period_end_us)
                               ? (period_start_us - period_end_us)
                               : 0;
      uint32_t ramp_steps  = (RAMP_DEC_US > 0)
                               ? (delta_us / RAMP_DEC_US)
                               : 0;

      for (uint32_t i = 0; i < steps_back; i++) {
        uint32_t this_period_us = (i < ramp_steps)
                                   ? (period_start_us - i * RAMP_DEC_US)
                                   : period_end_us;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        if (this_period_us > 10) {
          delayMicroseconds(this_period_us - 10);
        }
      }
      current_z = start_z;  // Now the motor sits exactly at start_z
    }
    // → No Arduino‐side “move to best_z” anymore. Python will handle the final move.
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// processGalvoScanCommand(...)  (optimized, generalized raster offload)
// ─────────────────────────────────────────────────────────────────────────────
void processGalvoScanCommand(int16_t x_neg, int16_t x_pos,
                             int16_t y_neg, int16_t y_pos,
                             uint16_t increment) {
  const int16_t CENTER = 2047;

  // 1) Convert to absolute raw DAC/PWM endpoints and clamp to [0..4095]
  int32_t x_start32 = (int32_t)CENTER + (int32_t)x_neg;
  int32_t x_end32   = (int32_t)CENTER + (int32_t)x_pos;
  int32_t y_start32 = (int32_t)CENTER + (int32_t)y_neg;
  int32_t y_end32   = (int32_t)CENTER + (int32_t)y_pos;

  if (x_start32 < 0)       x_start32 = 0;
  if (x_end32   > 4095)    x_end32   = 4095;
  if (y_start32 < 0)       y_start32 = 0;
  if (y_end32   > 4095)    y_end32   = 4095;
  if (increment == 0)      increment = 1;

  // 2) Compute how many columns (x_count)
  int32_t x_range = x_end32 - x_start32;
  if (x_range < 0) {
    // Invalid range → send back count=0
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    return;
  }
  uint16_t x_count = (uint16_t)((x_range / increment) + 1);

  // 3) For each column:
  for (uint16_t cx = 0; cx < x_count; cx++) {
    int16_t x_abs = (int16_t)(x_start32 + (int32_t)cx * increment);

    // 3a) Move DAC once per column
    dac.setVoltage((uint16_t)x_abs, false);

    // 3b) Compute how many rows (points) in this column:
    int32_t y_range = y_end32 - y_start32;
    if (y_range < 0) {
      // No points → header=0
      uint16_t zero = 0;
      Serial.write((uint8_t)(zero >> 8));
      Serial.write((uint8_t)(zero & 0xFF));
      continue;
    }
    uint16_t y_count = (uint16_t)((y_range / increment) + 1);

    // If column is too large for our 1000×2-byte buffer, abort:
    if (y_count > MAX_COLUMN_POINTS) {
      uint16_t zero = 0;
      Serial.write((uint8_t)(zero >> 8));
      Serial.write((uint8_t)(zero & 0xFF));
      return;
    }

    // 3c) For each y in this column, write only the 2-byte PD into columnBuf:
    for (uint16_t cy = 0; cy < y_count; cy++) {
      int16_t y_abs = (int16_t)(y_start32 + (int32_t)cy * increment);
      analogWrite(pwmPin, (uint16_t)y_abs);

      // ← 10 µs settling (instead of 50 µs)
      // delayMicroseconds(10);

      // readSinglePD() now does PD_SAMPLE_COUNT=4 reads
      uint16_t pd_val = readSinglePD();

      // store 2 bytes (big-endian) of PD at offset = cy*2
      uint16_t offset = (uint16_t)cy * 2;
      columnBuf[offset + 0] = (uint8_t)(pd_val >> 8);
      columnBuf[offset + 1] = (uint8_t)(pd_val & 0xFF);
    }

    // 3d) Send 2-byte header = col_idx, then 2-byte header = y_count
    Serial.write((uint8_t)(cx >> 8));
    Serial.write((uint8_t)(cx & 0xFF));
    Serial.write((uint8_t)(y_count >> 8));
    Serial.write((uint8_t)(y_count & 0xFF));

    // 3e) Send the entire PD column in one chunk (y_count × 2 bytes)
    Serial.write(columnBuf, (size_t)y_count * 2);

    // 3f) If not last column, jump diagonally to next column’s top (silent)
    if (cx + 1 < x_count) {
      int16_t x_next = (int16_t)(x_start32 + (int32_t)(cx + 1) * increment);
      dac.setVoltage((uint16_t)x_next, false);
      analogWrite(pwmPin, (uint16_t)y_start32);
      delayMicroseconds(50);
      // ← no PD read here (silent jump)
    }
  }
  // Done: streamed all x_count columns
}

// ─────────────────────────────────────────────────────────────────────────────
// (Y) Implementation of process3DLayerScanCommand(...)  (NEW)
// ─────────────────────────────────────────────────────────────────────────────
void process3DLayerScanCommand(int16_t x_neg, int16_t x_pos,
                               int16_t y_neg, int16_t y_pos,
                               uint16_t xy_inc,
                               int16_t z_neg, int16_t z_pos,
                               uint16_t z_inc,
                               uint16_t freq_field)
{
  // 1) Reconstruct frequencies and save start Z
  uint32_t stepper_freq_hz = (uint32_t)freq_field * 24UL;
  int16_t start_z = current_z;         // remember where we began
  int16_t neg_z_abs = (int16_t)(start_z + z_neg);
  int16_t pos_z_abs = (int16_t)(start_z + z_pos);

  // 2) Compute X/Y dimensions
  //    Center = 2047 on DAC and PWM side
  const int16_t CENTER = 2047;
  int32_t x_start32 = (int32_t)CENTER + (int32_t)x_neg;
  int32_t x_end32   = (int32_t)CENTER + (int32_t)x_pos;
  int32_t y_start32 = (int32_t)CENTER + (int32_t)y_neg;
  int32_t y_end32   = (int32_t)CENTER + (int32_t)y_pos;
  if (x_start32 < 0)       x_start32 = 0;
  if (x_end32   > 4095)    x_end32   = 4095;
  if (y_start32 < 0)       y_start32 = 0;
  if (y_end32   > 4095)    y_end32   = 4095;
  if (xy_inc == 0)         xy_inc    = 1;

  int32_t x_range = x_end32 - x_start32;
  if (x_range < 0) {
    // Invalid → send back 0 layers
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    return;
  }
  uint16_t x_count = (uint16_t)((x_range / xy_inc) + 1);

  int32_t y_range = y_end32 - y_start32;
  if (y_range < 0) {
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    return;
  }
  uint16_t y_count = (uint16_t)((y_range / xy_inc) + 1);

  // 3) Compute Z layers
  int32_t z_span = (int32_t)z_pos - (int32_t)z_neg;  // how far from neg→pos
  if (z_span < 0 || z_inc == 0) {
    uint16_t zero = 0;
    Serial.write((uint8_t)(zero >> 8));
    Serial.write((uint8_t)(zero & 0xFF));
    return;
  }
  uint16_t nz_layers = (uint16_t)((z_span / z_inc) + 1);

  // 4) Send back 2‐byte header = nz_layers
  Serial.write((uint8_t)(nz_layers >> 8));
  Serial.write((uint8_t)(nz_layers & 0xFF));

  // 5) Move stepper from current_z → neg_z_abs (ramp), discard first PD
  {
    int32_t delta0 = (int32_t)neg_z_abs - current_z;
    if (delta0 != 0) {
      bool dir0 = (delta0 > 0) ? 1 : 0;
      digitalWrite(DIR_PIN, dir0 ? HIGH : LOW);
      uint16_t steps0 = (uint16_t)abs(delta0);

      // Ramp computation for stepper_freq_hz (same as in processZScanCommand)
      uint32_t period_end_us = 1000000UL / stepper_freq_hz;
      uint32_t start_freq   = (stepper_freq_hz >= RAMP_START_FREQ)
                               ? RAMP_START_FREQ
                               : stepper_freq_hz;
      uint32_t period_start = 1000000UL / start_freq;
      uint32_t delta_us     = (period_start > period_end_us)
                               ? (period_start - period_end_us)
                               : 0;
      uint32_t ramp_steps   = (RAMP_DEC_US > 0) ? (delta_us / RAMP_DEC_US) : 0;

      for (uint32_t i = 0; i < steps0; i++) {
        uint32_t this_period = (i < ramp_steps)
                                ? (period_start - i * RAMP_DEC_US)
                                : period_end_us;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(STEP_PIN, LOW);
        if (this_period > 10) {
          delayMicroseconds(this_period - 10);
        }
      }
      // Discard first PD reading
      (void) readSinglePD();
      current_z = neg_z_abs;
    }
  }

  // 6) For each Z layer, do:
  for (uint16_t iz = 0; iz < nz_layers; iz++) {
    // --- Compute this layer’s Z target (absolute) ---
    int16_t this_z = (int16_t)(neg_z_abs + ((int32_t)iz * z_inc));
    // Already, when iz=0, current_z == neg_z_abs, so no move step needed.
    if (iz > 0) {
      // Move stepper from previous layer to this_z
      int32_t delta = (int32_t)this_z - current_z;
      if (delta != 0) {
        bool dir = (delta > 0) ? 1 : 0;
        digitalWrite(DIR_PIN, dir ? HIGH : LOW);
        uint16_t steps_move = (uint16_t)abs(delta);

        // Ramp logic (recompute at each Z step, same as above)
        uint32_t period_end_us = 1000000UL / stepper_freq_hz;
        uint32_t start_freq   = (stepper_freq_hz >= RAMP_START_FREQ)
                                 ? RAMP_START_FREQ
                                 : stepper_freq_hz;
        uint32_t period_start = 1000000UL / start_freq;
        uint32_t delta_us = (period_start > period_end_us)
                              ? (period_start - period_end_us)
                              : 0;
        uint32_t ramp_steps = (RAMP_DEC_US > 0) ? (delta_us / RAMP_DEC_US) : 0;

        for (uint32_t i = 0; i < steps_move; i++) {
          uint32_t this_period = (i < ramp_steps)
                                  ? (period_start - i * RAMP_DEC_US)
                                  : period_end_us;
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(STEP_PIN, LOW);
          if (this_period > 10) {
            delayMicroseconds(this_period - 10);
          }
        }
        // Discard PD after each step
        (void) readSinglePD();
        current_z = this_z;
      }
    }

    // --- Now do one full Galvo scan exactly as processGalvoScanCommand does ---
    //     We reuse most of processGalvoScanCommand’s inner loop, but inline here:

    // 6a) For each x‐column (0..x_count−1):
    for (uint16_t cx = 0; cx < x_count; cx++) {
      int16_t x_abs = (int16_t)(x_start32 + ((int32_t)cx * xy_inc));
      // Move DAC (x) exactly once per column:
      dac.setVoltage((uint16_t)x_abs, false);

      // 6b) Compute rows in this column
      //     y_range, y_count already computed above
      if (y_range < 0) {
        // send header=0
        uint16_t zero = 0;
        Serial.write((uint8_t)(cx >> 8));
        Serial.write((uint8_t)(cx & 0xFF));
        Serial.write((uint8_t)(zero >> 8));
        Serial.write((uint8_t)(zero & 0xFF));
        continue;
      }

      // 6c) For each y in this column, set PWM + read PD into columnBuf
      for (uint16_t cy = 0; cy < y_count; cy++) {
        int16_t y_abs = (int16_t)(y_start32 + ((int32_t)cy * xy_inc));
        analogWrite(pwmPin, (uint16_t)y_abs);

        // Let it settle 10 µs (instead of 50 µs)
        // delayMicroseconds(10);

        // Now read PD_SAMPLE_COUNT samples:
        uint16_t pd_val = readSinglePD();

        // Store two bytes big‐endian at offset = cy*2
        uint16_t offset = (uint16_t)cy * 2;
        columnBuf[offset + 0] = (uint8_t)(pd_val >> 8);
        columnBuf[offset + 1] = (uint8_t)(pd_val & 0xFF);
      }

      // 6d) Send back column header = [col_idx:uint16][y_count:uint16]
      Serial.write((uint8_t)(cx >> 8));
      Serial.write((uint8_t)(cx & 0xFF));
      Serial.write((uint8_t)(y_count >> 8));
      Serial.write((uint8_t)(y_count & 0xFF));

      // 6e) Send entire PD column in one chunk (y_count × 2 bytes):
      Serial.write(columnBuf, (size_t)y_count * 2);

      // 6f) If not last column, jump diagonally to next column’s “top” (silent)
      if (cx + 1 < x_count) {
        int16_t x_next = (int16_t)(x_start32 + ((int32_t)(cx + 1) * xy_inc));
        dac.setVoltage((uint16_t)x_next, false);
        analogWrite(pwmPin, (uint16_t)y_start32);
        // no PD read here
      }
    }

    // End of layer “iz.” The Arduino loops back to (6) to move stepper up by z_inc.
    // After the final layer, the stepper sits at pos_z_abs.
  }

  // Now we’re done with all layers.  current_z == pos_z_abs.
}
// ─────────────────────────────────────────────────────────────────────────────
void processShowAreaCommand(int16_t x_neg, int16_t x_pos,
                            int16_t y_neg, int16_t y_pos,
                            uint16_t inc, uint16_t loops)
{
  const int16_t CENTER = 2047;
  // Convert relative ↦ absolute (clamp safety)
  int16_t x0 = CENTER + x_neg;
  int16_t x1 = CENTER + x_pos;
  int16_t y0 = CENTER + y_neg;
  int16_t y1 = CENTER + y_pos;

  x0 = constrain(x0, 0, 4095);
  x1 = constrain(x1, 0, 4095);
  y0 = constrain(y0, 0, 4095);
  y1 = constrain(y1, 0, 4095);
  if (inc == 0) inc = 1;

  for (uint16_t k = 0; k < loops; k++) {
    // top edge  (x0→x1 @ y0)
    analogWrite(pwmPin, y0);
    for (int16_t x = x0; x <= x1; x += inc)
      dac.setVoltage(x, false);

    // right edge (y0→y1 @ x1)
    dac.setVoltage(x1, false);
    for (int16_t y = y0; y <= y1; y += inc)
      analogWrite(pwmPin, y);

    // bottom edge (x1→x0 @ y1)
    analogWrite(pwmPin, y1);
    for (int16_t x = x1; x >= x0; x -= inc)
      dac.setVoltage(x, false);

    // left edge (y1→y0 @ x0)
    dac.setVoltage(x0, false);
    for (int16_t y = y1; y >= y0; y -= inc)
      analogWrite(pwmPin, y);
  }

  // finished → send 0xAA55
  Serial.write(0xAA);
  Serial.write(0x55);
}
