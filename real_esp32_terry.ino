/*
 * SBUS receiver -> differential (arcade) mix -> two SPEED outputs (PFM)
 *                                             + two DIRECTION outputs.
 *
 * Reads an INVERTED SBUS stream, decodes the two joystick axes, mixes them
 * into left/right speeds, and emits two variable-duty pulse signals whose duty encodes speed
 * (the minority high/low phase is clamped to a fixed ~10 us floor).
 * Each side also drives a DIRECTION pin: HIGH when that side spins forward, 0V in reverse.
 *
 * INPUT  (SBUS):  100000 baud, 8E2, LSB-first, INVERTED, 25-byte frames
 *                 (0x0F header ... 0x00 footer), on UART2 RX.
 *     ch0 = steer     600 = full left  .. 1000 center .. 1400 = full right
 *     ch1 = throttle  600 = full rev   .. 1000 center .. 1400 = full fwd
 *
 * OUTPUT (per motor):  duty encodes speed; the MINORITY phase is clamped
 *   to a PULSE_US (10 us) floor, so:
 *     zero speed  -> constant low (no pulses)
 *     low speed   -> 10 us high pulses, long low gap   (duty < 50%)
 *     ~2/3 speed  -> 10 us / 10 us square              (duty = 50%)
 *     high speed  -> 10 us low notches, growing high   (duty > 50%)
 *     max speed   -> 30 us high + 10 us low            (duty = DUTY_MAX = 75%)
 *
 * TRIM (antisymmetric):  the left side runs a little hot forward / cold
 *     reverse; the right side is the mirror. A constant offset on the signed
 *     command corrects both directions at once (left -TRIM, right +TRIM),
 *     because writeMotor() acts on |command|. A sign-flip clamp keeps the
 *     trim from ever reversing a motor near standstill.
 *
 * No external library (LEDC). Arduino-ESP32 core 3.x.
 * Left and right run on separate LEDC timers so their frequencies are
 * independent (channel 0 = timer 0, channel 2 = timer 1).
 *
 * Wiring: inverted SBUS -> SBUS_RX_PIN (GPIO16), common GND.
 *         LEFT_PWM_PIN / RIGHT_PWM_PIN -> the two motor-driver speed inputs. 3.3V.
 *         LEFT_DIR_PIN / RIGHT_DIR_PIN -> the two motor-driver direction inputs. 3.3V.
 */

// ---- configuration --------------------------------------------------------
#define SBUS_RX_PIN     16
#define LEFT_PWM_PIN    25
#define RIGHT_PWM_PIN   26
#define LEFT_DIR_PIN    33          // HIGH = left spins forward, 0V = reverse
#define RIGHT_DIR_PIN   32          // HIGH = right spins forward, 0V = reverse
#define LEFT_CH         0           // LEDC channel -> timer 0
#define RIGHT_CH        2           // LEDC channel -> timer 1 (independent freq)

#define SBUS_BAUD       100000
#define SBUS_INVERTED   true
#define FRAME_GAP_US    3000UL
#define SIGNAL_TIMEOUT  100UL       // ms without a frame -> failsafe stop

const int DRIVE_MIN = 600, DRIVE_MID = 1000, DRIVE_MAX = 1400;

// speed -> duty output (minority phase clamped to a 10 us floor)
#define DUTY_MAX        0.5f        // full speed -> 30us high / 10us low (75%)
#define PULSE_US        10.0f       // minority-phase minimum pulse width
#define LEDC_RES        10          // bits
#define SPEED_DEADBAND  0.01f       // below this -> off (constant low)
#define MIN_SPEED_OUTPUT 0.55f      // 50% output when speed first becomes nonzero
const int LEDC_MAXDUTY = (1 << LEDC_RES) - 1;
constexpr float TRIML = 0.03f;   
constexpr float TRIMR = 0.01f;       // Adjust experimentally (constant L/R imbalance)
const bool INVERT_STEER = false;    // flips steering sense (affects the mix)

HardwareSerial SBUS(2);
uint8_t  buf[25];
int      idx = 0;
unsigned long lastByteUs = 0, lastFrameMs = 0;

// ---- SBUS decode ----------------------------------------------------------
void decodeChannels(const uint8_t* b, uint16_t* ch) {
  uint32_t bits = 0; int nbits = 0, ci = 0;
  for (int i = 1; i <= 22 && ci < 16; i++) {
    bits |= (uint32_t)b[i] << nbits; nbits += 8;
    while (nbits >= 11 && ci < 16) { ch[ci++] = bits & 0x7FF; bits >>= 11; nbits -= 11; }
  }
}

float norm(int v) {
  float x = (float)(v - DRIVE_MID) / (float)(DRIVE_MAX - DRIVE_MID);
  return constrain(x, -1.0f, 1.0f);
}

// |speed| -> duty, with the minority phase held at PULSE_US (10 us)
void writeMotor(int pin, float cmd) {
  float speed = constrain(fabsf(cmd), 0.0f, 1.0f);
    if (speed < SPEED_DEADBAND) {
        ledcWrite(pin, 0);
        return;
    }
    // Remap:
    // 0..1  ->  0.5..1
    speed = MIN_SPEED_OUTPUT + (1.0f - MIN_SPEED_OUTPUT) * speed;
    float duty = DUTY_MAX * speed;
    float minor = (duty < 0.5f) ? duty : (1.0f - duty);
    float freq = minor * 1000000.0f / PULSE_US;
    ledcChangeFrequency(pin,(uint32_t)lroundf(freq),LEDC_RES);
    ledcWrite(pin,(uint32_t)lroundf(duty * LEDC_MAXDUTY));
}

void driveStop() {
  ledcWrite(LEFT_PWM_PIN,  0);          // constant low
  ledcWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR_PIN,  LOW);     // defined direction state on failsafe
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

// ---- apply one frame ------------------------------------------------------
void applyFrame(const uint8_t* b) {
  uint16_t ch[16];
  decodeChannels(b, ch);

  float throttle = norm(ch[1])*norm(ch[1])*norm(ch[1]);
  float steer = 0.8*((norm(ch[0]))*(norm(ch[0]))*(norm(ch[0])));
  if (INVERT_STEER) steer = -steer;
  float left  = throttle + steer;
  float right = throttle - steer;

  float m = max(fabsf(left), fabsf(right));     // keep turn ratio if it overshoots
  if (m > 1.0f) { left /= m; right /= m; }

  // Antisymmetric imbalance trim (constant across range, flips with direction).
  // fwd(+): left magnitude down, right magnitude up; rev(-): the reverse, via fabsf in writeMotor.
  //float lt = left - TRIM;
  float lt = left;
  if (left < 0.0f) lt = left + TRIML; //slows down left if the motor is in reverse
 // float rt = right + TRIM;
  float rt = right;
  if (right > 0.0f) rt = right - TRIMR; //slows down the right side if the motor is forwards

  if ((lt < 0.0f) != (left  < 0.0f)) lt = 0.0f; // trim must not reverse a motor near standstill
  if ((rt < 0.0f) != (right < 0.0f)) rt = 0.0f;
  left = lt; right = rt;

  // Direction outputs: HIGH when that side spins forward, 0V otherwise.
  digitalWrite(LEFT_DIR_PIN,  left  > 0.0f ? HIGH : LOW);
  digitalWrite(RIGHT_DIR_PIN, right > 0.0f ? HIGH : LOW);

  writeMotor(LEFT_PWM_PIN,  left);              // magnitude -> frequency
  writeMotor(RIGHT_PWM_PIN, right);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  SBUS.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1, SBUS_INVERTED);

  // separate channels -> separate timers -> independent L/R frequencies
  ledcAttachChannel(LEFT_PWM_PIN,  1000, LEDC_RES, LEFT_CH);
  ledcAttachChannel(RIGHT_PWM_PIN, 1000, LEDC_RES, RIGHT_CH);

  pinMode(LEFT_DIR_PIN,  OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  driveStop();                                  // also sets DIR pins low

  Serial.println("SBUS receiver + mix ready (inverted input, duty speed output, dir outputs).");
}

void loop() {
  while (SBUS.available()) {
    unsigned long now = micros();
    if (now - lastByteUs > FRAME_GAP_US) idx = 0;
    lastByteUs = now;
    uint8_t byte = SBUS.read();
    if (idx == 0 && byte != 0x0F) continue;
    buf[idx++] = byte;
    if (idx == 25) {
      idx = 0;
      if (buf[0] == 0x0F && buf[24] == 0x00) { applyFrame(buf); lastFrameMs = millis(); }
    }
  }
  if (millis() - lastFrameMs > SIGNAL_TIMEOUT) driveStop();
}
