/*
 * SBUS + HOST -> differential drive -> two SPEED outputs (PFM) + two DIR outputs,
 * plus a HOST-DRIVEN BED CHANNEL emitted as SBUS frames on UART2 TX.
 *
 * Reads an INVERTED SBUS stream (RC), decodes the two joystick axes, mixes them
 * into left/right speeds; OR takes left/right wheel commands from a host (Jetson/
 * laptop) over USB. One shared actuator path (driveMotors) applies the physical
 * L/R trim, sets the direction pins, and emits the PFM speed signal (minority
 * high/low phase clamped to a ~10 us floor) for whichever source is active.
 * Each side drives a DIR pin: HIGH when that side spins forward, 0V in reverse.
 *
 * ---- ARBITRATION (see controlStep) --------------------------------------
 *   Autonomy is allowed only when the transmitter is NOT in control, i.e.:
 *       - no SBUS frames arriving (RC link down / receiver unpowered), OR
 *       - frames arriving but the flag byte marks the TX off (TXOFF_FLAG)
 *   In that case the host drives (if its command is fresh, else STOP).
 *   Whenever the transmitter IS live (frames arriving, TX on), RC control is
 *   exclusive -- the sticks drive and host commands are ignored.
 *
 * ---- HOST LINK (USB, UART0 / Serial) ------------------------------------
 *   Commands arrive over the ESP32's USB port (the same cable used to flash),
 *   which is UART0 = Serial via the onboard CP2102 bridge. No extra wiring.
 *   Protocol (newline-terminated ASCII):
 *       "M <left> <right> <bed>\n"  left,right in -1..+1; bed in {-1,0,+1}
 *                                   e.g.  M 0.50 -0.25 1
 *       "M <left> <right>\n"        legacy 2-arg form: NO bed command (the bed
 *                                   channel goes idle and TX2 reverts to echo)
 *       "S\n"                       stop: wheels 0 AND bed 0 (bed-stop frame)
 *   Every valid line is a heartbeat. No line within CMD_TIMEOUT -> stop.
 *   NOTE: Serial is the COMMAND link now, not a debug console -- keep
 *         Serial.print out of the running loop or it corrupts the stream.
 *   NOTE: opening the USB port auto-resets the board (~300 ms dead on connect).
 *
 * INPUT  (SBUS):  100000 baud, 8E2, LSB-first, INVERTED, 25-byte frames
 *                 (0x0F header ... 0x00 footer), on UART2 RX.
 *     ch0 = steer     600 = full left  .. 1000 center .. 1400 = full right
 *     ch1 = throttle  600 = full rev   .. 1000 center .. 1400 = full fwd
 *     flag byte = buf[23]  (TX-off detection)
 *
 * ---- UART2 TX: ECHO *or* BED FRAMES (mutually exclusive) -----------------
 *   Default (ECHO): every byte read on UART2 RX is written straight back out on
 *     UART2 TX (SBUS_TX_PIN, GPIO17) at the same baud/format/polarity, before any
 *     framing or validation -- a transparent tap of the receiver stream for a
 *     second SBUS consumer or an analyzer. Decoding is unaffected. Set SBUS_ECHO
 *     false to disable.
 *
 *   BED OVERRIDE: when the host has issued a bed command AND the transmitter is
 *     not in control (same autoAllowed test as the drive arbitration: no SBUS
 *     frames, or frames present with the TX flagged off), the echo is SUPPRESSED
 *     and UART2 TX instead emits a synthetic 25-byte SBUS frame every
 *     BED_FRAME_US (14.022 ms start-to-start, measured: 3.000 ms of frame plus
 *     an 11.022 ms idle gap), encoding the bed direction. The three frames are byte-for-byte
 *     identical except for BED_FRAME[7]:
 *         bed = +1  ->  0x70
 *         bed =  0  ->  0x3E
 *         bed = -1  ->  0x0C
 *     If the host command goes stale (CMD_TIMEOUT) the override is KEPT but the
 *     emitted byte is forced to bed-stop -- an active stop, mirroring driveStop
 *     on the wheels. The override drops (and the echo resumes, at the next frame
 *     boundary) only when the transmitter takes control again.
 *
 * OUTPUT (per motor):  duty encodes speed; the MINORITY phase is clamped
 *   to a PULSE_US (10 us) floor, so:
 *     zero speed  -> constant low (no pulses)
 *     low speed   -> 10 us high pulses, long low gap   (duty < 50%)
 *     ~2/3 speed  -> 10 us / 10 us square              (duty = 50%)
 *     high speed  -> 10 us low notches, growing high   (duty > 50%)
 *     max speed   -> 30 us high + 10 us low            (duty = DUTY_MAX)
 *
 * TRIM (antisymmetric):  the left side runs a little hot forward / cold
 *     reverse; the right side is the mirror. A constant offset on the signed
 *     command corrects it, because writeMotor() acts on |command|. A sign-flip
 *     clamp keeps the trim from ever reversing a motor near standstill.
 *
 * No external library (LEDC). Arduino-ESP32 core 3.x. Separate L/R timers.
 *
 * Wiring: inverted SBUS -> SBUS_RX_PIN (GPIO16), common GND.
 *         SBUS_TX_PIN (GPIO17) -> echoed inverted SBUS out / bed frames (3.3V).
 *         LEFT_PWM_PIN / RIGHT_PWM_PIN -> motor-driver speed inputs. 3.3V.
 *         LEFT_DIR_PIN / RIGHT_DIR_PIN -> motor-driver direction inputs. 3.3V.
 *         Host commands over USB (UART0). No command wiring needed.
 */

// ---- configuration --------------------------------------------------------
#define SBUS_RX_PIN     16
#define SBUS_TX_PIN     17          // UART2 TX: raw echo of the receiver stream
#define SBUS_ECHO       true        // false -> TX2 stays idle (pin still driven)
#define LEFT_PWM_PIN    25
#define RIGHT_PWM_PIN   26
#define LEFT_DIR_PIN    33          // HIGH = left spins forward, 0V = reverse
#define RIGHT_DIR_PIN   32          // HIGH = right spins forward, 0V = reverse
#define LEFT_CH         0           // LEDC channel -> timer 0
#define RIGHT_CH        2           // LEDC channel -> timer 1 (independent freq)

#define CMD_BAUD        115200      // host command link over USB (UART0 / Serial)

#define SBUS_BAUD       100000
#define SBUS_INVERTED   true
#define FRAME_GAP_US    3000UL
#define SIGNAL_TIMEOUT  100UL       // ms without an SBUS frame -> RC considered lost
#define CMD_TIMEOUT     200UL       // ms without a host command -> auto considered lost

// ---- control-source policy ------------------------------------------------
// The transmitter has priority whenever it is live. Autonomy is permitted only
// when RC is absent (no frames) or the receiver flags the TX as off.
// SBUS flag byte is buf[23]. This build has OBSERVED 0x10 for "transmitter
// off"; the STANDARD failsafe bit is 0x08 (raised by most receivers when the
// TX link is lost in flight, e.g. out of range -- the receiver keeps streaming
// frames with held/failsafe channel values). Either condition must hand control
// to the host: if only 0x10 were checked, an in-range-loss failsafe would keep
// driving the motors from the receiver's stale channel data.
#define TXOFF_FLAG      0x10        // observed: transmitter switched off
#define FAILSAFE_FLAG   0x08        // standard SBUS failsafe bit
#define RC_GONE_FLAGS   (TXOFF_FLAG | FAILSAFE_FLAG)

// ---- bed channel (synthetic SBUS out on UART2 TX) -------------------------
// Frame period (start-of-frame to start-of-frame) for the generated stream,
// in microseconds. Measured off the real transmitter: 11.022 ms from the end of
// one frame to the start of the next, plus 3.000 ms of line time for the frame
// itself (25 bytes x 12 bits x 10 us at 100 kbaud, no inter-byte gaps) =
// 14.022 ms. Scheduled in micros() rather than millis() because millis' 1 ms
// granularity would smear the period across 14-15 ms.
#define BED_FRAME_US    14022UL
#define BED_BYTE_INDEX  7           // the only byte that differs between states
#define BED_BYTE_UP     0x70        // bed = +1
#define BED_BYTE_STOP   0x3E        // bed =  0
#define BED_BYTE_DOWN   0x0C        // bed = -1

// Template frame; BED_FRAME_TEMPLATE[BED_BYTE_INDEX] is overwritten per state.
static const uint8_t BED_FRAME_TEMPLATE[25] = {
  0x0F, 0xE8, 0x63, 0x5A, 0xFA, 0xD0, 0x87, BED_BYTE_STOP,
  0xF4, 0x21, 0x63, 0xD4, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00
};

const int DRIVE_MIN = 600, DRIVE_MID = 1000, DRIVE_MAX = 1400;

// speed -> duty output (minority phase clamped to a 10 us floor)
#define DUTY_MAX        0.5f        // full speed -> 30us high / 10us low (75%)
#define PULSE_US        10.0f       // minority-phase minimum pulse width
#define LEDC_RES        10          // bits
#define SPEED_DEADBAND  0.01f       // below this -> off (constant low)
#define MIN_SPEED_OUTPUT 0.55f      // output floor when speed first becomes nonzero
const int LEDC_MAXDUTY = (1 << LEDC_RES) - 1;
constexpr float TRIML = 0.03f;
constexpr float TRIMR = 0.01f;       // Adjust experimentally (constant L/R imbalance)
const bool INVERT_STEER = false;    // flips steering sense (affects the mix)

HardwareSerial SBUS(2);
uint8_t  buf[25];
int      idx = 0;
unsigned long lastByteUs = 0, lastFrameMs = 0, lastCmdMs = 0;

// latest command from each source (magnitude+sign, -1..+1)
float   rcLeft = 0, rcRight = 0;
uint8_t rcFlags = 0;                // SBUS flag byte from the most recent frame
float   jLeft  = 0, jRight  = 0;

// bed command from the host: -1 / 0 / +1. jBedValid latches true on the first
// bed-bearing command (3-arg "M" or "S") and stays true: once the host has
// shown it speaks the bed protocol, the bed channel is never silently dropped.
int     jBed      = 0;
bool    jBedValid = false;

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

// |speed| -> duty, with the minority phase held at PULSE_US (10 us).
// Only touches the LEDC peripheral when the value actually changes: reprogramming
// the timer mid-period (controlStep runs every loop) restarts the counter and
// makes runt pulses / back-to-back highs. ledcChangeFrequency resets the counter,
// so it is guarded hardest; ledcWrite alone latches at the next period (clean).
void writeMotor(int pin, float cmd) {
  static uint32_t lastFreq[40] = {0};
  static uint32_t lastDuty[40] = {0};

  float speed = constrain(fabsf(cmd), 0.0f, 1.0f);
  if (speed < SPEED_DEADBAND) {                 // off -> constant low
    if (lastDuty[pin] != 0) { ledcWrite(pin, 0); lastDuty[pin] = 0; lastFreq[pin] = 0; }
    return;
  }
  // Remap: 0..1 -> MIN_SPEED_OUTPUT..1
  speed = MIN_SPEED_OUTPUT + (1.0f - MIN_SPEED_OUTPUT) * speed;
  float duty  = DUTY_MAX * speed;
  float minor = (duty < 0.5f) ? duty : (1.0f - duty);
  uint32_t f = (uint32_t)lroundf(minor * 1000000.0f / PULSE_US);
  uint32_t d = (uint32_t)lroundf(duty * LEDC_MAXDUTY);

  if (f != lastFreq[pin]) {                      // frequency change: reloads counter, do sparingly
    ledcChangeFrequency(pin, f, LEDC_RES);
    ledcWrite(pin, d);
    lastFreq[pin] = f; lastDuty[pin] = d;
  } else if (d != lastDuty[pin]) {               // duty-only change: latches cleanly at boundary
    ledcWrite(pin, d);
    lastDuty[pin] = d;
  }
  // else: unchanged -> leave the timer free-running (this is what fixes the glitch)
}

// ---- shared actuator path: trim -> dir pins -> PFM ------------------------
// Used by BOTH sources so the physical L/R imbalance trim always applies.
void driveMotors(float left, float right) {
  // Antisymmetric imbalance trim (constant across range, flips with direction).
  float lt = left;
  if (left  < 0.0f) lt = left  + TRIML;   // slows down left if the motor is in reverse
  float rt = right;
  if (right > 0.0f) rt = right - TRIMR;   // slows down the right side if the motor is forwards
  if ((lt < 0.0f) != (left  < 0.0f)) lt = 0.0f; // trim must not reverse a motor near standstill
  if ((rt < 0.0f) != (right < 0.0f)) rt = 0.0f;
  left = lt; right = rt;

  // Direction outputs: HIGH when that side spins forward, 0V otherwise.
  digitalWrite(LEFT_DIR_PIN,  left  > 0.0f ? HIGH : LOW);
  digitalWrite(RIGHT_DIR_PIN, right > 0.0f ? HIGH : LOW);

  writeMotor(LEFT_PWM_PIN,  left);              // magnitude -> frequency
  writeMotor(RIGHT_PWM_PIN, right);
}

void driveStop() {
  // Route through writeMotor (cmd 0 -> below deadband -> constant low) so its
  // change-detection cache resets too; a direct ledcWrite(0) would leave the
  // cache stale and the next identical command would be a no-op (dead output).
  writeMotor(LEFT_PWM_PIN,  0.0f);
  writeMotor(RIGHT_PWM_PIN, 0.0f);
  digitalWrite(LEFT_DIR_PIN,  LOW);     // defined direction state on failsafe
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

// ---- SBUS: decode one frame into the RC command --------------------------
void processFrame(const uint8_t* b) {
  uint16_t ch[16];
  decodeChannels(b, ch);

  float throttle = norm(ch[1])*norm(ch[1])*norm(ch[1]);
  float steer = 0.8*((norm(ch[0]))*(norm(ch[0]))*(norm(ch[0])));
  if (INVERT_STEER) steer = -steer;
  float left  = throttle + steer;
  float right = throttle - steer;

  float m = max(fabsf(left), fabsf(right));     // keep turn ratio if it overshoots
  if (m > 1.0f) { left /= m; right /= m; }

  rcLeft  = left;
  rcRight = right;
  rcFlags = b[23];                              // flag byte: failsafe / frame-lost / TX-off
}

// ---- host: parse one command line ----------------------------------------
void parseCommand(const char* s) {
  if (s[0] == 'S' || s[0] == 's') {             // explicit stop (still a heartbeat)
    jLeft = 0; jRight = 0;
    jBed = 0; jBedValid = true;                 // stop == commanded bed-stop frame
    lastCmdMs = millis();
    return;
  }
  float l, r, b;
  // bed parsed as a float so "1", "1.0" and "-1" all work; sign -> -1/0/+1.
  int n = sscanf(s, "M %f %f %f", &l, &r, &b);
  if (n < 2) n = sscanf(s, "%f %f %f", &l, &r, &b);
  if (n >= 2) {
    jLeft  = constrain(l, -1.0f, 1.0f);
    jRight = constrain(r, -1.0f, 1.0f);
    if (n == 3) {                               // bed field present -> bed command
      jBed = (b > 0.5f) ? 1 : ((b < -0.5f) ? -1 : 0);
      jBedValid = true;
    } else {
      // 2-arg line (spec is 3-arg, so this is a truncated/legacy line): treat
      // the missing field as bed STOP rather than invalidating the channel --
      // one malformed line must not silently flip TX2 back to echo while the
      // bed is mid-motion. jBedValid is left untouched: if no bed command was
      // ever issued (pure legacy host), TX2 behavior is unchanged.
      jBed = 0;
    }
    lastCmdMs = millis();
  }
}

void pollHost() {
  static char line[48];
  static uint8_t li = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (li > 0) { line[li] = '\0'; parseCommand(line); li = 0; }
    } else if (li < sizeof(line) - 1) {
      line[li++] = c;
    } else {
      li = 0;                                    // overflow -> drop line, resync
    }
  }
}

// ---- bed output on UART2 TX ----------------------------------------------
// True while the host bed channel owns TX2: a bed command has been issued at
// some point (jBedValid) and the transmitter is not in control (identical test
// to controlStep's autoAllowed). While this holds, the raw SBUS echo is
// suppressed.
//
// Deliberately does NOT require the host command to be fresh: on host timeout
// the wheels get an ACTIVE stop (driveStop), so the bed must get an active stop
// too -- ownership is kept and serviceBedOutput forces the stop byte. Dropping
// ownership instead would leave the bed's fate to the bed controller's own
// line-loss failsafe, which we cannot assume exists.
bool bedOwnsTx() {
  unsigned long now = millis();
  bool rcAlive = (now - lastFrameMs) < SIGNAL_TIMEOUT;
  bool txOff   = rcAlive && (rcFlags & RC_GONE_FLAGS);
  return jBedValid && ((!rcAlive) || txOff);
}

// Emit the bed frame every BED_FRAME_US. First frame after taking ownership goes
// out immediately. Never blocks: the whole 25-byte frame is handed to the driver
// in one write() so the bytes leave back-to-back with no inter-byte gaps (the
// 128-byte TX FIFO swallows it whole and write() returns before the line is
// clocked out) -- but only once the FIFO has room for all 25, otherwise the slot
// is skipped rather than stalling the loop (a stalled loop is a late failsafe).
//
// The deadline advances by exactly BED_FRAME_US instead of being reset to now(),
// so scheduling jitter does not accumulate into drift. If the loop falls far
// enough behind to miss whole slots (or micros() wraps at ~71 min), the phase is
// re-anchored to now() rather than firing a burst of catch-up frames.
void serviceBedOutput() {
  static unsigned long nextBedUs = 0;
  static bool owned = false;

  if (!bedOwnsTx()) { owned = false; return; }

  unsigned long now = micros();
  if (owned) {
    if ((long)(now - nextBedUs) < 0) return;                  // not due yet
    if ((long)(now - nextBedUs) > (long)BED_FRAME_US) nextBedUs = now;  // way late -> re-anchor
  } else {
    nextBedUs = now;                                          // first frame: send immediately
  }
  if (SBUS.availableForWrite() < (int)sizeof(BED_FRAME_TEMPLATE)) return;

  // Stale host command -> commanded bed STOP (mirror of driveStop on the
  // wheels). Direction bytes go out only while the command is fresh.
  bool cmdAlive = (millis() - lastCmdMs) < CMD_TIMEOUT;
  int  bed      = cmdAlive ? jBed : 0;

  uint8_t frame[sizeof(BED_FRAME_TEMPLATE)];
  memcpy(frame, BED_FRAME_TEMPLATE, sizeof(frame));
  frame[BED_BYTE_INDEX] = (bed > 0) ? BED_BYTE_UP
                        : (bed < 0) ? BED_BYTE_DOWN
                                    : BED_BYTE_STOP;
  SBUS.write(frame, sizeof(frame));

  nextBedUs += BED_FRAME_US;
  owned = true;
}

// ---- decide who drives, then act -----------------------------------------
void controlStep() {
  unsigned long now = millis();
  bool rcAlive  = (now - lastFrameMs) < SIGNAL_TIMEOUT;   // SBUS frames arriving
  bool cmdAlive = (now - lastCmdMs)   < CMD_TIMEOUT;      // host command fresh
  bool txOff    = rcAlive && (rcFlags & RC_GONE_FLAGS);   // frames present, but TX off or failsafed

  // Autonomy allowed only when the transmitter is not in control.
  bool autoAllowed = (!rcAlive) || txOff;

  if (autoAllowed) {
    if (cmdAlive) driveMotors(jLeft, jRight);   // host drives
    else          driveStop();                  // no fresh command -> stop
  } else {
    driveMotors(rcLeft, rcRight);               // transmitter live -> RC has exclusive control
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(CMD_BAUD);                                                // host command link (USB / UART0)
  // UART2 now has a TX pin so the incoming SBUS stream can be echoed out (or the
  // synthetic bed stream generated). The invert flag applies to BOTH directions,
  // so anything leaving SBUS_TX_PIN carries the same inverted polarity the
  // receiver sends -- a drop-in copy of the input for a second SBUS device or a
  // logic analyzer.
  SBUS.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, SBUS_TX_PIN, SBUS_INVERTED); // RC (UART2)

  // separate channels -> separate timers -> independent L/R frequencies
  ledcAttachChannel(LEFT_PWM_PIN,  1000, LEDC_RES, LEFT_CH);
  ledcAttachChannel(RIGHT_PWM_PIN, 1000, LEDC_RES, RIGHT_CH);

  pinMode(LEFT_DIR_PIN,  OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  driveStop();                                  // also sets DIR pins low
  // No banner print: Serial is the command link now, not a debug console.
}

void loop() {
  // 1) ingest host commands FIRST (USB) -> jLeft/jRight/jBed. Doing this before
  //    the SBUS pass means a just-arrived bed command suppresses the echo in
  //    the SAME iteration -- otherwise echoed bytes and the first bed frame
  //    could interleave on TX2 once at the handover.
  pollHost();

  // 2) ingest SBUS frames -> rcLeft/rcRight/rcFlags, echoing the raw stream on
  //    TX2 unless the bed channel owns it. When ownership is RELEASED, the echo
  //    does not resume mid-frame: echoLive stays false until a frame boundary
  //    (inter-frame gap followed by an 0x0F header), so the first thing the bed
  //    controller sees after a handover is a whole frame, not a headless tail.
  static bool echoLive = true;
  bool bedOwned = bedOwnsTx();
  if (bedOwned) echoLive = false;
  while (SBUS.available()) {
    unsigned long now = micros();
    bool atGap = (now - lastByteUs > FRAME_GAP_US);
    if (atGap) idx = 0;
    lastByteUs = now;
    uint8_t byte = SBUS.read();

    if (!bedOwned && !echoLive && atGap && byte == 0x0F) echoLive = true;

    // Echo before the framing logic, so the copy is byte-for-byte -- including
    // bytes this decoder discards while resyncing. Guarded by availableForWrite()
    // because SBUS.write() BLOCKS when the TX FIFO fills, and a stalled loop is a
    // late failsafe; dropping a byte is the cheaper failure. At 100 kbaud 8E2 a
    // 25-byte frame is ~3 ms of line time against a 7 ms frame period, so the
    // 128-byte FIFO never actually backs up in normal operation.
    if (SBUS_ECHO && echoLive && !bedOwned && SBUS.availableForWrite() > 0) SBUS.write(byte);

    if (idx == 0 && byte != 0x0F) continue;
    buf[idx++] = byte;
    if (idx == 25) {
      idx = 0;
      if (buf[0] == 0x0F && buf[24] == 0x00) { processFrame(buf); lastFrameMs = millis(); }
    }
  }

  // 3) bed channel: generate SBUS frames on TX2 when the host owns it
  serviceBedOutput();

  // 4) arbitrate and drive (or fail safe)
  controlStep();
}
