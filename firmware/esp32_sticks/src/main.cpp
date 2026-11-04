// Drives the DJI RC-N1C stick/wheel sensor lines with filtered 12-bit PWM.
//
// Each output goes through a two-stage RC filter (10k/100n, twice) into the
// RC's main-board side of the cut sensor wire. The Jetson sends duty values
// over USB serial; the RC's own stick report (seen by the Jetson over the
// AOA link) closes the loop for calibration.
//
// Serial protocol, one ASCII line per command, 115200 (USB CDC, baud ignored):
//   D d0 d1 d2 d3 d4   set duties 0..4095 (roll pitch throttle yaw wheel), feeds the timeout
//   N d0 d1 d2 d3 d4   set and save the neutral duties (sticks centred)
//   L d0 d1 d2 d3 d4   set and save the landing duties (throttle full down, rest centred)
//   H                  hold neutral now and ignore D until R
//   G                  land now (landing duties) and ignore D until R
//   R                  resume after H/G or a SW1/SW2 press
//   ?                  status line
// With no D for TIMEOUT_MS the outputs return to neutral (centred sticks = hover).
// SW1 latches HOLD (neutral), SW2 latches LAND (landing duties); both work
// without the Jetson and need R to release.
#include <Arduino.h>
#include <Preferences.h>

static const int N_CH = 5;
static const uint8_t PWM_PINS[N_CH] = {2, 4, 6, 8, 10};  // roll, pitch, throttle, yaw, wheel
static const uint32_t PWM_HZ = 19531;                    // 80 MHz APB clock / 2^12 (the default 40 MHz XTAL clock can't)
static const uint8_t PWM_BITS = 12;
static const uint16_t DUTY_MAX = (1 << PWM_BITS) - 1;
static const uint32_t TIMEOUT_MS = 300;

static const uint8_t PIN_LED_ACTIVE = 17;  // LED1: Jetson in control
static const uint8_t PIN_LED_BEAT = 18;    // LED2: toggles on each D
static const uint8_t PIN_SW_HOLD = 12;     // SW1: latch hold (needs R to resume); polarity read at boot
static const uint8_t PIN_SW_LAND = 13;     // SW2: latch land (needs R to resume)

static uint16_t duty[N_CH];
static uint16_t neutral[N_CH];
static uint16_t landing[N_CH];
static bool landing_set = false;
static uint32_t last_cmd_ms = 0;
static bool held = false;   // H or SW1 latched
static bool landing_now = false;  // G or SW2 latched (takes priority over held)
static bool active = false; // D commands are fresh and not held
static Preferences prefs;
static bool sw_idle;  // SW1 level at boot = released
static bool sw2_idle; // SW2 level at boot = released
static bool pwm_ok = true;

static void write_outputs(const uint16_t *d) {
  if (!pwm_ok) return;
  for (int i = 0; i < N_CH; i++) ledcWrite(PWM_PINS[i], d[i]);
}

static int parse_values(const char *s, uint16_t *out) {
  int n = 0;
  while (n < N_CH) {
    while (*s == ' ') s++;
    if (!*s) break;
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s) break;
    out[n++] = (uint16_t)constrain(v, 0, DUTY_MAX);
    s = end;
  }
  return n;
}

static void print_status() {
  if (!pwm_ok) Serial.print("PWM_FAILED ");
  Serial.printf("S %s age_ms=%lu duty=%u,%u,%u,%u,%u neutral=%u,%u,%u,%u,%u landing=%s\n",
                landing_now ? "LANDING" : (held ? "HELD" : (active ? "ACTIVE" : "NEUTRAL")),
                (unsigned long)(millis() - last_cmd_ms),
                duty[0], duty[1], duty[2], duty[3], duty[4],
                neutral[0], neutral[1], neutral[2], neutral[3], neutral[4], landing_set ? "set" : "NOT SET");
}

static void handle_line(char *line) {
  uint16_t v[N_CH];
  switch (line[0]) {
    case 'D':
      if (parse_values(line + 1, v) != N_CH) { Serial.println("E need 5 values"); return; }
      if (held || landing_now) return;
      memcpy(duty, v, sizeof(duty));
      write_outputs(duty);
      last_cmd_ms = millis();
      active = true;
      digitalWrite(PIN_LED_BEAT, !digitalRead(PIN_LED_BEAT));
      return;
    case 'N':
      if (parse_values(line + 1, v) != N_CH) { Serial.println("E need 5 values"); return; }
      memcpy(neutral, v, sizeof(neutral));
      prefs.putBytes("neutral", neutral, sizeof(neutral));
      if (!active) write_outputs(neutral);
      Serial.println("OK N");
      return;
    case 'L':
      if (parse_values(line + 1, v) != N_CH) { Serial.println("E need 5 values"); return; }
      memcpy(landing, v, sizeof(landing));
      landing_set = true;
      prefs.putBytes("landing", landing, sizeof(landing));
      Serial.println("OK L");
      return;
    case 'H':
      held = true;
      Serial.println("OK H");
      return;
    case 'G':
      if (!landing_set) { Serial.println("E landing duties not set"); return; }
      landing_now = true;
      Serial.println("OK G");
      return;
    case 'R':
      held = false;
      landing_now = false;
      Serial.println("OK R");
      return;
    case '?':
      print_status();
      return;
    default:
      Serial.println("E unknown");
  }
}

void setup() {
  // Outputs first: until PWM runs the pins float, which the RC reads as a stick at an end stop.
  prefs.begin("sticks", false);
  for (int i = 0; i < N_CH; i++) neutral[i] = DUTY_MAX / 2;
  if (prefs.isKey("neutral")) prefs.getBytes("neutral", neutral, sizeof(neutral));
  if (prefs.isKey("landing")) landing_set = prefs.getBytes("landing", landing, sizeof(landing)) == sizeof(landing);
  ledcSetClockSource(LEDC_USE_APB_CLK);
  for (int i = 0; i < N_CH; i++) {
    pwm_ok &= ledcAttach(PWM_PINS[i], PWM_HZ, PWM_BITS);
    duty[i] = neutral[i];
  }
  write_outputs(neutral);

  pinMode(PIN_LED_ACTIVE, OUTPUT);
  pinMode(PIN_LED_BEAT, OUTPUT);
  pinMode(PIN_SW_HOLD, INPUT);  // the board has its own resistor on SW1
  sw_idle = digitalRead(PIN_SW_HOLD);
  pinMode(PIN_SW_LAND, INPUT);
  sw2_idle = digitalRead(PIN_SW_LAND);
  Serial.begin(115200);
}

void loop() {
  static char buf[96];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      if (len) handle_line(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  static bool sw_prev = sw_idle;
  bool sw = digitalRead(PIN_SW_HOLD);
  if (sw != sw_idle && sw_prev == sw_idle) held = true;  // pressed: left its boot level
  sw_prev = sw;

  static bool sw2_prev = sw2_idle;
  bool sw2 = digitalRead(PIN_SW_LAND);
  if (sw2 != sw2_idle && sw2_prev == sw2_idle && landing_set) landing_now = true;
  sw2_prev = sw2;

  if (landing_now) {
    active = false;
    if (memcmp(duty, landing, sizeof(duty))) {
      memcpy(duty, landing, sizeof(duty));
      write_outputs(duty);
    }
  } else {
    bool stale = millis() - last_cmd_ms > TIMEOUT_MS;
    if ((active && (held || stale)) || (!active && memcmp(duty, neutral, sizeof(duty)))) {
      active = false;
      memcpy(duty, neutral, sizeof(duty));
      write_outputs(duty);
    }
  }
  // LED1: on = Jetson in control, fast blink = landing latched, slow blink = hold latched
  uint32_t ms = millis();
  digitalWrite(PIN_LED_ACTIVE, landing_now ? (ms / 100) % 2 : (held ? (ms / 500) % 2 : active));
}
