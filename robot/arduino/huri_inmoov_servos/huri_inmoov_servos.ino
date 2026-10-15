// huri_inmoov_servos: serial line protocol -> servos (Arduino Mega, as in InMoov;
// on an Uno/Nano, change PINS to pins it has).
//
// Protocol (115200 baud, ASCII, one command per line, '\n'-terminated), sent
// by huri_inmoov_robot.drivers.SerialDriver:
//
//   S<id>:<deg>   move channel <id> to <deg> degrees (decimals accepted,
//                 rounded to the nearest degree), e.g. "S3:92.5"
//   D             detach every servo (no torque)
//   A             attach every servo again, at its last commanded angle
//
// Malformed lines and unknown channels are ignored. The board clamps every
// angle to the channel's [MIN_DEG, MAX_DEG] again: the last line of defense if
// the PC software misbehaves.
//
// Watchdog: no valid line for WATCHDOG_MS -> the servos hold their position
// and the built-in LED lights up (the board never moves a servo on its own:
// walking back to rest at a safe speed is the PC's job).
//
// TODO: fill PINS / REST_DEG / MIN_DEG / MAX_DEG with your wiring and
// calibration, in the channel order of the robot config's `channels:`.
// An InMoov usually has two boards (left/right): flash one sketch per board
// with its own tables, and give each its own channels (one SerialDriver per
// port is future work).

#include <Servo.h>

const unsigned long BAUD = 115200;
const unsigned long WATCHDOG_MS = 1000;

// channel:                   0 neck, 1 rothead, 2 jaw
const uint8_t PINS[] = {12, 13, 26};
const int REST_DEG[] = {90, 90, 10};
const int MIN_DEG[] = {20, 20, 10};
const int MAX_DEG[] = {160, 160, 25};
const uint8_t N = sizeof(PINS) / sizeof(PINS[0]);

Servo servos[N];
int angle[N];
bool attached = false;
unsigned long lastLine = 0;
char line[32];
uint8_t len = 0;
bool overflow = false;  // dropping the rest of a too-long line

void attachAll() {
  for (uint8_t i = 0; i < N; i++) {
    servos[i].write(angle[i]);  // sets the pulse before attach: no jump
    servos[i].attach(PINS[i]);
  }
  attached = true;
}

void detachAll() {
  for (uint8_t i = 0; i < N; i++) servos[i].detach();
  attached = false;
}

void handle(char *cmd) {
  if (cmd[0] == 'D' && cmd[1] == '\0') {
    detachAll();
  } else if (cmd[0] == 'A' && cmd[1] == '\0') {
    attachAll();
  } else if (cmd[0] == 'S') {
    char *colon = strchr(cmd, ':');
    if (colon == NULL) return;
    *colon = '\0';
    char *end;
    long id = strtol(cmd + 1, &end, 10);
    if (end == cmd + 1 || *end != '\0' || id < 0 || id >= N) return;
    float deg = atof(colon + 1);
    int d = (int)(deg + 0.5f);
    d = constrain(d, MIN_DEG[id], MAX_DEG[id]);
    angle[id] = d;
    if (attached) servos[id].write(d);
  } else {
    return;
  }
  lastLine = millis();
}

void setup() {
  Serial.begin(BAUD);
  for (uint8_t i = 0; i < N; i++) angle[i] = REST_DEG[i];
  pinMode(LED_BUILTIN, OUTPUT);
  attachAll();
  lastLine = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      line[len] = '\0';
      if (!overflow) handle(line);
      len = 0;
      overflow = false;
    } else if (c != '\r' && !overflow) {
      if (len < sizeof(line) - 1) {
        line[len++] = c;
      } else {
        overflow = true;  // line too long: drop it whole
      }
    }
  }
  digitalWrite(LED_BUILTIN, millis() - lastLine > WATCHDOG_MS ? HIGH : LOW);
}
