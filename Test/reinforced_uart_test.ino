// Variables para almacenar los datos de cada cámara
float dribbler_distance = 0;
float dribbler_angle = 0;
bool dribbler_ball_seen = false;
bool ball_captured = false;

float ball_distance = 0;
float ball_angle = 0;
float goal_angle = 0;
float goal_distance = 0;
bool open_ball_seen = false;
bool goal_seen = false;

// Buffers de lectura por cámara
String serial1_line = "";
String serial2_line = "";

void setup() {
  Serial.begin(115200);      // Solo para depuración
  Serial1.begin(115200);     // OpenMV 1
  Serial2.begin(115200);     // OpenMV 2
}

void loop() {
  readSerialLines();

  // Aquí puedes usar los datos actualizados
  // Por ejemplo:
  /*
  Serial.print("Dribbler ball seen: ");
  Serial.println(dribbler_ball_seen);
  Serial.print("Ball distance: ");
  Serial.println(ball_distance);
  */

  // Resto de tu lógica de comportamiento...
}

void readSerialLines() {
  // Leer desde Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      processSerial1(serial1_line);
      serial1_line = "";
    } else {
      serial1_line += c;
    }
  }

  // Leer desde Serial2
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processSerial2(serial2_line);
      serial2_line = "";
    } else {
      serial2_line += c;
    }
  }
}

void processSerial1(String line) {
  float dist, ang;
  int parsed = sscanf(line.c_str(), "%f %f", &dist, &ang);
  if (parsed == 2) {
    dribbler_distance = dist;
    Serial.print("ball_distance 1 ");
    Serial.println(dribbler_distance);
    dribbler_angle = ang;
    Serial.print("Angulo 1 ");
    Serial.println(dribbler_angle);
    dribbler_ball_seen = (dist != 0 && ang != 0);
    ball_captured = (dist <= 20 && ang == 0);
  }
}

void processSerial2(String line) {
  float dist, ang, g_ang, g_dist;
  int parsed = sscanf(line.c_str(), "%f %f %f %f", &dist, &ang, &g_ang, &g_dist);
  if (parsed == 4) {
    ball_distance = dist;
    Serial.print("ball_distance 2 ");
    Serial.println(ball_distance);
    ball_angle = ang;
    Serial.print("ball_distance 2 ");
    Serial.println(ball_angle);
    goal_angle = g_ang;
    goal_distance = g_dist;
    open_ball_seen = (dist != 0 && ang != 0);
    goal_seen = (g_ang != 0 && g_dist != 0);
  }
}
