const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 6;
const int IN3 = 10;
const int IN4 = 11;

const unsigned long TIMEOUT_MS = 400;
unsigned long lastCommandTime = 0;

const int BUFFER_SIZE = 16;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  stopMotors();
}

void loop() {
  readSerialCommand();

  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stopMotors();
  }
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      inputBuffer[bufferIndex] = '\0';
      parseCommand(inputBuffer);
      bufferIndex = 0;
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex] = c;
      bufferIndex++;
    }
  }
}

void parseCommand(char *line) {
  char *commaPos = strchr(line, ',');
  if (commaPos == NULL) {
    return;
  }

  *commaPos = '\0';
  int leftSpeed = atoi(line);
  int rightSpeed = atoi(commaPos + 1);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
  lastCommandTime = millis();
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed;
  }
  analogWrite(ENA, speed);
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed = -speed;
  }
  analogWrite(ENB, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
