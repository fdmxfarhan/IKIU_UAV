int volt = 0, x_set = 0, y_set = 0, z_set = 0, x=0, y=0;
char recv;
int buff, cnt = 0;
int speed = 30;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  volt = map(analogRead(A0), 0, 1023, 128, 255);
  if (Serial1.available() > 0) {
    recv = Serial1.read();
    if (recv == 'J') {
      z_set = 0;
      while (buff != '\n') {
        // delay(10);
        if (Serial1.available() > 0) {
          buff = Serial1.read();
          if (buff != '\n' && buff >= '0' && buff <= '9') {
            z_set = z_set * 10 + (buff - '0');

          }
        }
      }
      // Serial.println(z_set);
      buff = ' ';
    }
    else if(recv == 'X') y_set--;
    else if(recv == 'Y') y_set++;
    else if(recv == 'M') x_set--;
    else if(recv == 'N') x_set++;
    else if(recv == 'F') x = -speed;
    else if(recv == 'G') x = speed;
    else if(recv == 'L') y = -speed;
    else if(recv == 'R') y = speed;
    else if(recv == 'S') {
      x = 0;
      y = 0;
    }

  }
  analogWrite(6 , 128 + z_set + x_set + y_set + x + y);  // FL
  analogWrite(9 , 128 + z_set + x_set - y_set + x - y);  // FR
  analogWrite(5 , 128 + z_set - x_set - y_set - x - y);  // BR
  analogWrite(10, 128 + z_set - x_set + y_set - x + y);  // BL
}
