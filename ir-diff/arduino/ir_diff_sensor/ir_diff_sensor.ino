int16_t const Tmin = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(57600);
}

void far(bool enable) {
  if(enable){
    digitalWrite(0, HIGH);
  }else {
    digitalWrite(0, LOW);
  }
}

void near(bool enable) {
  if(enable){
    digitalWrite(1, HIGH);
  }else {
    digitalWrite(1, LOW);
  }
}

uint16_t sensor_avg() {
  int constexpr n = 8;
  analogRead(A0);
  uint16_t avg = 0;
  for(int i=0;i<n;++i) {
    avg += analogRead(A0);
  }
  return avg / n;
}

void loop() {
  near(0);
  far(0);
  _delay_ms(1);
  uint16_t v_ambient = sensor_avg();
  far(1);
  _delay_ms(1);
  uint16_t v_far = sensor_avg();
  far(0);
  near(1);
  _delay_ms(1);
  uint16_t v_near = sensor_avg();
  near(0);
  far(0);

  {
   
   int16_t const c_far = v_far - v_ambient;
   int16_t const c_near = v_near - v_ambient;
   if ((c_far>Tmin) || (c_near>Tmin)) {
    int32_t diff = c_far - c_near;
    if(abs(diff) < 2) {
      digitalWrite(7, HIGH);
    } else {
      digitalWrite(7, LOW);
    }
   }else{
    digitalWrite(7, LOW);
   }
   
  }

  if(Serial) {
    Serial.print(v_ambient, DEC);
    Serial.print(';');
    Serial.print(v_far, DEC);
    Serial.print(';');
    Serial.println(v_near, DEC);
  }
  
}
