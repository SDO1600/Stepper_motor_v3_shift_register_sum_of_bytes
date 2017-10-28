int dataPin = 51; // MOSI (data) line, пин подключен к DS входу 74HC595, pin ?9 for Nano
int latchPin = 53; // (SS) Пин подключен к ST_CP входу 74HC595, pin ?10 for Nano 
int clockPin = 52; //SCK line, пин подключен к SH_CP входу 74HC595, pin ?11 for Nano
int potPin1 = A0; // the potentiometer to control rotation speed
int potPin2 = A1; // the potentiometer to control rotation speed
bool array_data[2][4];
/*
// Для шагового режима
bool motorPhases[4][4] = { // [phase][pin]
// -------- pins ----------
// Winding    A  B  A  B
// Motor Pin  1  2  3  4
// Color      Bl Pi Ye Or
  {           1, 1, 0, 0},
  {           0, 1, 1, 0},
  {           0, 0, 1, 1},
  {           1, 0, 0, 1}
};

*/

void setup() { //устанавливаем режим OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
/*  digitalWrite(latchPin, LOW); // устанавливаем синхронизацию "защелки" на LOW
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);*/
  Serial.begin(9600);// Setup Baud rate as 9600
}

int phase1 = 0;
int phase2 = 0;
int _step1 = 1; // 0 -- no movement, 1 -- clockwise rotation, -1 -- counterclockwise rotation
int _step2 = 1; // 0 -- no movement, 1 -- clockwise rotation, -1 -- counterclockwise rotation
unsigned long timeout = 300; // the timeout [ms] to let the motor to do a rotation
unsigned long currTime = 0;
unsigned long prevTime = 0;
byte powerByte1_fwd[4] = {B11000000, B01100000, B00110000, B10010000}; // rotation forward
byte powerByte1_rev[4] = {B10010000, B00110000, B01100000, B11000000};  // reversed rotation
byte powerByte2_fwd[4] = {B00001100, B00000110, B00000011, B00001001}; // rotation forward 
byte powerByte2_rev[4] = {B00001001, B00000011, B00000110, B00001100};  // reversed rotation
byte rotByte[4];
void loop() {
// map(value, fromLow, fromHigh, toLow, toHigh)
//  timeout = map(analogRead(potPin), 0, 1023, 1, 50); // timeout in ms because it'll be used with millis()
  int potValue1 = analogRead(potPin1);
  int potValue2 = analogRead(potPin2);
  Serial.println(timeout);
  
  if (potValue1<330) rotByte[4] = powerByte1_rev[4];
//  {
//    for (int i=0;i<4;i++){
//        rotByte[i] = powerByte1_rev[i];
//    }
//    }

  if (potValue1>660) rotByte[4] = powerByte1_fwd[4];
  if (potValue2<330) rotByte[4] = rotByte[4] + powerByte2_rev[4];
  if (potValue1>660) rotByte[4] = rotByte[4] + powerByte2_fwd[4];
     
  if (currTime - prevTime >= timeout) {
//    int phase = 0;
//    int pins = 0;

    for (int j = 0; j < 4; j++){ 
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, rotByte[j]); // передаем последовательно на dataPin  
    Serial.println(rotByte[i]);
    digitalWrite(latchPin, HIGH); //"защелкиваем" регистр, тем самым устанавливая значения на выходах    
    delay(timeout);
  }
//    Serial.print("timeout [ms] = ");
//    Serial.print(timeout);
//    phase1 += _step1;
//    phase2 += _step2;
//    Serial.print("  phase1 [parrots] = ");
//    Serial.println(phase1);  
//    Serial.print("  phase2 [parrots] = ");
//    Serial.println(phase2);  
//    if (phase1 > 3) phase = 0; // replace 3 for 7 for half-step mode
//    if (phase1 < 0) phase = 3; // replace 3 for 7 for half-step mode
//    if (phase2 > 3) phase = 0; // replace 3 for 7 for half-step mode
//    if (phase2 < 0) phase = 3; // replace 3 for 7 for half-step mode
//    prevTime = currTime;
    Serial.println("yahoo!!");  
//    for (int i = 0; i < 4; i++) {  
//      digitalWrite(pins[i], ((motorPhases[phase][i] == 1) ? HIGH : LOW));     
//    
//    }
  }
  

}
//    for (int i = 0; i < 4; i++){ 
//    digitalWrite(latchPin, LOW);
//    shiftOut(dataPin, clockPin, LSBFIRST, rotByte[i]); // передаем последовательно на dataPin  
////    Serial.println(rotByte[i]);
//    digitalWrite(latchPin, HIGH); //"защелкиваем" регистр, тем самым устанавливая значения на выходах    
//    delay(timeout);
//  }
