///SENSOR RPM///
int sensor = 2;
unsigned long start_time = 0;
unsigned long end_time = 0;
int steps=0;
float steps_old=0;
float temp=0;
float rpm=0;

///MOTOR Konveyor///
const byte INA = 6;  // INA: Clockwise input
const byte INB = 7;  // INB: Counter-clockwise input
const byte PWM = 9;  // PWM input
const byte CS = 1;  // CS: Current sense ANALOG input
const byte EN = 5 ;  // EN: Status of switches output (Analog pin)

///PID///
float Kp = 2.8;
float Ki = 0.5;
float Kd = 0.3;
uint32_t lastTime;
float lastError;
float pwm;
float setPoint = 50;

///MOTOR PINTU///
#define IN1 50 // deklarasi pin IN1
#define IN2 51 // deklarasi pin IN2
#define IN3 52  // deklarasi pin IN3
#define IN4 53  // deklarasi pin IN4

///Button///
#define button1 14 // deklarasi pin button1
#define button2 15 // deklarasi pin button2
int tombol1, tombol2;

//IO sensor atas
#define echoPin 30 
#define trigPin 31 
//IO Sensor kiri
#define echoPin2 32 
#define trigPin2 33 
//IO Sensor kanan
#define echoPin3 34
#define trigPin3 35 
//RELAY//
#define RL1 16


float ukur, waktu, out;
float kecil,sedang,besar;
float Lambat,Sedang,Cepat;
float rule1, rule2a, rule2b, rule3;
int keluaran;
///DEKLARASI HCSR///
int maximumRange, maximumRange2, maximumRange3 = 200;
int minimumRange, minimumRange2, minimumRange3 = 00; 
long duration, distance, duration2, distance2, duration3, distance3 ; 
int ukuratas, lebar ;

//////////////////////////////////PROSES HCSR//////////////////////////
unsigned char  atas(){
digitalWrite(trigPin, LOW);delayMicroseconds(2);
digitalWrite(trigPin, HIGH);delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH); 
distance = duration/58.2;
ukuratas = 46-distance;
return ukuratas;
}

unsigned char  kiri(){
digitalWrite(trigPin2, LOW);delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
duration2 = pulseIn(echoPin2, HIGH);
distance2 = duration2/58.2;
return distance2; 
}

unsigned char  kanan(){
digitalWrite(trigPin3, LOW);delayMicroseconds(2);
digitalWrite(trigPin3, HIGH);delayMicroseconds(10);
digitalWrite(trigPin3, LOW);
duration3 = pulseIn(echoPin3, HIGH);
distance3 = duration3/58.2; 
return distance3;
}
void HCSR(){
  atas(), kiri(), kanan();
  lebar = 51 - (distance2+distance3);
  ukur=(ukuratas*lebar);
  return ukur;
}
////////////////////////////////////////////////////

unsigned char ukurkecil(){
  if (ukur <= 300){kecil =1;}
  else if (ukur >=300 && ukur <=450){kecil=(450-ukur)/150;}
  else if (ukur >= 450){kecil =0;}
  return kecil;
}
unsigned char ukursedang(){
  if (ukur <= 300){sedang =0;}
  else if (ukur >=300 && ukur <=450){sedang=(ukur-300)/150;}
  else if (ukur >=450 && ukur <=600){sedang=(600-ukur)/150;}
  else if (ukur >= 600){sedang =0;}
  return sedang;
}
unsigned char ukurbesar (){
  if (ukur <=450 ){besar =0;}
  else if (ukur >=450 && ukur <=600){besar=(ukur-450)/150;}
  else if (ukur >= 600){besar =1;}
  return besar;
}
unsigned char waktuLambat(){
  if (waktu <= 100){Lambat =1;}
  else if (waktu >=100 && waktu <=150){Lambat=(150-waktu)/50;}
  else if (waktu >= 150){Lambat =0;}
  return Lambat;
}
unsigned char waktuSedang(){
  if (waktu <= 100){Sedang =0;}
  else if (waktu >=100 && waktu <=150){Sedang=(waktu-100)/50;}
  else if (waktu >=150 && waktu <=200){Sedang=(200-waktu)/50;}
  else if (waktu >= 200){Sedang =0;}
  return Sedang;
}
unsigned char waktuCepat (){
  if (waktu <= 100){Lambat =1;}
  else if (waktu >=150 && waktu <=200){Cepat=(waktu-150)/50;}
  else if (waktu >= 200){Cepat =0;}
  return Cepat;
}
//Fuzzifikasi
void fuzzifikasi(){
  ukurkecil();
  ukursedang();
  ukurbesar();
  waktuLambat();
  waktuSedang();
  waktuCepat();
}
void fuzzy_rule (){
fuzzifikasi();
// jika ukur kecil maka motor lambat
rule1 = 150 - (kecil*50);
// jika ukur sedang maka motor sedang
rule2a = 100 + (sedang*50);
rule2b = 200 - (sedang*50);
// jika ukur besar maka motor cepat
rule3 = 150 + (besar*50);
//defuzifikasi
out = ((rule1*kecil) + (rule2a*sedang) + (rule2b*sedang) + (rule3*besar)) / (kecil+sedang+sedang+besar);
return out;
}

void RELAY(){
fuzzy_rule();
 if (out <= 130) {
    digitalWrite(RL1, HIGH);
    delay (30000);
    digitalWrite(RL1, LOW);
    delay (1000);}
    else if (out >= 130 && out <= 170){
    digitalWrite(RL1, HIGH);
    delay (35000);
    digitalWrite(RL1, LOW);
    delay (1000);}
    else if (out >= 170){
    digitalWrite(RL1, HIGH);
    delay (40000);
    digitalWrite(RL1, LOW);
    delay (1000);}
}

void setup() 
{
  Serial.begin(9600);
  pinMode(sensor,INPUT_PULLUP);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, HIGH);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(RL1, OUTPUT);

 // contoh kita memasukan nilai ukur 54 derajat

HCSR();
fuzzy_rule();
atas();
 // memanggil fungsi fuzzifikasi untuk menghitung keanggotaan masing2 variable
 Serial.print (" Lebar= ");
Serial.print(lebar);
Serial.println(" cm ");
Serial.print (" Tinggi= ");
Serial.print(ukuratas);
Serial.println(" cm ");
Serial.print (" luas= ");
Serial.print(ukur);
Serial.println(" cm2 ");
Serial.print("Kecil : ");
Serial.print(kecil);
Serial.println("t");
Serial.print("Sedang : ");
Serial.print(sedang);
Serial.println("t");
Serial.print("Besar : ");
Serial.println(besar);
Serial.print("Hasil DeFuzzy: ");
Serial.println(out);
 // contoh kita memasukan nilai ukur 54 derajat
}
 
void loop()
{
  putaran();
  rulePID(); 
  
  tombol1=digitalRead(button1);
  tombol2=digitalRead(button2);

  if(tombol1 == LOW){
  digitalWrite(INA, HIGH);
  digitalWrite(INB,LOW);
  analogWrite(PWM, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  }
  if(tombol2 == LOW){
  digitalWrite(INA, LOW);
  digitalWrite(INB,LOW);
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  
  RELAY();
  delay(1000); 
  digitalWrite(INA, HIGH);
  digitalWrite(INB,LOW);
  analogWrite(PWM, pwm);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay (20000);
  digitalWrite(INA, LOW);
  digitalWrite(INB,LOW);
  analogWrite(PWM, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); 
  //delay (10000);
  }


 //Serial.print(rpm);
 //Serial.println(" RPM ");

}
void putaran(){
 start_time=millis();
 end_time=start_time+1000;
 while(millis()<end_time)
 {
   if(digitalRead(sensor))
   {
    steps=steps+1; 
    while(digitalRead(sensor));
   }
}
    temp=steps-steps_old;
    steps_old=steps;
    rpm =(temp/20)*60;
    return rpm;
   // Serial.print(rps);
    //Serial.print("   ");
}


void rulePID(){
  float error = setPoint - rpm;
  float deltaError = error - lastError;
  float sigmaError = error + lastError;
  sigmaError = constrain(sigmaError, -1000, 1000);
  lastError = error;
  float P = Kp * error;
  float I = Ki * sigmaError;
  float D = Kd * deltaError;
  float PID = P + I + D;
  Serial.print(P);
  Serial.println("  ");
  pwm = pwm + PID;
  pwm = constrain(pwm, -20, 20);  
  return pwm;

}
