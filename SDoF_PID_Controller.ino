//PID controller for SDoF pendulum
//Control at 25Hz, continuous logging from IMU stored in global variables


#include "ICM_20948.h"
#include "Servo.h"
#include "math.h"
#include "Wire.h"

#define SERIAL_PORT Serial

#define WIRE_PORT Wire
#define AD0_VAL   1 

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

Servo ESC;

float theta = 0;
float Kp = 0.206;
float Ki = 0.175;
float Kd = 0.0437;
volatile float P;
volatile float I;
volatile float D;
volatile float integrator = 0;
volatile float error;
float setpoint = 90;
volatile float output;

float accXbias;
float accYbias;
float accZbias;
float gyrXbias;
float gyrYbias;
float gyrZbias;

float timer;

int bias_points = 500;
volatile int counter = 0;
int throttle;

volatile int motor = 0;

void setup() {

  pinMode(8,OUTPUT);

  ESC.attach(9,1000,2000);
  ESC.write(180);
  delay(1000);
  digitalWrite(8,HIGH);
  delay(2500);
  ESC.write(0);
  delay(1000);

  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  
  bool initialized = false;
  while( !initialized ){
    myICM.begin( WIRE_PORT, AD0_VAL );

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }

ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d111bw4_n136bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d119bw5_n154bw3;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
                                          
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if( myICM.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, true );
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));


  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));

Serial.println("Calibrating");

while(counter<bias_points){
    if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units
    //Serial.println(myICM.gyrX());
    accXbias = accXbias + myICM.accX();
    accYbias = accYbias + myICM.accY();
    accZbias = accZbias + myICM.accZ();
    gyrXbias = gyrXbias + myICM.gyrX();
    gyrYbias = gyrYbias + myICM.gyrY();
    gyrZbias = gyrZbias + myICM.gyrZ();
    counter = counter + 1;
    delay(10);
  }else{
    Serial.println("Waiting for data");
    delay(100);
  }
}
counter = 0;

accXbias = accXbias/bias_points;
accYbias = accYbias/bias_points;
accZbias = accZbias/bias_points;
gyrXbias = gyrXbias/bias_points;
gyrYbias = gyrYbias/bias_points;
gyrZbias = gyrZbias/bias_points;
Serial.println("accXbias");
Serial.println(accXbias);
Serial.println("accYbias");
Serial.println(accYbias);
Serial.println("accZbias");
Serial.println(accZbias);
Serial.println("gyrXbias");
Serial.println(gyrXbias);
Serial.println("gyrYbias");
Serial.println(gyrYbias);
Serial.println("gyrZbias");
Serial.println(gyrZbias);

noInterrupts(); // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 250; // compare match register = 16 MHz/25Hz/1024 = 655
  //shooting for 25Hz but limited to 8 bit register on timer 2
  //going to use counter in ISR to trigger ever 5 interupt cycles to maintain 50Hz servo()
  //function on timer 1

  TCCR2A |= (1 << WGM21); // compare-then-clear mode
  TCCR2B |= (1 << CS22);
  //TCCR2B |= (1 << CS20); // set prescaler to 1024

  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupts

  interrupts(); // enable the interrupts!

delay(1000);
timer = millis();

}

void loop() {
  
  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
//    printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units
    
    theta = theta + (((myICM.gyrX()-gyrXbias)*(timer - micros())/1000000));
        timer = micros();
    //Serial.println(theta);
    delay(10);
    motor = 1;
  }
  
}


//ISR for control computation

ISR (TIMER2_COMPA_vect) {

if (counter >= 40 && motor == 1){
  error = setpoint - theta;

  P = Kp*error;
  D = Kd*myICM.gyrX();

  integrator = integrator + (error/25);
  I = Ki*integrator;

  output = P + I + D + 9; //minimum ESC output is 9

  counter  = 0;
  //Serial.println("ISR");
  //output = map(output, 0, 100, 0, 180);

  if(output<0){
    output = 0;
  }
  if(output>80){
    output = 80;
  } 
  //Serial.print("Theta ");
  //Serial.println(theta);
  //Serial.print("  OUTPUT ");
  //Serial.println(output);
   // Serial.print("P ");
  //Serial.println(P);
  //Serial.print("D ");
  //Serial.println(D);
  //Serial.println("ON");
  ESC.write(output);
  Serial.println(output);
  
  }
else if((counter % 20) == 0){
Serial.println(theta);
}

counter = counter + 1;
//Serial.println("else");

}































// helper functions to print the data nicely

void printPaddedInt16b( int16_t val ){
  if(val > 0){
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  }else{
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
