
// Project Black-Box 
// Black box designed for vehicles to record acceleration, velocity, displacement, and interactions between the driver and other vehicle
// features/controls such as windows, radio, accelerator pedal, and brake pedal.

#include <Wire.h> //MPU Library used to set up device and establish connection

// Calculate actual counter frequency 16Mhz/prescales
const long counterFreq = 16e6/256;  

// Gyro & Accel calculations 
const int delayTime = 1;
int counter = 0;

// Arrays for briefly storing data
float rotation[3][10];
float acceleration[3][10];
float velocity[3][10];
float displacement[3][10];

// Timer variables
volatile unsigned long cycles = 0;
long period = 65535;
unsigned long int oldSeconds = -1; // Keeps track of time to detect when new second is reached

//Interupt variables
bool interuptDetected = false;
String whichDistraction = ""; //Stores name of interupt that was triggered
int interuptInitialTime = 0;
int interuptFinalTime = 0;
int accumulativeBrake = 0;

//Other
int brakePadGood = 3;
int brakePadLow = 4;
int brakePadReplace = 5;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() //Main Setup Code
{
  Serial.begin(38400); //connection to serial monitor
  Wire.begin(); //initializes connection with library the MPU (accelerometer/gyroscope) needs to function
  setupMPU(); // establish connection with mpu and set up registers for our preferences 
  analogRead(3); //set analog pin three to read voltage (for detection of which button is being pressed)

  //Set initial conditions for brake pad detection light and define each pin as an output
  pinMode(brakePadGood, OUTPUT);
  digitalWrite(brakePadGood, HIGH);
  pinMode(brakePadLow, OUTPUT);
  pinMode(brakePadReplace, OUTPUT);

  attachInterrupt(0, distractedDriving, CHANGE); // interrupt 0 is mapped to pin 2 on the Uno, measure rising edges only
  cli();
  // clear bits for timer 0 
  TCCR1A = 0;
  TCCR1B = 0;
  // set timer prescaling to clk/256
  TCCR1B = ( 1 << 2 );
  TIMSK1 |= ( 7 << 0 ); // Enable timer interupts
  OCR1A = -1; //Set interupt condition
  OCR1B = period - 1; //Set interupt condition
  sei();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() { // Main program code

   unsigned long int seconds = 0; //timer calculated in seconds
   unsigned int minutes = 0;
   unsigned int hours = 0;
   duration(seconds, minutes, hours); // Calculate updated time values

   if ( counter == 10) // resets the counter so it does not store data beyond the bounds of the array 
  {
   counter = 0;
  }

   if (seconds == (oldSeconds + 1)) // make so it outputs movement data once per second!
   { 
    // Gyro & Accel storage
    long accelX, accelY, accelZ;
    float gForceX, gForceY, gForceZ;
    long gyroX, gyroY, gyroZ;
    float rotX, rotY, rotZ;
    float velocityX, velocityY, velocityZ;
    float displacementX, displacementY, displacementZ;

    //printData(seconds, minutes, hours); //Print all data that would be stored in eeprom / hard memory
    
    recordGyroRegisters(rotX, rotY, rotZ, gyroX, gyroY, gyroZ); // stores rotation data from the gyroscope
    recordAccelRegisters(gForceX, gForceY, gForceZ, accelX, accelY, accelZ); // stores acceleration data from the accelerometer
   
    getVelocity(gForceX, gForceY, gForceZ, velocityX, velocityY, velocityZ);  // converts acceleration data to velocity
    getDisplacement(displacementX, displacementY, displacementZ, velocityX, velocityY, velocityZ);  // converts acceleration data to displacement
    
    storeGyroData(rotX, rotY, rotZ); // stores X,Y,Z components of rotation in a matrix
    storeAccelData(gForceX, gForceY, gForceZ);  // stores X,Y,Z components of acceleration in a matrix
    storeVeloData(velocityX, velocityY, velocityZ); // stores X,Y,Z components of velocity in a matrix
    storeDispData(displacementX, displacementY, displacementZ);  // stores X,Y,Z components of displacement in a matrix
    
    printData(seconds, minutes, hours); //Print all data that would be stored in eeprom / hard memory
  
    counter++; // increments counter to next position in the array for storage/display of values
  
    oldSeconds++;
   }

  if (interuptDetected) 
  {
   whichInterupt(seconds, minutes, hours); 
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Timer functions

void duration(unsigned long int& seconds, unsigned int& minutes, unsigned int& hours)
{ 
  seconds = ( (TCNT1 + (cycles * period)) / counterFreq );
  minutes = seconds / 60;
  hours = minutes / 60;
}

void timeStamp(unsigned long int seconds, unsigned int minutes, unsigned int hours)
{
  Serial.print("Time: ");
  Serial.print(hours);
  Serial.print("hr ");
  Serial.print(minutes - ( hours * 60 ) );
  Serial.print("min ");
  Serial.print( seconds - (minutes * 60) - (hours * 60));
  Serial.println("sec ");
}


ISR (TIMER1_COMPB_vect)
{
  TCNT1 = 0;
  cycles++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

// reads acceleration data
void recordAccelRegisters(float& gForceX, float& gForceY, float& gForceZ, long& accelX, long& accelY, long& accelZ) {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData(gForceX, gForceY, gForceZ, accelX, accelY, accelZ);
}

// scales acceleration data read
void processAccelData(float& gForceX, float& gForceY, float& gForceZ, long accelX, long accelY, long accelZ){
  gForceX = accelX / 16384.0 * 9.81;
  gForceY = accelY / 16384.0 * 9.81; 
  gForceZ = (accelZ / 16384.0 * 9.81) - 9.81;
}

// reads gyroscope data
void recordGyroRegisters(float& rotX, float& rotY, float& rotZ, long& gyroX, long& gyroY, long& gyroZ) {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData(rotX, rotY, rotZ, gyroX, gyroY, gyroZ);
}

// scales gyroscope data
void processGyroData(float& rotX, float& rotY, float& rotZ, long gyroX, long gyroY, long gyroZ) {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

// stores X,Y,Z components of rotation in a matrix
void storeGyroData(float rotX, float rotY, float rotZ)
{
  rotation[0][counter] = rotX;
  rotation[1][counter] = rotY;
  rotation[2][counter] = rotZ;
}

// stores X,Y,Z components of acceleration in a matrix
void storeAccelData(float gForceX, float gForceY, float gForceZ)
{
  acceleration[0][counter] = gForceX;
  acceleration[1][counter] = gForceY;
  acceleration[2][counter] = gForceZ;
}

// stores X,Y,Z components of velocity in a matrix
void storeVeloData(float velocityX, float velocityY, float velocityZ)
{
  velocity[0][counter] = velocityX;
  velocity[1][counter] = velocityY;
  velocity[2][counter] = velocityZ;
}

// stores X,Y,Z components of displacement in a matrix
void storeDispData(float displacementX, float displacementY, float displacementZ)
{
  displacement[0][counter] = displacementX;
  displacement[1][counter] = displacementY;
  displacement[2][counter] = displacementZ;
}

// convert to m/s^2 (g's of force to acceleration) and calculate speed (*2)
void getVelocity(float gForceX, float gForceY, float gForceZ, float& velocityX, float& velocityY, float& velocityZ) {
  velocityX = ((gForceX) * delayTime); 

  velocityY = ((gForceY) * delayTime);

  velocityZ = ((gForceZ) * delayTime);
}

// convert to m/s^2 (g's of force to acceleration) and calculate speed (*2)
void getDisplacement(float& displacementX, float& displacementY, float& displacementZ, float velocityX, float velocityY, float velocityZ) {
  displacementX = (velocityX * delayTime); 
  
  displacementY = (velocityY * delayTime);

  displacementZ = (velocityZ * delayTime);
}

// prints data
void printData(unsigned long int seconds, unsigned int minutes, unsigned int hours) 
{
  Serial.println("////////////////////////////////////////////////////////////////////////////////////////////");
  timeStamp(seconds, minutes, hours);

  //Print Gyro 
  Serial.print("Gyro: ");
  Serial.print("X = ");
  Serial.print(rotation[0][counter]);
  Serial.print("deg Y = ");
  Serial.print(rotation[1][counter]);
  Serial.print("deg Z = ");
  Serial.print(rotation[2][counter]);
  Serial.println("deg");

  //Print Acceleration 
  Serial.print("Acceleration: ");
  Serial.print("X = ");
  Serial.print(acceleration[0][counter]);
  Serial.print("m/s^2 Y = ");
  Serial.print(acceleration[1][counter]);
  Serial.print("m/s^2 Z = ");
  Serial.print(acceleration[2][counter]);
  Serial.println("m/s^2");

  //Print Velocity
  Serial.print("Velocity: ");
  Serial.print("X = ");
  Serial.print(velocity[0][counter]);
  Serial.print("m/s Y = ");
  Serial.print(velocity[1][counter]);
  Serial.print("m/s Z = ");
  Serial.print(velocity[2][counter]);
  Serial.println("m/s");

  //Print Displacement
  Serial.print("Displacement: ");
  Serial.print("X = ");
  Serial.print(displacement[0][counter]);
  Serial.print("m Y = ");
  Serial.print(displacement[1][counter]);
  Serial.print("m Z = ");
  Serial.print(displacement[2][counter]);
  Serial.println("m");
}

// Flag variable to run additional code in main loop --> (whichInterupt)
void distractedDriving()
{
  interuptDetected = true;
}

//Determine which interupt was flagged and print coresponding messages
void whichInterupt(unsigned long int seconds, unsigned int minutes, unsigned int hours)
{
  bool interuptReleased = false;
  int voltage = analogRead(3);
  //Serial.println(voltage);
 
  if (voltage > 1000)
    interuptReleased = true; //Set to print ending message
  else if (voltage == 0)
    whichDistraction = "Window controls";
  else if (voltage < 50 && voltage > 0 )
    whichDistraction = "Radio controls";
  else if (voltage > 50 && voltage < 340)
    whichDistraction = "Accelerator pedal";
  else 
    whichDistraction = "Brake pedal";

  //Print distraction notice
  if (interuptReleased == false)
  {
    Serial.println("============================================================================================");
    interuptInitialTime = seconds;
    Serial.print(whichDistraction);
    Serial.println(" engaged");
    timeStamp(seconds, minutes, hours);
  }
  else
  {
    Serial.println("============================================================================================");
    interuptFinalTime = seconds;
    int changeintime = (interuptFinalTime - interuptInitialTime);
    Serial.print(whichDistraction);
    Serial.print(" released after ");
    if ( whichDistraction == "Brake pedal")
    {
    accumulativeBrake += changeintime;
    }
    if(changeintime == 0 && whichDistraction == "Brake pedal")
    {
      accumulativeBrake += 0.5; // add median time of brake depressed if less than one second is triggered
    }
    checkBrakes(); // call function to update brake pad status;
    
    if (changeintime == 0)
    Serial.print("<1");
    else
    Serial.print(changeintime);
    Serial.println(" seconds");
    timeStamp(seconds, minutes, hours);
    interuptReleased = false;
  }
  interuptDetected = false;
}

void checkBrakes()
{
  if (accumulativeBrake <= 5)
  {
  digitalWrite(brakePadGood, HIGH);
  }
  
  else if (accumulativeBrake > 5 && accumulativeBrake <= 10)
  {
  digitalWrite(brakePadGood, LOW);
  digitalWrite(brakePadLow, HIGH);
  }
 
  else 
  {
  digitalWrite(brakePadLow, LOW);
  digitalWrite(brakePadReplace, HIGH);
  }
}



