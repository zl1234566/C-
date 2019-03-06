// A0 and A1 will be the thermometer inputs
#define GRILLTHERM A0         
#define MEATTHERM A1
// 100k nominal thermistors at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// take 100 temperature readings and average them to reduce noise
#define NUMSAMPLES 100
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
//Thermistors are in voltage divider configuration with 120k resistors
#define SERIESRESISTOR 120000   
 
uint16_t grillSamples[NUMSAMPLES];
uint16_t meatSamples[NUMSAMPLES];
volatile int setPoint = 200;    //Fan kicks on when grill is below this temp
                                //This temp can be manually changed with 2 buttons

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
 
void setup(void) {

//set buttons to digital pin 2, 3 as inputs with pullup resistors enabled
pinMode(2, INPUT_PULLUP);
pinMode(3, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(2), raiseTemp, FALLING);
attachInterrupt(digitalPinToInterrupt(3), lowerTemp, FALLING);

// initialize the LCD screen
lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
  
  Serial.begin(9600);
  analogReference(EXTERNAL);  //vref is connected to 3.3V for less noisy signal
  pinMode(22, OUTPUT);        //Pin 22 will control the relay
  digitalWrite(22, LOW);      //relay is active high, start off

}
 
void loop(void) {
  uint8_t i;
  float averageGrill;
  float averageMeat;

  // read the resistance of each thermistor 100 times and store as samples
  for (i=0; i< NUMSAMPLES; i++) {
   grillSamples[i] = analogRead(GRILLTHERM);
   meatSamples[i] = analogRead(MEATTHERM);
  }
 
  // average all the samples out
  averageGrill = 0;
  averageMeat = 0;

  for (i=0; i< NUMSAMPLES; i++) 
  {
     averageGrill += grillSamples[i];
     averageMeat += meatSamples[i];

  }
  averageGrill /= NUMSAMPLES;
  averageMeat /= NUMSAMPLES;

//  Serial stuff for troubleshooting, uncomment next 4 lines to see raw input values in serial monitor
//  Serial.print("averageGrill analog reading "); 
//  Serial.println(averageGrill);
//  Serial.print("averageMeat analog reading "); 
//  Serial.println(averageMeat);
 
  // Convert raw input values to thermistor resistances
  averageGrill = 1023 / averageGrill - 1;
  averageGrill = SERIESRESISTOR / averageGrill;
  averageMeat = 1023 / averageMeat - 1;
  averageMeat = SERIESRESISTOR / averageMeat;

//  Serial stuff for troubleshooting, uncomment next 4 lines to see thermistor resistances in serial monitor
//  Serial.print("Thermistor resistance for Grill Thermistor: "); 
//  Serial.println(averageGrill);
//  Serial.print("Thermistor resistance for Meat Thermistor: "); 
//  Serial.println(averageMeat);

 //Use thermistor resistances to find temperatures
  float grillTemp;
  grillTemp = averageGrill / THERMISTORNOMINAL;     // (R/Ro)
  grillTemp = log(grillTemp);                  // ln(R/Ro)
  grillTemp /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  grillTemp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  grillTemp = 1.0 / grillTemp;                 // Invert
  grillTemp -= 273.15;                         // convert to C
  grillTemp = ((grillTemp*(1.8)) + 32);          // convert to F

  //apply correction factor from quadratic regression
  //grillTemp = -0.000730208*pow(grillTemp,2) + 1.16905*grillTemp - 11.0198; 
 


    float meatTemp;
  meatTemp = averageMeat / THERMISTORNOMINAL;     // (R/Ro)
  meatTemp = log(meatTemp);                  // ln(R/Ro)
  meatTemp /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  meatTemp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  meatTemp = 1.0 / meatTemp;                 // Invert
  meatTemp -= 273.15;                         // convert to C
  meatTemp = ((meatTemp*(1.8)) + 32);          // convert to F

  //apply correction factor from quadratic regression
  //meatTemp = -0.00054287*pow(meatTemp,2) + 1.1023*meatTemp - 7.97296;

//More serial troubleshooting stuff
//  Serial.print("Grill Temperature "); 
//  Serial.print(grillTemp);
//  Serial.println(" *F");
//  Serial.print("Meat Temperature "); 
//  Serial.print(meatTemp);
//  Serial.println(" *F");

if (grillTemp <= (setPoint))  //Check if grill temp is lower than we want it to be
  {
  digitalWrite(22, HIGH);  //Turn fan on
  }
 else 
 {
  digitalWrite(22,LOW);  //Turn fan off
 }

  lcd.setCursor ( 0, 0 );            // Display the grill temperature
  lcd.print("Grill: ");
  lcd.setCursor ( 8, 0 );
  lcd.print(grillTemp); 
  lcd.print(" F "); 

  lcd.setCursor ( 0, 1 );            // Display the meat temperature
  lcd.print("Meat: "); 
  lcd.setCursor ( 8, 1 );
  lcd.print(meatTemp); 
  lcd.print(" F "); 

  lcd.setCursor ( 0, 3 );            // Display the fan setpoint temperature
  lcd.print("Fan Setpoint "); 
  lcd.print(setPoint); 
  lcd.print(" F ");  
}

//Interrupt service routine for raising setpoint temperature
void raiseTemp()
{
setPoint = setPoint + 5;
}

//Interrupt service routine for lowering setpoint temperature
void lowerTemp()
{
setPoint = setPoint - 5;
}
