#include <avr/io.h>
#include <util/delay.h>

/*  

#define S0 28
#define S1 29
#define S2 27
#define S3 25
#define sensorOut 26
*/

#define S0_PIN 6
#define S1_PIN 7
#define S2_PIN 5
#define S3_PIN 3
#define SENSOR_OUT_PIN 4
// return 0 if Red, return 1 if Green, return 2 if neither detected
long colourArray[3] = {0,0,0}; 
void setup() {
// Setting pin modes
DDRA |= (1 << S0_PIN) | (1 << S1_PIN) | (1 << S2_PIN) | (1 << S3_PIN);
DDRA &= ~(1 << SENSOR_OUT_PIN); // sensorOut pin as input

// Setting frequency scaling to 20%
PORTA |= (1 << S0_PIN);
PORTA &= ~(1 << S1_PIN);
Serial.begin(9600);
}

unsigned short redGreen()
{
  //setting red filtered photodiodes to be read
  PORTA &= ~((1 << S2_PIN) | (1 << S3_PIN));
  colourArray[0] = pulseIn(26, LOW);
 // Serial.print(" R = ");
  //Serial.print(colourArray[0]);
  delay(200);

  //setting green filtered photodiodes to be read
  PORTA |= (1 << S2_PIN) | (1 << S3_PIN);
  colourArray[1] = pulseIn(26, LOW);
//  Serial.print(" G = ");
 // Serial.print(colourArray[1]);
  delay(200);

  //setting blue filtered photodiodes to be read
  PORTA &= ~(1 << S2_PIN);
  PORTA |= (1 << S3_PIN);
  colourArray[2] = pulseIn(26, LOW);
  delay(200);
  //Serial.print(" B = ");
 // Serial.print(colourArray[2]);
  //delay(100);
  //return 0;
//}
  
 return determine_colour(colourArray);
}
// floats to hold colour array
//to be callibrated
long redArray[3] = {273,461,363};
long greenArray[3] = {423,389,350};
long gArray[3] = {414,301,223};
long rArray[3] = {244, 450, 317};
long blueArray[3] = {472,376,260};
long orangeArray[3] = {287,516,442};
long purpleArray[3] = {399,443,297};
long blackArray[3] = {600,627,516};
long whiteArray[3] = {183,187,152};

long red2Array[3] = {144,352,270};
long green2Array[3] = {364,293,299};
long g2Array[3] = {288,195,199};
long r2Array[3] = {181, 383, 273};
long blue2Array[3] = {277,158,108};
long orange2Array[3] = {172,388,328};
long purple2Array[3] = {250,271,172};
long black2Array[3] = {576,560,445};
long white2Array[3] = {107,109,89};

long red3Array[3] = {372,482,388};
long green3Array[3] = {456,442,382};
long g3Array[3] = {468,355, 327};
long r3Array[3] = {305,511,358};
long blue3Array[3] = {558,422,314};
long orange3Array[3] = {411,616,510};
long purple3Array[3] = {489,522,379};
long black3Array[3] = {692,733,590};
long white3Array[3] = {310, 330, 263} ;

long *refColours[21] = {redArray, red2Array, red3Array, greenArray,  green2Array,  green3Array, whiteArray, blackArray, blueArray, orangeArray, purpleArray, white2Array, black2Array, blue2Array, orange2Array, purple2Array, black3Array, white3Array, blue3Array, orange3Array, purple3Array};
short determine_colour(long colourArray[]) {
  double min_dist = 100000;
  short min_i = -1;
  
  // finds minimum Euclidean distance between measured color and reference colors
  for (short i = 0; i < 15; i += 1) { //redArray là 0 ref[0][0] mean redArray, R
  //only to 15 because to that limit sure it is another color
    double dist = sqrtf((double)(refColours[i][0] - colourArray[0]) * (refColours[i][0] - colourArray[0])
      + (double)(refColours[i][1] - colourArray[1]) * (refColours[i][1] - colourArray[1])
      + (double)(refColours[i][2] - colourArray[2]) * (refColours[i][2] - colourArray[2]));

    if (dist < min_dist) {
      min_dist = dist;
      if (i < 3) //still in the field of redArray
      {
        min_i = 0;
      }
      else if (i < 6) // still in the range of greenArray
      {
        min_i = 1;
      }
      else{
        min_i = 2;
      }
    }
   }
return min_i;
}

/*

void loop () {
  short result = redGreen();
  Serial.print("Value : ");
  Serial.println(result);
  delay (200);
}
*/

/*
void loop() {
//  setup();
 //setting red filtered photodiodes to be read
  PORTA &= ~((1 << S2_PIN) | (1 << S3_PIN));
  colourArray[0] = pulseIn(26, LOW);
  Serial.print(" R = ");
  Serial.print(colourArray[0]);
  delay(100);

  //setting green filtered photodiodes to be read
  PORTA |= (1 << S2_PIN) | (1 << S3_PIN);
  colourArray[1] = pulseIn(26, LOW);
  Serial.print(" G = ");
  Serial.print(colourArray[1]);
  delay(100);

  //setting blue filtered photodiodes to be read
  PORTA &= ~(1 << S2_PIN);
  PORTA |= (1 << S3_PIN);
  colourArray[2] = pulseIn(26, LOW);
  Serial.print(" B = ");
  Serial.print(colourArray[2]);
  delay(100);

   short result = redGreen();
  Serial.print("Value : ");
  Serial.println(result);
  delay (200);
  
}
*/


