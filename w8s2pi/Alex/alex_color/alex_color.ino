#include <serialize.h>
#include <stdarg.h>
#include <AFMotor.h>
#include "Arduino.h"
#include <math.h>
#include "packet.h"
#include "constants.h"
#define PI 3.141592654

#define ALEX_LENGTH 25 //we need to fking callibrate this for the turning!!!!!!!!!!!!!!!!!!
#define ALEX_BREADTH 16

float alexDiagonal = 0.0;
float alexCirc = 0.0;
volatile TDirection dir;
/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.
unsigned long moveStartTime = 0; // Time when the move command was issued
unsigned int moveDuration = 0; // Duration for the move in milliseconds
bool isMoving = false; // Flag to check if movement is ongoing

#define COUNTS_PER_REV 4 //we need to fking callibrate this for the forward backward!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 21

//for the colour sensor pin
#define S0_PIN 6
#define S1_PIN 7
#define S2_PIN 5
#define S3_PIN 3
#define SENSOR_OUT_PIN 4

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
// volatile unsigned long leftRevs;
// volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaDistback;
unsigned long newDistback;

//variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//indicate the trig and echo pin of ultrasonic sensor
const int trigPinLeft = A8;    // left ultrasonic sensor trig pin (White)
const int echoPinLeft = A9;    // left ultrasonic sensor echo pin (Blue)
const int trigPinRight = A10;  // right ultrasonic sensor trig pin (Yellow)
const int echoPinRight = A11;  // right ultrasonic sensor echo pin (Green)
const int trigPinFront = A13;  // front ultrasonic sensor trig pin (Orange)
const int echoPinFront = A14;  // front ultrasonic sensor echo pin (Brown)

//to store readings from left and right ultrasonic sensors
int distanceLeft = 0;   // distance from left ultrasonic sensor
int distanceRight = 0;  // distance from right ultrasonic sensor
int distanceFront = 0;  // distance from front ultrasonic sensor

//new function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// void left(float ang, float speed) {
//   if (ang == 0)
//     deltaTicks = 99999999;
//   else
//     deltaTicks = computeDeltaTicks(ang);
//   targetTicks = leftReverseTicksTurns + deltaTicks;
//   ccw(ang, speed);
// }

// void right(float ang, float speed) {
//   if (ang == 0)
//     deltaTicks = 99999999;
//   else
//     deltaTicks = computeDeltaTicks(ang);
//   targetTicks = leftForwardTicksTurns + deltaTicks;
//   cw(ang, speed);
// }

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".
  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus() {
  TPacket statusPacket;                            // Create a new packet named statusPacket
  statusPacket.packetType = PACKET_TYPE_RESPONSE;  // Set packet type to PACKET_TYPE_RESPONSE
  statusPacket.command = RESP_STATUS;              // Set the command field to RESP_STATUS
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}
void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 1
void enablePullups() {
  DDRD &= ~((1 << PD2) | (1 << PD3));
  PORTD |= (1 << PD2) | (1 << PD3);
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
  // Check the current direction and update the appropriate counters
  if (dir == FORWARD1) {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD1) {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
    //leftForwardTurnsDist = (unsigned long) ((float) leftForwardTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
    // leftReverseTurnsDist = (unsigned long) ((float) leftReverseTicksTurns / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR() {
  // Check the current direction and update the appropriate counters
  if (dir == FORWARD1) {
    rightForwardTicks++;
  } else if (dir == BACKWARD1) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}
// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EICRA = 0b10100000;
  EIMSK = 0b00001100;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

ISR(INT2_vect) {
  rightISR();
}
ISR(INT3_vect) {
  leftISR();
}
// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  // Set frame format: 8 data bits, no parity, 1 stop bit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  UBRR0L = 103;
  UBRR0H = 0;
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {
  int count = 0;
   while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}


void writeSerial(const char *buffer, int len) {
  for (int i = 0; i < len; i += 1) {
    // Wait until the transmit buffer is ready for new data
    while (!(UCSR0A & 0b00100000)) ;
      // This loop spins until the UDRE0 bit of UCSR0A register is set,
      // (the UDR0 register is empty and can accept new data).
       UDR0 = (uint8_t)buffer[i]; // Place the current character into the USART Data Register
    }
    
  }

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
}
// Intialize Alex's internal states

void initializeState() {
  clearCounters();
}

long colourArray[3] = { 0, 0, 0 };

void setupColor() {
  // Setting pin modes
  DDRA |= (1 << S0_PIN) | (1 << S1_PIN) | (1 << S2_PIN) | (1 << S3_PIN);
  DDRA &= ~(1 << SENSOR_OUT_PIN);  // sensorOut pin as input

  // Setting frequency scaling to 20%
  PORTA |= (1 << S0_PIN);
  PORTA &= ~(1 << S1_PIN);
  Serial.begin(9600);
}

long redArray[3] = { 273, 461, 363 };
long greenArray[3] = { 423, 389, 350 };
long gArray[3] = { 414, 301, 223 };
long rArray[3] = { 244, 450, 317 };
long blueArray[3] = { 472, 376, 260 };
long orangeArray[3] = { 287, 516, 442 };
long purpleArray[3] = { 399, 443, 297 };
long blackArray[3] = { 600, 627, 516 };
long whiteArray[3] = { 183, 187, 152 };

long red2Array[3] = { 144, 352, 270 };
long green2Array[3] = { 364, 293, 299 };
long g2Array[3] = { 288, 195, 199 };
long r2Array[3] = { 181, 383, 273 };
long blue2Array[3] = { 277, 158, 108 };
long orange2Array[3] = { 172, 388, 328 };
long purple2Array[3] = { 250, 271, 172 };
long black2Array[3] = { 576, 560, 445 };
long white2Array[3] = { 107, 109, 89 };

long red3Array[3] = { 372, 482, 388 };
long green3Array[3] = { 456, 442, 382 };
long g3Array[3] = { 468, 355, 327 };
long r3Array[3] = { 305, 511, 358 };
long blue3Array[3] = { 558, 422, 314 };
long orange3Array[3] = { 411, 616, 510 };
long purple3Array[3] = { 489, 522, 379 };
long black3Array[3] = { 692, 733, 590 };
long white3Array[3] = { 310, 330, 263 };

long *refColours[21] = {
    redArray, red2Array, red3Array,
    greenArray, green2Array, green3Array,
    whiteArray, blackArray, blueArray,
    orangeArray, purpleArray, white2Array,
    black2Array, blue2Array, orange2Array,
    purple2Array, black3Array, white3Array,
    blue3Array, orange3Array, purple3Array
};


short determine_colour(long colourArray[]) {
  double min_dist = 100000;
  short min_i = -1;

  // finds minimum Euclidean distance between measured color and reference colors
  for (short i = 0; i < 15; i += 1) {  //redArray là 0 ref[0][0] mean redArray, R
                                       //only to 15 because to that limit sure it is another color
    double dist = sqrtf((double)(refColours[i][0] - colourArray[0]) * (refColours[i][0] - colourArray[0])
                        + (double)(refColours[i][1] - colourArray[1]) * (refColours[i][1] - colourArray[1])
                        + (double)(refColours[i][2] - colourArray[2]) * (refColours[i][2] - colourArray[2]));

    if (dist < min_dist) {
      min_dist = dist;
      if (i < 3)  //still in the field of redArray
      {
        min_i = 0;
      } else if (i < 6)  // still in the range of greenArray
      {
        min_i = 1;
      } else {
        min_i = 2;
      }
    }
  }
  return min_i;
}

unsigned short redGreen() {
  //setting red filtered photodiodes to be read
  PORTA &= ~((1 << S2_PIN) | (1 << S3_PIN));
  colourArray[0] = pulseIn(26, LOW);
  delay(200);

  //setting green filtered photodiodes to be read
  PORTA |= (1 << S2_PIN) | (1 << S3_PIN);
  colourArray[1] = pulseIn(26, LOW);
  delay(200);

  //setting blue filtered photodiodes to be read
  PORTA &= ~(1 << S2_PIN);
  PORTA |= (1 << S3_PIN);
  colourArray[2] = pulseIn(26, LOW);
  delay(200);


return determine_colour(colourArray);
}

void sendColor() {
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_COLOR;
  statusPacket.params[0] = redGreen();
  sendResponse(&statusPacket);
}

void setupUltra() {
  Serial.begin(9600);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
}

int findUltraLeft()  //find ultrasonic reading from both sensors edited by yj 1242
{
  // Send ultrasonic sensor pulse for left sensor
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);

  // Measure time for left ultrasonic sensor echo
  long durationLeft = pulseIn(echoPinLeft, HIGH);

  // Calculate distance from left ultrasonic sensor
  distanceLeft = durationLeft / 58;
  return distanceLeft;
}


int findUltraRight()  //find ultrasonic reading from right sensor
{
  
  // Send ultrasonic sensor pulse for right sensor
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);

  // Measure time for right ultrasonic sensor echo
  long durationRight = pulseIn(echoPinRight, HIGH);

  // Calculate distance from right ultrasonic sensor
  distanceRight = durationRight / 58;

 
  return distanceRight;
}

int findDist()  //find ultrasonic reading from front sensor
{
  
  // Send ultrasonic sensor pulse for right sensor
  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);

  // Measure time for right ultrasonic sensor echo
  long durationFront = pulseIn(echoPinFront, HIGH);

  // Calculate distance from right ultrasonic sensor
  distanceFront = durationFront / 58;

 
  return distanceFront;
}

void sendUltra() {  //send ultrasonic packet
  TPacket ultraPacket;
  ultraPacket.packetType = PACKET_TYPE_RESPONSE;
  ultraPacket.command = RESP_ULTRA;
  ultraPacket.params[0] = findUltraLeft();
  ultraPacket.params[1] = findUltraRight();
  sendResponse(&ultraPacket);
}
void sendDist() {  //send distance packet edited yj
  TPacket distPacket;
  distPacket.packetType = PACKET_TYPE_RESPONSE;
  distPacket.command = RESP_DIST;
  distPacket.params[0] = findDist();
  sendResponse(&distPacket);
}


void handleCommand(TPacket *command) {

  switch (command->command) {
    case COMMAND_GET_STATS:
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      // Call clearOneCounter, though it clears all counters in this implementation.
      clearOneCounter(command->params[0]);
      // Send back an OK packet to acknowledge the command.
      sendOK();
     // break;
      break;
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
 
      sendOK();
      forward((double)command->params[0], (float)command->params[1]);
      sendDist();  //edited
      break;
    case COMMAND_REVERSE:
      sendOK();
      backward((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((double)command->params[0], (float)command->params[1]);
      break;
    case COMMAND_COLOR:
      sendOK();
      sendColor();
      break;

    case COMMAND_ULTRA:
      sendOK();
      sendUltra();
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {


        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  }  // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  setupColor();
  setupUltra();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet) {
  
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
    
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
 /*TXY commented out at 551pm,1442024
  if (isMoving && (millis() - moveStartTime >= moveDuration)) {
    stop(); // Stop the robot after the set duration
  }*/
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 100);
  //Serial.println("q");
  // Uncomment the code below for Week 9 Studio 2


  // put your main code here, to run repeatedly:
  TPacket recvPacket;  // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }
/* TXY commented at 12:04am, 14/4/2024 
  if (deltaDist > 0) {
    if (dir == FORWARD){
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD){
      if (reverseDist > newDistback) {
        deltaDist = 0;
        newDistback = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (leftForwardTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }*/
  /*
  current_time = millis();
  if (dir == FORWARD1) 
  {
    move
    
    stop();
    delay
}
*/
/* if (deltaDist > 0) {
if (dir == FORWARD1) {
      delay(200);
      stop();
    } else if (dir == BACKWARD1) {
      delay(200);
      stop();
    } else if (dir == STOP) {
      stop();
    }
  }
    if (deltaTicks > 0) {
  if (dir == LEFT) {
    delay(200);
    stop();
    } else if (dir == RIGHT) {
      delay(200);
      stop(); 
    } else if (dir == STOP) {
      stop();
    }
  }*/
  if (isMoving && (millis() - moveStartTime >= moveDuration)) {
    stop(); // Stop the robot after the set duration
  }
}
