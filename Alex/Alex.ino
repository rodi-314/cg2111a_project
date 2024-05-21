#include <serialize.h>
#include <stdarg.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define sensorOut 26
#define sensorOut_mapped 4
#define colorSensorDelay 50
#define colorAverageDelay 20

#define echoPinF 52
#define echoPinL 41
#define echoPinR 42
#define trigPin 1
#define echoPinF_mapped 1
#define echoPinL_mapped 0
#define echoPinR_mapped 7
#define SPEED_OF_SOUND 340

#define buzzer 0

#define MOVEMENT_SPEED 50
#define TURNING_SPEED 50
#define NUDGE_DELAY 400

volatile TDirection dir;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the wheel encoder.
#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          21

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and right encoders

volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

volatile unsigned long distFront = 0;
volatile unsigned long distLeft = 0;
volatile unsigned long distRight = 0;

volatile unsigned long redFreq = 0;
volatile unsigned long greenFreq = 0;
volatile unsigned long blueFreq = 0;

/*
 *
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
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

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start (args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);  
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

void sendColour() {
  //Obtain the distance the colour values were taken from
  unsigned long distance = getDistance(echoPinF);
  
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = redFreq;
  colourPacket.params[1] = greenFreq;
  colourPacket.params[2] = blueFreq;
  colourPacket.params[3] = distance;
  sendResponse(&colourPacket);
}

void colourSetup() {
  DDRA |= ((1 << S0) | (1 << S1) | (1 << S2) | (1 << S3)); //Set S0, S1, S2, S3 to OUTPUT
  DDRA &= ~(1 << sensorOut_mapped); //Set sensorOut to INPUT
  
  //Setting frequency scaling to 20%
  PORTA |= (1 << S0);
  PORTA &= ~(1 << S1);
}

int avgFreq() {
  int reading;
  int total = 0;

  //Obtain 5 readings
  for (int i = 0; i < 5; i++) {
    reading = pulseIn(sensorOut, LOW);
    total += reading;
    delay(colorSensorDelay);
  }
  //Return the average of 5 readings
  return total / 5;
}

void findColor() { 
  // Setting RED filtered photodiodes to be read
  PORTA &= ~((1 << S2) | (1 << S3)); 
  delay(colorSensorDelay);

  // Reading the output frequency for RED
  redFreq = avgFreq();
  delay(colorSensorDelay);

  // Setting GREEN filtered photodiodes to be read
  PORTA |= ((1 << S2) | (1 << S3));
  delay(colorSensorDelay);

  // Reading the output frequency for GREEN
  greenFreq = avgFreq();
  delay(colorSensorDelay);

  // Setting BLUE filtered photodiodes to be read
  PORTA &= ~(1 << S2);
  PORTA |= (1 << S3);
  delay(colorSensorDelay);

  // Reading the output frequency for BLUE
  blueFreq = avgFreq();
  delay(colorSensorDelay);
}

void setupUltrasonic() {
  //Set trigPin as OUTPUT and echoPinF, echoPinL, echoPinR as INPUT
  DDRG |= (1 << trigPin);
  DDRG &= ~(1 << echoPinL_mapped);
  DDRL &= ~(1 << echoPinR_mapped);
  DDRB &= ~(1 << echoPinF_mapped);
}

unsigned long getDistance(int echoPin) {
  PORTG &= ~(1 << trigPin);
  delayMicroseconds(2);
  PORTG |= (1 << trigPin);
  delayMicroseconds(10);
  PORTG &= ~(1 << trigPin);

  unsigned long duration = pulseIn(echoPin, HIGH);
  return duration * SPEED_OF_SOUND / 20000.0; //returns the distance in cm
}

void sendDistance() {
  distFront = getDistance(echoPinF);
  distLeft = getDistance(echoPinL);
  distRight = getDistance(echoPinR);
  
  TPacket distancePacket;
  distancePacket.packetType = PACKET_TYPE_RESPONSE;
  distancePacket.command = RESP_DIST;
  distancePacket.params[0] = distFront;
  distancePacket.params[1] = distLeft;
  distancePacket.params[2] = distRight;
  sendResponse(&distancePacket);
}

void setupBuzzer() {
  //Set buzzer as OUTPUT and turn off buzzer
  DDRB |= (1 << buzzer);
  PORTB |= (1 << buzzer);
}

void buzz() {
  //Turn on buzzer
  PORTB &= ~(1 << buzzer);
  delay(500);

  //Turn off buzzer
  PORTB |= (1 << buzzer);
  delay(500);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */

// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= ~(0b00001100);
  PORTD |= 0b1100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((((float) leftForwardTicks) / (float) COUNTS_PER_REV) * WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((((float) leftReverseTicks) / COUNTS_PER_REV) * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
}

// Set up the external interrupt pins INT2 and INT3 for falling edge triggered
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be falling edge triggered.
  EIMSK = 0b00001100;
  EICRA = 0b10100000;
  
}

ISR(INT2_vect)
{
  rightISR();
}

ISR(INT3_vect)
{
  leftISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */

// Set up the serial connection. 
void setupSerial()
{
  //Setting baud rate
  UBRR0L = 103;
  UBRR0H = 0;
  //Set Asynchronous USART, disable parity, set 1 stop bit, set 8-bit frame
  UCSR0C = 0b110;
  //Disable double-speed mode and multiprocessor mode
  UCSR0A = 0;
}

// Start the serial connection.
void startSerial()
{
  //Start by enabling RXEN0 and TXEN0
  UCSR0B = 0b11000;
}

// Read the serial port.
int readSerial(char *buffer)
{
  int count=0;
  while ((UCSR0A & (1 << RXC0)) == 0); //Poll until data has been received
  buffer[count++] = UDR0; // Assign received byte to buffer array
  return 1;
}

// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  for (int i = 0; i < len; i++) {
    while ((UCSR0A & (1 << UDRE0)) == 0); //Poll until UDRE0 is set
    UDR0 = buffer[i];
  }
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;

  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Intialize Alex's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    case COMMAND_FORWARD:
        sendOK();
        dir = (TDirection) FORWARD;
        move(MOVEMENT_SPEED, FORWARD);
      break;
      
    case COMMAND_REVERSE:
        sendOK();
        dir = (TDirection) BACKWARD;
        move(MOVEMENT_SPEED, BACKWARD);
      break;
      
    case COMMAND_TURN_LEFT:
        sendOK();
        dir = (TDirection) LEFT;
        move(TURNING_SPEED, CCW);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        dir = (TDirection) RIGHT;
        move(TURNING_SPEED, CW);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;

    case COMMAND_GET_STATS:
        sendStatus();
      break;
    
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearCounters();
      break;

    case COMMAND_COLOUR:
        sendOK();
        findColor();
        sendColour();
        buzz();
        break;

    case COMMAND_DIST:
        sendOK();
        sendDistance();
      break;

    case COMMAND_NUDGE_FORWARD:
        sendOK();
        dir = (TDirection) FORWARD;
        move(MOVEMENT_SPEED, FORWARD);
        delay(NUDGE_DELAY);
        stop();
      break;

    case COMMAND_NUDGE_BACKWARD:
        sendOK();
        dir = (TDirection) BACKWARD;
        move(MOVEMENT_SPEED, BACKWARD);
        delay(NUDGE_DELAY);
        stop();
      break;

    case COMMAND_NUDGE_LEFT:
        sendOK();
        dir = (TDirection) LEFT;
        move(MOVEMENT_SPEED, CCW);
        delay(NUDGE_DELAY);
        stop();
      break;

    case COMMAND_NUDGE_RIGHT:
        sendOK();
        dir = (TDirection) RIGHT;
        move(MOVEMENT_SPEED, CW);
        delay(NUDGE_DELAY);
        stop();
      break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  cli();
  setupEINT();
  colourSetup();
  setupBuzzer();
  setupUltrasonic();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
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
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
}
