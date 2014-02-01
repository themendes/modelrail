/******************************************************************
 *
 * Train control system for the Arduino Mega 2560
 *
 * Copyright (c) Chris Mendes 2014
 *
 */
 
/*

   /---------------------------------------------------\
  //------------------------------------\               \
 ///-----------------------------------======---------\  \
///                                       \            \  |
|||                                       X2            | |
|||                                        |            | |
|||                       /---*---X6-S-}{-/             | |
|||                      /   /                          | |
|||                    X8   X7                         X3 X1
\\\                1   /   /                            | |
 \\\-----------X3--*--/   /                             | |
  \\                \    /                             /  |
   \\--------S-X2----*--*--*-----X5-----*--X3A-S--}{--/  /
    \                2  3  4\5        6/7               /
     \-------S-X1------------*---X4---*----X1A-S--}{---/
     
*/

#define  DEBUG  0  // LEVLES 0 = OFF, 1-5 increase in verbosity

void
DebugPrint(int  level, char *s)
{
  if(level > DEBUG)
    return;
  Serial.println(s);
}

void
DebugPrintf(int  level, char *fmt,...)
{
  char tmp[256];   // resulting string limited to DISPLAY_MAX_STRING_LEN chars

  if(level > DEBUG)
    return;

  va_list args;
  va_start (args, fmt);

  vsnprintf(tmp, 256, fmt, args);

  va_end (args);

  DebugPrint(level, tmp);

}

#include  <TimerFive.h>

#define  FIFO_SIZE  32

unsigned long  Fifo[FIFO_SIZE];
int  FifoTail = 0;
int  FifoHead = 0;

void
FifoInit()
{
  noInterrupts();
  memset(Fifo,0,FIFO_SIZE);
  FifoHead = -1;
  FifoTail = -1;
  interrupts();
}

boolean
FifoPush(unsigned long  i)
{
  noInterrupts();
  if(FifoIsEmpty())
  {
    FifoHead = FifoTail = 0;
    Fifo[0] = i;
    
    interrupts();
    return true;
  }
  else
  {
    FifoHead++;
    if( FifoHead == FIFO_SIZE)
    {
      FifoHead = 0;
    }
    
    if( FifoHead == FifoTail )
    {
      FifoHead--;
      
      if(FifoHead == -1)
        FifoHead = FIFO_SIZE - 1;
        
      interrupts();  
      return false;
    }
    
    Fifo[FifoHead] = i;
    interrupts();
    return true;
  }
  
}

boolean
FifoPop(unsigned long *i)
{
  noInterrupts();
  
  if(FifoIsEmpty())
  {
    interrupts();
    return false;
  }
  
  *i = Fifo[FifoTail];

  if(FifoTail == FifoHead)
  {
    FifoTail = FifoHead = -1;
  }
  else
  {
    FifoTail++;
    if( FifoTail == FIFO_SIZE)
      FifoTail = 0;
  }
  interrupts();
  return true;
}

boolean
FifoPeek(unsigned long *i)
{
  noInterrupts();
  if(FifoIsEmpty())
  {
    interrupts();
    return false;
  }
  *i = Fifo[FifoTail];
  interrupts();
  return true;
}

boolean
FifoIsEmpty()
{
  if((FifoHead == -1) && (FifoTail == -1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

//
// ################## DISPLAY ################### 
//
#include <stdarg.h>

#define  DISPLAY_MAX_STRING_LEN  32

HardwareSerial  *DisplaySerial;

void
DisplayGetSize(int  *x, int  *y)
{
  *x = 16;
  *y = 5;
}

void
DisplayInit(HardwareSerial  *port)
{
  DisplaySerial = port;
  
  DisplaySerial->begin(9600);

  DisplaySerial->write("DC");
  DisplaySerial->write((byte)0x00);
  DisplaySerial->write("CL");
  DisplayPrint(0,0,"Initialising...");
  DebugPrintf(1,"==== Initialising ====");
}

void
DisplayClear()
{
  DisplaySerial->write("CL");
}

void
DisplayPrint(int x, int y, char *str)
{
  DisplaySerial->write("TP");
  DisplaySerial->write((byte)x);
  DisplaySerial->write((byte)y);  
  DisplaySerial->write("TT");
  DisplaySerial->print(str);
  DisplaySerial->write(0x0D);  
  DisplaySerial->flush();
}


void
DisplayPrintf(int x, int y, char *fmt,...)
{
  char tmp[DISPLAY_MAX_STRING_LEN];   // resulting string limited to DISPLAY_MAX_STRING_LEN chars

  va_list args;
  va_start (args, fmt);

  vsnprintf(tmp, DISPLAY_MAX_STRING_LEN, fmt, args);

  va_end (args);

  DisplayPrint(x,y,tmp);
  
  DebugPrint(5,tmp);
}
//
// ^^^^^^^^^^^^^^^^^^ DISPLAY ^^^^^^^^^^^^^^^^^^
//

//
// ################## POINTS ################### 
//

#define  NUM_POINTS  8
#define  POINT_STRAIGHT  0
#define  POINT_SWITCH    1

#define  ServoResetPin  2


byte  PointAngles[NUM_POINTS][2] = { {86,96}, {85,94}, {94,86}, {85,96}, {97, 85}, {115, 98}, {93, 74}, {97, 79}};

byte  Command[NUM_POINTS];
byte  LastCommand[NUM_POINTS];

boolean  GotCommand;

unsigned long      points_rest_timer;
#define            POINT_REST_TIME_MS  2000L  // milliseconds to wait before turningoff all points.

HardwareSerial  *ServoSerial;

void ServoInit(HardwareSerial *port)
{
  int  i;
  int  pos;
  
  ServoSerial = port;
  
  points_rest_timer = 0;
  
  //Reset the servo controller
  pinMode(ServoResetPin, OUTPUT);
  digitalWrite(ServoResetPin, LOW);
  delay(20);
  digitalWrite(ServoResetPin, HIGH);
  
  // set the data rate for the SoftwareSerial port
  ServoSerial->begin(38400);
  
  for (i = 0 ; i < 8 ; i++ )
  {
    set_speed(i, 2);
    delay(100);
    set_neutral(i, 3000);
    delay(100);
    set_servo_parameters(i, 0, 15);
    delay(100);
    // Move all points to position 0 - straight
    pos = PointAngles[i][0];
    position_8bit(i,(byte)map(pos,0,179,0,255));
  }
  //wait for them all to move, then turn them off
  delay(3000);
  for (i = 0 ; i < 8 ; i++ )
  {
    set_servo_parameters(i, 0, 15);
  }
    
  memset(Command,0x00,NUM_POINTS);
  memset(LastCommand,0x00,NUM_POINTS);

  GotCommand = false;

} //ServoSetup



void 
ServoLoop()
{
  int  i;
  
  if(GotCommand == true)
  {
    for(i = 0; i < NUM_POINTS; i++)
    {
      if(LastCommand[i] != Command[i]) // if it was changed
      {
        int  pos;
        
        pos = PointAngles[i][(Command[i])];

        //move it!
        position_8bit(i,(byte)map(pos,0,179,0,255));
        delay(200);
        points_rest_timer = millis() + POINT_REST_TIME_MS;
        
        DebugPrintf(5,"%lu = points_rest_timer", points_rest_timer);
      }
      else
      {
        // motor is now idle so turn it off
        set_servo_parameters(i, 0, 15);
        set_speed(i, 2);
      }
    }
    GotCommand = false;
    memcpy(LastCommand,Command,NUM_POINTS);
  }

  if( (points_rest_timer > 0)  && (points_rest_timer < millis()) )
  {
    DebugPrintf(5,"%lu %lu Turning points off", millis(), points_rest_timer);

    points_rest_timer = 0;
    
    for (i = 0; i < NUM_POINTS; i++)
    {
        // motor is now idle so turn it off
        if( (i != 5) && (i != 6)) // points 6 & 7 are subject to a bit of noise so this is a temp hack to never turn them off!!
        {  
          set_speed(i, 2);
          set_servo_parameters(i, 0, 15);
        }
    }
    
  }
}

// POLOLU CODE

void set_servo_parameters(byte servo, byte OnOff, byte rangeVal)
{
   //this function uses pololu mode command 0 to set servo parameters
   //servo is the servo number (typically 0-7)
   //rangeVal of 15 gives 180 deg in 8-bit, 90 deg in 7 bit
   
   byte temp;
   byte parameters;
   
   temp = OnOff << 6;                     //set first two bits of parameters (on = 1, forward = 0)
   temp = temp + (rangeVal & 0x1f);   //put first five bits of rangeVal into temp
   parameters = temp & 0x7f;          //take only bottom 7 bits
      
   //Send a Pololu Protocol command
   ServoSerial->write(0x80);       //start byte
   ServoSerial->write(0x01);       //device id
   ServoSerial->write((byte)0x00);       //command number
   ServoSerial->write(servo);      //servo number
   ServoSerial->write(parameters); //parameters
   ServoSerial->flush();
}

void set_speed(byte servo, byte speedVal)
{
   //this function uses pololu mode command 1 to set speed
   //servo is the servo number (typically 0-7)
   //speedVal is servo speed (1=slowest, 127=fastest, 0=full)
   //set speedVal to zero to turn off speed limiting
   
   speedVal = speedVal & 0x7f; //take only lower 7 bits of the speed
   
   //Send a Pololu Protocol command
   ServoSerial->write(0x80);     //start byte
   ServoSerial->write(0x01);     //device id
   ServoSerial->write(0x01);     //command number
   ServoSerial->write(servo);    //servo number
   ServoSerial->write(speedVal); //speed
   ServoSerial->flush();
}

void position_7bit(byte servo, byte posValue)
{
  //this function uses pololu mode command 2 to set position  
  //servo is the servo number (typically 0-7)
  //posValue * range (set with command 0) adjusted by neutral (set with command 5)
  //determines servo position

   byte pos = posValue & 0x7f;     //get lower 7 bits of position

   //Send a Pololu Protocol command
   ServoSerial->write(0x80);     //start byte
   ServoSerial->write(0x01);     //device id
   ServoSerial->write(0x02);     //command number
   ServoSerial->write(servo);    //servo number
   ServoSerial->write(pos);     //position
   ServoSerial->flush();
}

void position_8bit(byte servo, byte posValue)
{
  //this function uses pololu mode command 3 to set position  
  //servo is the servo number (typically 0-7)
  //posValue * range (set with command 0) adjusted by neutral (set with command 5)
  //determines servo position

   byte temp;
   byte pos_hi,pos_low;
   
   temp = posValue & 0x80;      //get bit 8 of position
   pos_hi = temp >> 7;            //shift bit 8 by 7
   pos_low = posValue & 0x7f;     //get lower 7 bits of position

   //Send a Pololu Protocol command
   ServoSerial->write(0x80);     //start byte
   ServoSerial->write(0x01);     //device id
   ServoSerial->write(0x03);     //command number
   ServoSerial->write(servo);    //servo number
   ServoSerial->write(pos_hi);  //bits 8 thru 13
   ServoSerial->write(pos_low); //bottom 7 bits
   ServoSerial->flush();
}

void position_absolute(byte servo, int angle)
{
  //this function uses pololu mode command 4 to set absolute position  
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500

   unsigned int temp;
   byte pos_hi,pos_low;
   
   temp = angle & 0x1f80;  //get bits 8 thru 13 of position
   pos_hi = temp >> 7;     //shift bits 8 thru 13 by 7
   pos_low = angle & 0x7f; //get lower 7 bits of position

   //Send a Pololu Protocol command
   ServoSerial->write(0x80);    //start byte
   ServoSerial->write(0x01);    //device id
   ServoSerial->write(0x04);    //command number
   ServoSerial->write(servo);   //servo number
   ServoSerial->write(pos_hi);  //bits 8 thru 13
   ServoSerial->write(pos_low); //bottom 7 bits
   ServoSerial->flush();
}

void set_neutral(byte servo, int angle)
{
  //this function uses pololu mode command 5 to set neutral position  
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500

   unsigned int temp;
   byte pos_hi,pos_low;
   
   temp = angle & 0x1f80;  //get bits 8 thru 13 of position
   pos_hi = temp >> 7;     //shift bits 8 thru 13 by 7
   pos_low = angle & 0x7f; //get lower 7 bits of position

   //Send a Pololu Protocol command
   ServoSerial->write(0x80);    //start byte
   ServoSerial->write(0x01);    //device id
   ServoSerial->write(0x05);    //command number
   ServoSerial->write(servo);   //servo number
   ServoSerial->write(pos_hi);  //bits 8 thru 13
   ServoSerial->write(pos_low); //bottom 7 bits
   ServoSerial->flush();
}

// POINTS 
byte  PointCommand[NUM_POINTS];

void
PointsInit()
{
  int  i;

  ServoInit(&Serial1);
  memset(PointCommand,0x00,NUM_POINTS);
  
  PointsDisplay();
}

void
PointsDisplay()
{
  int  i;
  
  for(i = 0; i< NUM_POINTS;i++)
  {
    if( PointCommand[i] == POINT_SWITCH)
      DisplayPrint((i+8),0,"C");
    else
      DisplayPrint((i+8),0,"S");
  }
}

void
PointsFlush()
{
  int  i;
  
  memcpy(Command,PointCommand,NUM_POINTS);
  GotCommand = true;

  ServoLoop();
  
  PointsDisplay();
}

//
// Points are numbered starting from 1 as per the diagram in the top of this file.
//
void
PointsMove(int  p, int  pos)
{
  // p is the point number as seen on the table
  PointCommand[p-1] = pos;

  DebugPrintf(4,"PointsMove(%d, %d)", p, pos);

}

//
// ^^^^^^^^^^^^^^^^^^ POINTS ^^^^^^^^^^^^^^^^^^
//

//
// ################## MOTORS ################### 
//
#define  NUM_MOTORS  2
#define  MOTOR_CONTROL_NONE  -1

typedef struct {
  int  in1_pin;
  int  in2_pin;
  int  enable_pin;
  int  speed;
  int  control_id;
} MotorType;

MotorType  Motors[NUM_MOTORS];

// MOTOR SECTION
void
MotorsInit()
{
  int  i;
  
  Motors[0].in1_pin = 3;
  Motors[0].in2_pin = 4;
  Motors[0].enable_pin = 7;
  Motors[0].speed = 0;
  Motors[0].control_id = MOTOR_CONTROL_NONE;
  
  Motors[1].in1_pin = 5;
  Motors[1].in2_pin = 6;
  Motors[1].enable_pin = 8;
  Motors[1].speed = 0;
  Motors[1].control_id = MOTOR_CONTROL_NONE;

/*
  TIMER 4
  Value                   Divisor               Frequency
  0×01                        1                 31250 hz
  0×02                        8                  3926.25 hz
  0×03                       32                   976.5625 hz
  0×04                       64                   488.28125 hz    // default
  0×05                      128                   244.140625 hz
  Code:                 TCCR4B = (TCCR4B & 0xF8) | value ;
*/

  // Not ethat the default PWM setting although audible at low speed works most efficiently.
  
  TCCR4B = (TCCR4B & 0xF8) | 0x04 ;
  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  
}

// Motors are 1 indexed.
// A negative speed implies reverse (-255 to +255)

#define  MOTOR_TRAM    1
#define  MOTOR_TRAINS  0

#define  MOTOR_ZERO_PT  10


#define  MOTOR_ERR_OK  0
#define  MOTOR_ERR_BAD_MOTOR   1
#define  MOTOR_ERR_IN_USE      2
#define  MOTOR_ERR_BAD_CONTROL 3

//
// MOTORS ARE ZERO INDEXED
//

int
MotorSetSpeed(int  motor, int  spd, int  control_id)
{
  boolean reverse = false;
  char  temp[5];
  
  if(motor >= NUM_MOTORS)
    return MOTOR_ERR_BAD_MOTOR;
    
  if( control_id < MOTOR_CONTROL_NONE )
    return MOTOR_ERR_BAD_CONTROL;
  
  if(Motors[motor].control_id > MOTOR_CONTROL_NONE)
  {
    // if the motor is being controlled then check if it is the right person giving command
    if( control_id != Motors[motor].control_id)
      return MOTOR_ERR_IN_USE;
  }
  else
  {
    // take control of the motor
    Motors[motor].control_id = control_id;
  }
  
  Motors[motor].speed = spd;

  if(spd < 0)
    reverse = true;
    
  spd = abs(spd);
   
  if(spd > 255)
    spd = 255;  // clip it just in case!

  if(reverse)
  {
    digitalWrite(Motors[motor].in1_pin,HIGH);
    digitalWrite(Motors[motor].in2_pin,LOW);    
  }
  else
  {
    digitalWrite(Motors[motor].in1_pin,LOW);
    digitalWrite(Motors[motor].in2_pin,HIGH);    
  }

  if (spd <= MOTOR_ZERO_PT)
  {
    digitalWrite(Motors[motor].in1_pin,HIGH);
    digitalWrite(Motors[motor].in2_pin,HIGH);    
  }
  analogWrite(Motors[motor].enable_pin, spd);
  
  return  MOTOR_ERR_OK;
}

int
MotorGetSpeed(int  m)
{
  return Motors[m].speed;
}

int
MotorWhoControls(int  m)
{
  return Motors[m].control_id;
}

// returns true if the control ID provided is currently in control of the motor or if it's not controlled at all (!)
boolean
MotorGotControl(int  m, int  control_id)
{
  if ( Motors[m].control_id == MOTOR_CONTROL_NONE)
    return true;
    
  return (Motors[m].control_id == control_id);
}

// returns true if you have successfully taken control of the motor.
boolean
MotorGetControl(int  m, int  control_id)
{
  if ( ( Motors[m].control_id == MOTOR_CONTROL_NONE) ||
       ( Motors[m].control_id == control_id) )
  {
    Motors[m].control_id = control_id;
    return true;
  }
  else
    return false;
}


void
MotorReleaseControl(int  m, int  control_id)
{
  if( control_id == Motors[m].control_id)
  {
    Motors[m].control_id = MOTOR_CONTROL_NONE;
  }  
}

void
MotorsDisplay()
{
  DisplayPrintf(7, 4, "%04d:%04d", Motors[0].speed, Motors[1].speed);   
}


void
MotorsLoop()
{
    if( (int)( millis() / 100) % 2 == 0)
      MotorsDisplay();
}

//
// ^^^^^^^^^^^^^^^^^^ MOTORS ^^^^^^^^^^^^^^^^^^
//

 
/*
 * LAYOUT
 *
 *    /---------------------------------------------------\
 *   //------------------------------------\               \
 *  ///-----------------------------------======---------\  \
 * ///                                       \            \  |
 * |||                                       X2            | |
 * |||                           8            |            | |
 * |||                       /---*---X6-S-}{-/             | |
 * |||                      /   /                          | |
 * |||                    X8   X7                         X3 X1
 * \\\                1   /   /                            | |
 *  \\\-----------X3--*--/   /                             | |
 *   \\                \    / 4            7              /  |
 *    \\--------S-X2----*--*--*-----X5-----*--X3A-S--}{--/  /
 *     \                2  3   \          /                /
 *      \-------S-X1------------*---X4---*----X1A-S--}{---/
 *                              5        6
 *
 * Tram line is not shown.
 * 
 * Sensors
 *   ID Pin  Destination
 *   __ ___  ____________
 *   0   0   TRAM_VILLAGE
 *   1   1   TRAM_STATION
 *   2   2   X6
 *   3   3   X3A
 *   4   4   X1A
 *   5   5   X1
 *   6  12   X2
 *   7   7   X7
 *   8   8   X8
 *
 */

typedef  enum { VILLAGE, STATION, X6, X3A, X1A, X1, X2, X7, X8, NUM_DESTINATIONS } Destinations;
#define  DESTINATION_UNDEFINED  -1

//
// ################## SECTIONS ################### 
//

//
// A section of track is an electrically isolated section with independent wiring to a power souce.
// It may be relay controlled or hard wired to be always on
// A section could have more than one motor potentially but not in this layout
// A section can have more than one sensor/destination
//

#define  NUM_SECTION_RELAYS       8
#define  SECTION_RELAY_START_PIN  22
#define  SECTION_RELAY_INCR       2
#define  SECTION_NOT_OCCUPIED     -1

#define  NO_SECTION_RELAY  -1

typedef enum { S_TRAM = 0, S_X1, S_X2, S_X3, S_X4, S_X5, S_X6, S_X7, S_X8, S_X9, S_X1A, S_X3A, NUM_SECTIONS } SectionIDs;

typedef struct {
  char          *name;
  int           relay_pin;
  int           num_sensors;
  int           motor;
  boolean       blocked;
  int           occupied;
} Section;


Section  Sections[NUM_SECTIONS];  //indexed by SectionID enum type.

void
SectionsInit()
{
  int  i;
  
  // Do the basic setup for the relay pins
  for(i = 0; i < NUM_SECTION_RELAYS; i++ )
  {
    pinMode(SECTION_RELAY_START_PIN + ( i * SECTION_RELAY_INCR ), OUTPUT);
    digitalWrite(SECTION_RELAY_START_PIN + ( i * SECTION_RELAY_INCR ), HIGH);
  }

  for(i = S_TRAM; i < NUM_SECTIONS; i++ )
  {
    Sections[i].motor = MOTOR_TRAINS;
    Sections[i].blocked = false;
    Sections[i].occupied = SECTION_NOT_OCCUPIED;
  }


  Sections[S_TRAM].name = "TRAM LINE";
  Sections[S_TRAM].relay_pin = NO_SECTION_RELAY;
  Sections[S_TRAM].num_sensors = 2;
  Sections[S_TRAM].motor = MOTOR_TRAM;

  Sections[S_X1].name = "OUTER LINE X1";
  Sections[S_X1].relay_pin = (0 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X1].num_sensors = 1;

  Sections[S_X2].name = "MIDDLE LINE X2";
  Sections[S_X2].relay_pin = (1 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X2].num_sensors = 1;

  Sections[S_X3].name = "INNER LINE X3";
  Sections[S_X3].relay_pin = (2 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X3].num_sensors = 1;

  Sections[S_X4].name = "OUTER LINE POINTS";
  Sections[S_X4].relay_pin = NO_SECTION_RELAY;
  Sections[S_X4].num_sensors = 0;

  Sections[S_X5].name = "INNER LINE POINTS";
  Sections[S_X5].relay_pin = NO_SECTION_RELAY;
  Sections[S_X5].num_sensors = 0;

  Sections[S_X6].name = "STATION APPROACH";
  Sections[S_X6].relay_pin = (3 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X6].num_sensors = 1;

  Sections[S_X7].name = "PLATFORM 2";
  Sections[S_X7].relay_pin = (4 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X7].num_sensors = 1;

  Sections[S_X8].name = "PLATFORM 1";
  Sections[S_X8].relay_pin = (5 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X8].num_sensors = 1;

  Sections[S_X9].name = "MID-INNER POINTS";
  Sections[S_X9].relay_pin = NO_SECTION_RELAY;
  Sections[S_X9].num_sensors = 0;

  Sections[S_X1A].name = "OUTER LOOP POINTS";
  Sections[S_X1A].relay_pin = (6 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X1A].num_sensors = 1;

  Sections[S_X3A].name = "INNER LOOP POINTS";
  Sections[S_X3A].relay_pin = (7 * SECTION_RELAY_INCR) + SECTION_RELAY_START_PIN;
  Sections[S_X3A].num_sensors = 1;


  for(i = S_TRAM; i < NUM_SECTIONS; i++ )
  {
    SectionIsolate(i);
    delay(100);
    SectionActivate(i);
  }
}


// Relays are turned ON by setting LOW
// Sections are isolated by relayes being ON
// Returns false if the section can't be isolated
boolean
SectionIsolate(int  s)
{
  if( Sections[s].relay_pin != NO_SECTION_RELAY)
  {
    int  p;
    
    p =  Sections[s].relay_pin;
    
    digitalWrite(p , LOW);
    DisplayPrint(((p-SECTION_RELAY_START_PIN)/SECTION_RELAY_INCR),0,"O");
    return  true;
  }
  else
    return  false;
}

boolean
SectionActivate(int  s)
{
  if( Sections[s].relay_pin != NO_SECTION_RELAY)
  {
    int  p;
    
    p =  Sections[s].relay_pin;
    
    digitalWrite(p , HIGH);
    DisplayPrint(((p-SECTION_RELAY_START_PIN)/SECTION_RELAY_INCR),0,"*");
    return  true;
  }
  else
    return  false;
}

boolean
SectionOccupied(int s)
{
  return ( Sections[s].occupied != SECTION_NOT_OCCUPIED);
}

void
SectionOccupy(int s, int train)
{
  if( (Sections[s].occupied  != SECTION_NOT_OCCUPIED) && (s != S_TRAM) )
  {
    DebugPrintf( 0, "Section being occupied but is already (s,o,t) %d, %d, %s", s, Sections[s].occupied, TrainsGetName(train));
    // PANIC
  }
  
  Sections[s].occupied = train; 
  DebugPrintf(4, "SectionOccupy(%d, %s)", s, TrainsGetName(train));
}

void
SectionLeave(int s, int train)
{
  if( Sections[s].occupied  == SECTION_NOT_OCCUPIED)
  {
    DebugPrintf(0, "Section being left but was not occupied (s,t) %d, %s",s,TrainsGetName(train));
  }
  else if( Sections[s].occupied  != train )
  {  
    DebugPrintf(0, "Section being left but was occupied by another (s,o,t) %d, %d, %d", s, Sections[s].occupied , train);
  }
  
  DebugPrintf(4, "SectionLeave(%d, %s)", s, TrainsGetName(train));
  
  Sections[s].occupied = SECTION_NOT_OCCUPIED;
}

int
SectionBlocked(int s)
{
  return Sections[s].blocked;
}

void
SectionBlock(int  s)
{
  Sections[s].blocked = true;
  DebugPrintf(4, "SectionBlock(%d)", s);
}

void
SectionClear(int s)
{
  Sections[s].blocked = false;
  DebugPrintf(4, "SectionClear(%d)", s);
}


//
// ^^^^^^^^^^^^^^^^^^ SECTIONS ^^^^^^^^^^^^^^^^^^
//


//
// ################## SENSORS ################### 
//

#define  NUM_SENSORS         9
#define  SENSOR_THRESHOLD    800
#define  SENSOR_DEBOUNCE_MS  200

typedef struct {
  int            pin;
  Destinations   destination;
  char           *name;
  int            threshold;
  int            current_value;
  unsigned long  debounce;
  SectionIDs     section;
} Sensor;

Sensor  Sensors[NUM_SENSORS];
  
void
SensorsInit()
{
  int  i;
  
  for(i = 0; i < NUM_SENSORS; i++)
  {
    Sensors[i].threshold = SENSOR_THRESHOLD;
    Sensors[i].debounce = 0;
    Sensors[i].current_value = 1024;
  }
  
  Sensors[0].destination = VILLAGE;
  Sensors[0].pin = 0;
  Sensors[0].name = "Village";
  Sensors[0].section = S_TRAM;
  
  Sensors[1].destination = STATION;
  Sensors[1].pin = 1;
  Sensors[1].name = "Station";
  Sensors[1].section = S_TRAM;
  
  Sensors[2].destination = X6;
  Sensors[2].pin = 2;
  Sensors[2].name = "Approach Station X6";
  Sensors[2].section = S_X6;
  
  Sensors[3].destination = X3A;
  Sensors[3].pin = 3;
  Sensors[3].name = "Inner Points X3A";
  Sensors[3].section = S_X3A;
  
  Sensors[4].destination = X1A;
  Sensors[4].pin = 4;
  Sensors[4].name = "Outer Points X1A";
  Sensors[4].section = S_X1A;
  
  Sensors[5].destination = X1;
  Sensors[5].pin = 5;
  Sensors[5].name = "Outer Line X1";
  Sensors[5].section = S_X1;
  
  Sensors[6].destination = X2;
  Sensors[6].pin = 12;
  Sensors[6].name = "Station Line X2";
  Sensors[6].section = S_X2;
  
  Sensors[7].destination = X7;
  Sensors[7].pin = 7;
  Sensors[7].name = "Platform 2 X7";
  Sensors[7].section = S_X7;
  
  Sensors[8].destination = X8;
  Sensors[8].pin = 8;
  Sensors[8].name = "Platform 1 X8";
  Sensors[8].section = S_X8;
  
}


void
SensorsDisplayAnalogs()
{
  
  /*
  int  i;
  
  for(i = 0; i < 16; i++)
  {
    int  j;
    char s[3];
    
    j = analogRead(i);
    
    sprintf(s,"%02d",j/11);
    
    DisplayPrint(i,4,s);
  }
  */
}


int
SensorsGetSection(int s)
{
  return Sensors[s].section;
}

int
SensorsGetDestination(int  s)
{
  return Sensors[s].destination;
}

int
SensorsMapDestinationToSection(int  d)
{
  return Sensors[d].section;
}

void
SensorsPoll()
{
  int  i;
  int  j;
  
  for ( i = 0; i < NUM_SENSORS; i++ )
  {
        Sensors[i].current_value = analogRead(Sensors[i].pin);
                
        if ( (Sensors[i].current_value < Sensors[i].threshold) && \
             (Sensors[i].debounce == 0) )
        {
          Sensors[i].debounce = millis() + SENSOR_DEBOUNCE_MS;
          //TrainsHandleSensorTrigger(i);
          FifoPush((unsigned long)i);
        }
        if ( Sensors[i].debounce > 0 )
        {
          if( millis() > Sensors[i].debounce )
            Sensors[i].debounce = 0;  // debounce period is now over so reset it
        }
  }

}


//
// ^^^^^^^^^^^^^^^^^^ SENSORS ^^^^^^^^^^^^^^^^^^
//

//
// ################## TRAINS ################### 
//

#define  DFWD  -1
#define  DREV  1

typedef struct {
  int  dest;
  int  stop_time_s;
  boolean  try_to_run;
  int  dir;
} WayPoint;

typedef struct {
  int        num_waypoints;
  WayPoint   waypts[32];
  
} Route;

Route  TramRoute = {
  2, 
  { 
    {(int)VILLAGE, 30, true, DFWD},
    {(int)STATION, 30, true, DREV}
  }
};

Route  DemoRoute = {
  13, 
  { 
    {(int)X7 , 20, false, DREV},
    {(int)X3A,  0, true,  DREV},
    {(int)X3A,  5, false, DREV},
    {(int)X2 ,  0, false, DREV},
    {(int)X6 ,  0, false, DREV},
    {(int)X8 , 20, false, DREV},
    {(int)X3A, 10, false, DREV},
    {(int)X1 ,  0, false, DREV},
    {(int)X1A,  0, true,  DREV},
    {(int)X1 ,  0, true,  DREV},
    {(int)X1A, 10, true,  DREV},
    {(int)X2 ,  0, false, DREV},
    {(int)X6 ,  0, false, DREV}  
  }
};

Route  ShortDemoRoute = {
  12, 
  { 
    {(int)X7 , 40, false, DREV},
    {(int)X3A,  0, true,  DREV},
    {(int)X2 , 20, false, DREV},
    {(int)X6 ,  2, false, DREV},
    {(int)X8 , 40, false, DREV},
    {(int)X3A, 10, false, DREV},
    {(int)X1 , 20, false, DREV},
    {(int)X1A,  0, true,  DREV},
    {(int)X1 , 20, true,  DREV},
    {(int)X1A, 10, true,  DREV},
    {(int)X2 ,  0, false, DREV},
    {(int)X6 ,  2, false, DREV}  
  }
};

Route  ShortRoute = {
  7, 
  { 
    {(int)X8 , 30, false, DREV},
    {(int)X3A,  5, false,  DREV},
    {(int)X1,  10, false, DREV},
    {(int)X1A, 10, false, DREV},
    {(int)X3A,  0, false, DREV},
    {(int)X2 , 10, false, DREV},
    {(int)X6 ,  2, false, DREV},
  }
};

Route  OuterRoute = {
  8, 
  { 
    {(int)X1A,  0, false, DREV},
    {(int)X1 , 20, false, DREV},
    {(int)X1A,  0, false, DREV},
    {(int)X2 , 20, false, DREV},
    {(int)X6 ,  2, false, DREV},
    {(int)X7 , 40, false, DREV},
    {(int)X3A,  5, false, DREV},
    {(int)X1 , 20, false, DREV},
  }
};

typedef enum {  bootup = 1,          //1
                stopping_scheduled,  //2
                stopped_scheduled,   //3
                stopping_blocked,    //4
                stopped_blocked,     //5
                check_proceed_start, //6
                check_proceed_run,   //7
                starting,            //8 
                running              //9
              } TrainState;

typedef struct {
  char           *name;
  int            set_speed;         // this is the target speed we want to be at, current speed is read from the motor.
  int            motor;
  int            current_position;  // a DESTINATION which will have to be mapped to a section...
  int            destination;
  int            cruise_speed;
  int            crawl_speed;
  unsigned long  timer;
  TrainState     state;
  int            display_row;
  Route          my_route;
  // destination { STATION, VILLAGE, X6, X3A, X1A, X1, X2, X7, X8 }
  int            stop_delays[NUM_DESTINATIONS]; // delay in 1/10ths second to run for after entering a stop before decelrating
  unsigned long  stop_delay_timer;
  float          brake_delta;
  float          accel_delta;
  int            speed_step;
  int            way_pt_index;  // always points to the destination we are aiming for in the route
  int            stop_for_time; // time for next stop in milli-seconds.
  boolean        braking;
  unsigned long  backoff_timer;

} Train;

#define  NUM_TRAINS  4

Train  Trains[] = {
  {
    "TRM", 0, MOTOR_TRAM  , 0, DESTINATION_UNDEFINED, 100,  70, 0L, bootup, 0, TramRoute,
    { 3, 5, 0, 0, 0, 0, 0, 0, 0 },
    0L, 2.0, 5.0, 0, 0, 0L, false, 0L
  },
  {
    "462", 0, MOTOR_TRAINS, 0, DESTINATION_UNDEFINED, 165,  70, 0L, bootup, 1, ShortDemoRoute,
    { 0, 0, 0, 0, 0, 80, 60, 5, 6 },
    0L, 5.0, 2.0, 0, 0, 0L, false, 0L
  },
  {
    "RED", 0, MOTOR_TRAINS, 0, DESTINATION_UNDEFINED,  90,  60, 0L, bootup, 2, ShortRoute,
    { 0, 0, 0, 0, 0, 80, 60, 5, 1 },
    0L, 1.0, 4.0, 0, 0, 0L, false, 0L
  },
  /*
  {
    "USD", 0, MOTOR_TRAINS, 0, DESTINATION_UNDEFINED, 130,  70, 0L, bootup, 2, ShortDemoRoute,
    { 0, 0, 0, 0, 0, 80, 60, 5, 5 },
    0L, 3.0, 2.0, 0, 0, 0L, false, 0L
  },
  */
  
  {
    "DBG", 0, MOTOR_TRAINS, 0, DESTINATION_UNDEFINED, 60,  40, 0L, bootup, 3, OuterRoute,
    { 0, 0, 0, 0, 0, 80, 60, 0, 0 },
    0L, 1.0, 5.0, 0, 0, 0L, false, 0L
  }
  
};  

void
TrainsDisplay()
{
  int  i;
  int  d;
  char  c;
  unsigned long  t;
  
  t = millis();
  
  for(i=0; i < NUM_TRAINS; i++)
  {

    if(Trains[i].timer)
    {
      if(t > Trains[i].timer)
      {
        c = 'E';
      }
      else
      {
        d = (int)((Trains[i].timer - t))/1000;
        if(d > 9)
        {
          if(d % 2)
            c = '^';
          else
            c = 'v';
        }
        else if (d >=0)
          c = (char)((int)'0' + d);
        else
          c = '?';
      }
    }
    else
    {
      c = '#';
    }

    if(Trains[i].display_row > 0)
    {
      DisplayPrintf(0, Trains[i].display_row, "%s %04d %01d %01d %01d %c",
                    Trains[i].name,
                    Trains[i].set_speed,
                    Trains[i].current_position,
                    Trains[i].destination,
                    Trains[i].state,
                    c
                  );
    }             
  }
}

char *
TrainsGetName(int train)
{
  return Trains[train].name;  
}


int
TrainsGetNextWayPt(int  t)
{
  int      next_way_pt;

  next_way_pt = Trains[t].way_pt_index + 1;  
  if( next_way_pt == Trains[t].my_route.num_waypoints)
    next_way_pt = 0;
    
  return next_way_pt;
}

#define  TRAIN_CHECK_BACKOFF_TIME_MS  (150 + random(100))

void
TrainProcess(int t)
{
  int      next_way_pt;
  int      cur_sect;
  boolean  rev;
  
  
  switch (Trains[t].state)
  {
    case bootup:
      next_way_pt = TrainsGetNextWayPt(t);  

      Trains[t].destination = Trains[t].my_route.waypts[next_way_pt].dest;
      Trains[t].way_pt_index = next_way_pt; 
      
      Trains[t].current_position = Trains[t].my_route.waypts[0].dest;

      cur_sect = SensorsMapDestinationToSection(Trains[t].current_position);
      SectionOccupy(cur_sect, t);
      
      Trains[t].state = stopped_blocked;
      
      break;
    case stopping_blocked:
    case stopping_scheduled:

      // set a timer to keep running for the right period based on this stop
      if( (Trains[t].stop_delays[Trains[t].current_position] > 0) && (! Trains[t].braking) )
      {
        // There should be a dleay here before stopping
        if( Trains[t].stop_delay_timer > 0 ) // we are already waiting to apply the brakes
        {
          if( millis() > Trains[t].stop_delay_timer) // timer is expired so we can apply brakes
          {
            Trains[t].stop_delay_timer = 0;
            Trains[t].braking = true;
            DebugPrintf(4, "Train %s can apply brakes at %d", 
                        Trains[t].name, 
                        Trains[t].current_position);
         }
          else
            return;  // do not start braking yet - it's not time to
        }
        else
        {
          // set the timer off...
          Trains[t].stop_delay_timer = millis() + Trains[t].stop_delays[Trains[t].current_position] * 100;
          DebugPrintf(4, "Setting stop delay timer for train %s, %d at %d", 
                        Trains[t].name, 
                        Trains[t].stop_delays[Trains[t].current_position], 
                        Trains[t].current_position);
          return;
        }
      }

      Trains[t].set_speed = 0;
           
      // ensure I control the motor otherwise print a panic and then go to hard stop and correct state.
      if( MotorGetControl( Trains[t].motor, t ) )
      {
        // slow down
        if( abs(MotorGetSpeed(Trains[t].motor)) > Trains[t].brake_delta) // we are within a fart of 0
        {
          if( MotorGetSpeed(Trains[t].motor) < 0)
            MotorSetSpeed(Trains[t].motor,  MotorGetSpeed(Trains[t].motor) + Trains[t].brake_delta, t );
          else
            MotorSetSpeed(Trains[t].motor,  MotorGetSpeed(Trains[t].motor) - Trains[t].brake_delta, t );
        }
        else
        {
          DebugPrintf(3, "Train %s now stopped at %d", Trains[t].name, Trains[t].current_position);
          Trains[t].braking = false;
          
          if( Trains[t].state == stopping_blocked )
            Trains[t].state = stopped_blocked;
          
          if( Trains[t].state == stopping_scheduled )
            Trains[t].state = stopped_scheduled;
        }
      }
      else
      {
        // I can't control the motor to slow down!
        // Force an isolation...
        DebugPrintf(3, "Train %s can't control motor so isolating at %d", Trains[t].name, Trains[t].current_position);
        Trains[t].braking = false;

        if( Trains[t].state == stopping_blocked )
          Trains[t].state = stopped_blocked;
          
        if( Trains[t].state == stopping_scheduled )
          Trains[t].state = stopped_scheduled;
          
        Serial.print("Can't control motor to stop: ");
        Serial.println(t);
      }
      break;
    case stopped_blocked:
      // Isolate myself
      SectionIsolate( SensorsMapDestinationToSection(Trains[t].current_position) );
      // Release the motor
      MotorReleaseControl(Trains[t].motor, t );

      // Set the state to check_proceed
      Trains[t].state = check_proceed_start;
      
      break;
    case stopped_scheduled:
      // check the timer. If its zero and we need to set a time then
      //  - Isolate myself
      //  - Release the motor
      //  - Set up the relevant timer
      // otherwise, check that the current time > the timer
      // if so, set the timer to 0 and start the train
      // otherwise just sit here
      
      // Isolate
      SectionIsolate( SensorsMapDestinationToSection(Trains[t].current_position) );
      // Release the motor
      MotorReleaseControl(Trains[t].motor, t);
      
      if(Trains[t].timer > 0)
      {
        // I'm waiting...
        if( millis() > Trains[t].timer)
        {
          // time to go!
          Trains[t].timer = 0;
          Trains[t].state = check_proceed_start;
        }  
        
      }
      else
      {
        //set the timer off
        Trains[t].timer = millis() + (long)(Trains[t].stop_for_time) ; //* 1000L;
        
        DebugPrintf(2, "Starting timer for train %d (%d)",t, Trains[t].stop_for_time);
      }
      break;
      
   case check_proceed_start:
      if( Trains[t].backoff_timer > 0L)
      {
        if( millis() > Trains[t].backoff_timer)
        {
          Trains[t].backoff_timer = 0;
        }
        else
          return;
      }

      if( SectionOccupied( SensorsMapDestinationToSection( Trains[t].destination) ) && 
          ( S_TRAM != SensorsMapDestinationToSection( Trains[t].destination) )) // tram line is always occupied!
      {
        Trains[t].state = stopped_blocked;
        return;
      }
      
      if (! MotorGetControl(Trains[t].motor, t) )
      {
        // I cant control the motor, dammit
        Trains[t].state = stopped_blocked;
        Trains[t].backoff_timer = millis() + TRAIN_CHECK_BACKOFF_TIME_MS;

        DebugPrintf(4,"%s can't control the motor because %s has it", Trains[t].name, Trains[ MotorWhoControls(Trains[t].motor) ].name);
        // Now, what happens if this was in CheckProceedRun? It should never be that way a this stage.
        return;
      }
      // fall through - no break
    case check_proceed_run:
      if (ControllerRequestTransition(t, Trains[t].current_position, Trains[t].destination) )
      {
        if(Trains[t].state == check_proceed_run)
          Trains[t].state = running;
          
        if(Trains[t].state == check_proceed_start) // valid as fall through into this case...
          Trains[t].state = starting;
      }
      else
      {
        Trains[t].state = stopping_blocked;
        //Trains[t].backoff_timer = millis() + TRAIN_CHECK_BACKOFF_TIME_MS;
      }
      break;
    case starting:
      if(Trains[t].set_speed == 0)
      {
        // we need to set a speed and get going...
        
        Trains[t].set_speed = Trains[t].cruise_speed * Trains[t].my_route.waypts[Trains[t].way_pt_index].dir;
        
        if(MotorGotControl(Trains[t].motor, t))
        {
          // I can control the speed
          SectionActivate( SensorsMapDestinationToSection(Trains[t].current_position) );
        }
        else
        {
          // soemone else is controlling the motor - I should never enter this code.
          DebugPrintf(0,"Train tried to start but did not have motor control %s", Trains[t].name);
          
          /* Here's a crude attempt at just going in any case bacause I have clearance 
          if( 
              (
                (MotorGetSpeed(Trains[t].motor) < 0) && 
                (Trains[t].set_speed < 0)
              )
              ||
               (
                (MotorGetSpeed(Trains[t].motor) > 0) && 
                (Trains[t].set_speed > 0)
              )
             )
           {
             // I want to go in the same direction so I can activate - not nice but effective.
             Trains[t].state = running;
             SectionActivate( SensorsMapDestinationToSection(Trains[t].current_position) ); 
             
           }
           else
           {
             // I'm effectively blocked here...            
           }
           */
        }
      }
      else
      {
        // we are already accellerating
        // if we are at the right speed now then set to running
        // else accellerate
        if(abs(abs(Trains[t].set_speed) - abs(MotorGetSpeed(Trains[t].motor))) < Trains[t].accel_delta)
        {
          Trains[t].state = running;
        }
        else
        {
          int  spd;
          
          if( Trains[t].set_speed < 0)
          {
            if(Trains[t].set_speed < MotorGetSpeed(Trains[t].motor))
              spd = MotorGetSpeed(Trains[t].motor) - Trains[t].accel_delta;
          }
          else
          {
            if(Trains[t].set_speed > MotorGetSpeed(Trains[t].motor))
              spd = MotorGetSpeed(Trains[t].motor) + Trains[t].accel_delta;
          }
          
          if ( MotorSetSpeed(Trains[t].motor,  spd, t) != MOTOR_ERR_OK )
          {
            // PANIC - I somehow lost control of the motor??
            DebugPrintf(0, "Train %s lost control of motor to %s",Trains[t].name, Trains[ MotorWhoControls(Trains[t].motor) ].name);
            
          }    
        }
      }
      break;
    case running:
      // Check if I have control of the motor and if I do then set my speed
      MotorSetSpeed(Trains[t].motor,  Trains[t].set_speed, t); // brute force method     
      break;
    default:
      // PANIC
      DebugPrintf(0, "Train in unknown state (t,s) %s, %d", Trains[t].name, Trains[t].state);
      break;
  }
}

void
TrainsHandleBlocked(int  train)
{
  Trains[train].state = stopping_blocked;
}

void
TrainsHandleScheduledStop(int  train, int  period_ms)
{
  Trains[train].state = stopping_scheduled;
  Trains[train].stop_for_time = period_ms;
}

void
TrainsHandleHadTransition(int  train)
{
  Trains[train].state = check_proceed_run;
}


void
TrainsInit()
{
  TrainsDisplay();
}

void
TrainsLoop()
{
  int  i;
  
  i = random(0, NUM_TRAINS);

  //for(i=0; i < NUM_TRAINS; i++)
  //{
    TrainProcess(i);
  //}
   
  if( (int)( millis() / 100) % 2 == 0)
    TrainsDisplay();
}

void
TrainsHandleSensorTrigger(int dest)
{
  int      i;
  int      arr_sect, cur_sect;
  boolean  proceed;
  boolean  rev;
  boolean  blocked;
  boolean  scheduled_stop;
  int      next_way_pt;
  int      t;
  
  DebugPrintf(1, "Event %d",dest);
  
  for(i=0; i < NUM_TRAINS; i++)
  {
    if( dest == Trains[i].destination && (Trains[i].state == running) )
    {
      // so now we are where we wanted to be
      ControllerTransitionCompleted(i,Trains[i].current_position, dest); // tell the controller
      arr_sect = SensorsMapDestinationToSection(dest); // update the sections
      cur_sect = SensorsMapDestinationToSection(Trains[i].current_position);
      
      SectionOccupy(arr_sect, i);
      
      if(arr_sect != cur_sect) // I cant have left a section I've just entered!!
        SectionLeave(cur_sect, i);
 
      // So we've dealt with where we have been, now let's move on.
    
      Trains[i].current_position = dest;
      
      next_way_pt = TrainsGetNextWayPt(i);
      
      Trains[i].destination = Trains[i].my_route.waypts[next_way_pt].dest;
      cur_sect = arr_sect;

      // so we know where we are and where we want to go next

      blocked = SectionBlocked(cur_sect);
      t = Trains[i].my_route.waypts[Trains[i].way_pt_index].stop_time_s;
      
      scheduled_stop =  (t > 0);
      
      rev = Trains[i].my_route.waypts[Trains[i].way_pt_index].stop_time_s;
      
      Trains[i].way_pt_index = next_way_pt; 

      if ( scheduled_stop )  // we need to stop here: - due to a scheduled stop or due to the section being blocked
      {
        TrainsHandleScheduledStop(i, t*1000); 
      }
      else if (blocked) // Commented out the 'if' for cautios running
      {
        TrainsHandleBlocked(i);
      }
      // Now here's what we tried for continuous running...
      else
      {
        TrainsHandleHadTransition(i);
      }
      
      // ONLY ONE TRAIN CAN TAKE AN EVENT AND THIS IS IT SO RETURN
      
      return;
      
    }
    else
    {
      DebugPrintf(3, "Rejected by train (t,d) %s %d", Trains[i].name, Trains[i].destination);
    }
  }
}

//
// ^^^^^^^^^^^^^^^^^^ TRAINS ^^^^^^^^^^^^^^^^^^
//

//
// ################## CONTROLLER ################### 
//

typedef struct {
  int      from; // destination transitions, not sections!
  int      to;
  boolean  reverse;
  int      point_moves[NUM_POINTS]; // POINTS_STRAIGHT OR POINTS_SWITCH or -1 for no move.
  boolean  block[NUM_SECTIONS];
  boolean  in_transit;
  int      clear_time; // how long in seconds after the transit is complete can I unblock the sections?
  unsigned long  timer;
} Transition;

// slightly naughty - point constants not in use here - 0 => Straight, 1 => Switch
// typedef enum { S_TRAM = 0, S_X1, S_X2, S_X3, S_X4, S_X5, S_X6, S_X7, S_X8, S_X9, S_X1A, S_X3A, NUM_SECTIONS } SectionIDs;

Transition  TransitionTable[] = {
  /* 0 */  { VILLAGE, STATION,  true, {-1, -1, -1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 1 */  { STATION, VILLAGE,  true, {-1, -1, -1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 2 */  { X8,      X3A,     false, { 0, -1, -1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 3 */  { X7,       X2,     false, {-1,  0,  1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,1}, false, 1 , 0},
  /* 4 */  { X7,      X3A,     false, { 1,  1,  1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 5 */  { X6,       X8,     false, {-1, -1, -1, -1, -1, -1, -1,  0 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 6 */  { X6,       X7,     false, {-1, -1, -1, -1, -1, -1, -1,  1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 7 */  { X2,       X6,     false, {-1, -1, -1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 8 */  { X1,      X1A,     false, {-1, -1, -1, -1, -1, -1, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 0 , 0},
  /* 9 */  { X1A,      X1,     false, {-1, -1, -1, -1,  0,  0, -1, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 1 , 0},
  /* 10 */ { X1A,      X2,     false, {-1,  0,  0,  1,  1,  0, -1, -1 }, {0,0,0,1,0,0,0,1,0,0,0,0}, false, 1 , 0},
  /* 11 */ { X1A,     X3A,     false, { 1,  1,  0,  1,  1,  0, -1, -1 }, {0,0,0,0,0,0,0,1,1,0,0,1}, false, 0 , 0},
  /* 12 */ { X3A,      X1,     false, {-1, -1, -1, -1,  0,  1,  1, -1 }, {0,0,0,0,0,0,0,0,0,0,1,0}, false, 1 , 0},
  /* 13 */ { X3A,      X2,     false, {-1,  0,  0,  0, -1, -1,  0, -1 }, {0,0,0,0,0,0,0,0,0,0,0,0}, false, 1 , 0},
  /* 14 */ { X3A,     X3A,     false, { 1,  1,  0,  0, -1, -1,  0, -1 }, {0,0,0,0,0,0,0,1,1,0,0,0}, false, 0 , 0},
};

#define  NUM_TRANSITIONS  15

void
ControllerInit()
{
  // nothing to do yet...
}

int
ControllerFindTransition(int  src, int dest)
{
  int  i;
  int  t;
  
  t = -1;
  
  for(i = 0; i < NUM_TRANSITIONS; i++)
  {
    if((src == TransitionTable[i].from) && (dest == TransitionTable[i].to))
      t = i;
  }
  
  DebugPrintf(5,"ControllerFindTransition(%d %d) = %d", src, dest, t);
  
  if( t == -1)
  {
    DebugPrintf(1, "Bad transition request (f,t) %d %d", src, dest);
  }
  
  return t;
}

boolean
ControllerRequestTransition(int  train, int  src, int dest)
{
  int  sect_src, sect_dest;
  int  trans;
  int  i, j;
  boolean  r;
  
  sect_src = SensorsMapDestinationToSection(src);
  sect_dest = SensorsMapDestinationToSection(dest);

  DebugPrintf(4, "ControllerRequestTransition (t,s,d) %s, %d, %d", TrainsGetName(train), src, dest);
  
  trans = ControllerFindTransition(src, dest);
  if( trans <  0 )
  {
    DebugPrintf(0,"No such transition!");
    return false;
  }
  
  if(SectionOccupied(sect_dest))
  {
    DebugPrintf(4, "Destination section occupied train, dest %s, %d", TrainsGetName(train), sect_dest);

    if(sect_dest != sect_src)
    {    
      return false;
    }
    
    // I am doing a loop on the same section with only one sensor => most likely X3A
    DebugPrintf(4, "It's OK, we're doing a loop on X3A %d, %d", sect_src, sect_dest);
    
  }
  
  if(SectionBlocked(sect_src)) 
  {
    DebugPrintf(4, "Source section blocked %s, %d", TrainsGetName(train), sect_src);
    
    return false;
  }
  
  if(TransitionTable[trans].in_transit)
  {
    //PANIC
    DebugPrintf(0, "PANIC: Transition already in transit (s,d,t,b) %d, %d, %s, %d", src, dest, TrainsGetName(train), trans);
    
    return false;
  }
  
  // NOW CHEKCK IF OTHER TRANSITIONS ARE USING THE SAME POINTS AND ARE ACTIVE
  for(i = 0 ; i < NUM_TRANSITIONS; i++)
  {
    if( TransitionTable[i].in_transit )
    {
      for( j = 0 ; j < NUM_POINTS; j++)
      {
        if( TransitionTable[trans].point_moves[j] >= 0 ) // For this transition, I need to move this point
        {
          if (TransitionTable[i].point_moves[j] >= 0)    // but another active transition need to use it...
          {
            DebugPrintf(4,"Busy points (this_xn, pt, other_xn) %d, %d, %d", trans, j+1, i);
            return false;
          }
        }
      }
    }
  }

  DebugPrintf(4,"Start transition %s %d",TrainsGetName(train), trans);

  if(trans > NUM_TRANSITIONS)
  {
    DebugPrintf(0,"PANIC: Wierd Transition ID %d", trans);
    while(true)
      ;
  }
  
  TransitionTable[trans].in_transit = true;
    
  for(i = 0 ; i < NUM_SECTIONS; i++)
  {
    if( TransitionTable[trans].block[i] > 0)
      SectionBlock(i);
  }

  for(i = 0 ; i < NUM_POINTS; i++)
  {
    if( TransitionTable[trans].point_moves[i] >= 0)
    {
      DebugPrintf(4,"Transition %d moving point %d to %d", trans, i+1, TransitionTable[trans].point_moves[i]);
      PointsMove(i+1, TransitionTable[trans].point_moves[i]);
    }
    else
    {
      DebugPrintf(5,"Transition %d not moving point %d", trans, i+1);
    }
  }
  
  PointsFlush();

  return true;
}

void
ControllerTransitionCompleted(int  train, int  src, int dest)
{
  // mark the transition as complete if timer = 0 otherwise, it happens later.
  // set the timer for clearing the blockage on the relevant sections
  int  sect_src, sect_dest;
  int  trans;
  int  i;
  
  DebugPrintf(4,"Transition Completed (t,s,d) %s %d %d",TrainsGetName(train),src,dest);
  
  sect_src = SensorsMapDestinationToSection(src);
  sect_dest = SensorsMapDestinationToSection(dest);
  
  trans = ControllerFindTransition(src, dest);
  if( trans <  0 )
  {
    DebugPrintf(5,"Transition Completed - Transition NOT FOUND ! (t,s,d, b) %d %d %d %d",train,src,dest, trans);
    return ;
  }
  
  if( TransitionTable[trans].clear_time == 0 )
  {
    TransitionTable[trans].in_transit = false;
    DebugPrintf(4,"Transition %d cleared by %s.",trans, TrainsGetName(train));
    // unblock the relevant sections
    for(i = 0 ; i < NUM_SECTIONS; i++)
    {
      if( TransitionTable[trans].block[i] > 0)
        SectionClear(i);
    }
  }
  else
  {
    TransitionTable[trans].timer = millis() + TransitionTable[trans].clear_time * 1000;
    DebugPrintf(4,"Transition %d timer started (%d) by %s for %lu.",trans, TransitionTable[trans].clear_time, TrainsGetName(train), TransitionTable[trans].timer );
  }
    
}
  

void 
ControllerLoop()
{
  // Check all sections to see if they can be unblocked
  // If the transition is active and the timer is not zero, then the train has passed the destination
  // and we are waiting for the timer to expire before we clear the transition.

  int  i, j ;
  
  for(i = 0; i < NUM_TRANSITIONS; i++)
  {
    if((TransitionTable[i].timer > 0) && TransitionTable[i].in_transit )
    {
      if( millis() > TransitionTable[i].timer) // The time is now past the timer value
      {
        for(j = 0 ; j < NUM_SECTIONS; j++)       // for each one listed, clear the section
        {
          if( TransitionTable[i].block[j] > 0)
            SectionClear(j);
        }
        
        TransitionTable[i].in_transit = false;   // we can finally clear the transit
        DebugPrintf(4,"Transition %d timer goes off %lu %lu.",i, TransitionTable[i].timer, millis());       
        TransitionTable[i].timer = 0;            // reset the timer
      }
    }
  }
}

//
// ^^^^^^^^^^^^^^^^^^ CONTROLLER ^^^^^^^^^^^^^^^^^^
//

//
// ################## APPLICATION ################### 
//

void
HandleEvents()
{
  unsigned long k;
  
  while( ! FifoIsEmpty() )
  {
    FifoPop(&k);
    TrainsHandleSensorTrigger((int)k);
  }
}
void
ShowTime()
{
  unsigned long  t;
  
  t = millis();
  
  if( (t / 500) % 2 == 0)
    DisplayPrint(0,4,"*");
  else
    DisplayPrint(0,4,"O");
    
}

void
ConsoleInit()
{
  Serial3.begin(9600);
}
void
ConsoleLoop()
{
  char  c;
  
  while(Serial3.available() )
  {
    c = Serial3.read();
    Serial.print(c);
  }
}

//
// ^^^^^^^^^^^^^^^^^^ APPLICATION ^^^^^^^^^^^^^^^^^^
//


void
setup()
{
  int  i;
  
  Serial.begin(9600);
  randomSeed(analogRead(15));
  
  ConsoleInit();
  FifoInit();
  DisplayInit(&Serial2);
  PointsInit();
  SectionsInit();
  MotorsInit();
  SensorsInit();
  ControllerInit();
  TrainsInit();
  
  Timer5.initialize(10000);           
  Timer5.attachInterrupt(SensorsPoll);  

  for(i = 0; i < NUM_SECTIONS; i++)
  {
    SectionIsolate(i);
    delay(100);
  }
  for(i=30;i > 0;i--)
  {
    DisplayPrintf(2,4,"== SETUP %02d ==", i);
    delay(1000);
  }
  for(i = 0; i < NUM_SECTIONS; i++)
  {
    SectionActivate(i);
    delay(100);
  }
  DisplayPrintf(2,4,"              ", i);

}


void
loop()
{
  ConsoleLoop();
  HandleEvents();
  ServoLoop();
  MotorsLoop();
  ControllerLoop();
  TrainsLoop();
  SensorsDisplayAnalogs();

  ShowTime();
  // delay(10);
}

