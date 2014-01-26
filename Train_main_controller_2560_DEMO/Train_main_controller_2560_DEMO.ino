
/*
 * If using the 8 pin LCD system
 
#include  <LiquidCrystal.h>

LiquidCrystal lcd(23,25,27,29,31,33);
*/

unsigned long  ticker = 0;

//FOR POINTS AND ASSOC. SERVOS

#define  NUM_POINTS  8
#define  POINT_STRAIGHT  0
#define  POINT_SWITCH    1

#define  ServoResetPin  2
#define  ServoSerial  Serial1

byte  PointAngles[NUM_POINTS][2] = { {86,96}, {85,94}, {94,86}, {85,94}, {97, 85}, {115, 98}, {93, 74}, {97, 79}};

byte  Command[NUM_POINTS];
byte  LastCommand[NUM_POINTS];

boolean  GotCommand;

int  big_loop;
#define  LOOP_RESET  2000  // milliseconds to wait before turningoff all points.


void ServoSetup()
{
  int  i;
  int  pos;
  
  big_loop = 0;
  
  //Reset the servo controller
  pinMode(ServoResetPin, OUTPUT);
  digitalWrite(ServoResetPin, LOW);
  delay(20);
  digitalWrite(ServoResetPin, HIGH);
  
  // set the data rate for the SoftwareSerial port
  ServoSerial.begin(38400);
  
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
  
  big_loop++;
  
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
    big_loop = 0;
  }

  if( big_loop == LOOP_RESET )
  {
    big_loop = 0;
    
    for (i = 0; i < NUM_POINTS; i++)
    {
        // motor is now idle so turn it off
        set_speed(i, 2);
        set_servo_parameters(i, 0, 15);
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
   ServoSerial.write(0x80);       //start byte
   ServoSerial.write(0x01);       //device id
   ServoSerial.write((byte)0x00);       //command number
   ServoSerial.write(servo);      //servo number
   ServoSerial.write(parameters); //parameters
   ServoSerial.flush();
}

void set_speed(byte servo, byte speedVal)
{
   //this function uses pololu mode command 1 to set speed
   //servo is the servo number (typically 0-7)
   //speedVal is servo speed (1=slowest, 127=fastest, 0=full)
   //set speedVal to zero to turn off speed limiting
   
   speedVal = speedVal & 0x7f; //take only lower 7 bits of the speed
   
   //Send a Pololu Protocol command
   ServoSerial.write(0x80);     //start byte
   ServoSerial.write(0x01);     //device id
   ServoSerial.write(0x01);     //command number
   ServoSerial.write(servo);    //servo number
   ServoSerial.write(speedVal); //speed
   ServoSerial.flush();
}

void position_7bit(byte servo, byte posValue)
{
  //this function uses pololu mode command 2 to set position  
  //servo is the servo number (typically 0-7)
  //posValue * range (set with command 0) adjusted by neutral (set with command 5)
  //determines servo position

   byte pos = posValue & 0x7f;     //get lower 7 bits of position

   //Send a Pololu Protocol command
   ServoSerial.write(0x80);     //start byte
   ServoSerial.write(0x01);     //device id
   ServoSerial.write(0x02);     //command number
   ServoSerial.write(servo);    //servo number
   ServoSerial.write(pos);     //position
   ServoSerial.flush();
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
   ServoSerial.write(0x80);     //start byte
   ServoSerial.write(0x01);     //device id
   ServoSerial.write(0x03);     //command number
   ServoSerial.write(servo);    //servo number
   ServoSerial.write(pos_hi);  //bits 8 thru 13
   ServoSerial.write(pos_low); //bottom 7 bits
   ServoSerial.flush();
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
   ServoSerial.write(0x80);    //start byte
   ServoSerial.write(0x01);    //device id
   ServoSerial.write(0x04);    //command number
   ServoSerial.write(servo);   //servo number
   ServoSerial.write(pos_hi);  //bits 8 thru 13
   ServoSerial.write(pos_low); //bottom 7 bits
   ServoSerial.flush();
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
   ServoSerial.write(0x80);    //start byte
   ServoSerial.write(0x01);    //device id
   ServoSerial.write(0x05);    //command number
   ServoSerial.write(servo);   //servo number
   ServoSerial.write(pos_hi);  //bits 8 thru 13
   ServoSerial.write(pos_low); //bottom 7 bits
   ServoSerial.flush();
}

// POINTS 
byte  PointCommand[NUM_POINTS];

void
PointsInit()
{
  int  i;

  ServoSetup();
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
      DisplayPrint((i+8),1,"C");
    else
      DisplayPrint((i+8),1,"S");
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

void
PointsMove(int  p, int  pos)
{
  // p is the point number as seen on the table
  PointCommand[p-1] = pos;
}


// SENSOR SECTION

// The sensor array maps sensor to actual pins
// We assume a maximum of 16 sensors 0-15
// Each sensor maps to a pin in the array, -1 being absent

#define  NUM_SENSORS  16

#define  SENSOR_FIELD_PIN  0
#define  SENSOR_FIELD_THRESHOLD  1
#define  SENSOR_FIELD_CURRENT  2
#define  SENSOR_FIELD_TICKER  3
#define  NUM_SENSOR_FIELDS  4

int  Sensors[NUM_SENSORS][NUM_SENSOR_FIELDS];

#define  SENSOR_ABSENT  -1
#define  SENSOR_THRESHOLD  800
#define  SENSOR_DEBOUNCE_CNT  50

void
SensorsInit()
{
  int  i;
  
  for ( i = 0; i < 16; i++ )
  {
      Sensors[i][SENSOR_FIELD_PIN] = SENSOR_ABSENT;
      Sensors[i][SENSOR_FIELD_THRESHOLD] = SENSOR_THRESHOLD;
      Sensors[i][SENSOR_FIELD_CURRENT] = 0;
      Sensors[i][SENSOR_FIELD_TICKER] = 0;
  }

  // DisplayPrint(0,2,"0123456789012345");  
  // sensors are labelled starting at 1 so subtraction is needed!
  Sensors[0][SENSOR_FIELD_PIN] = 0;
  Sensors[1][SENSOR_FIELD_PIN] = 1;
  Sensors[2][SENSOR_FIELD_PIN] = 2;
  Sensors[3][SENSOR_FIELD_PIN] = 3;
  Sensors[4][SENSOR_FIELD_PIN] = 4;
  Sensors[5][SENSOR_FIELD_PIN] = 5;
  Sensors[6][SENSOR_FIELD_PIN] = 12;
  Sensors[7][SENSOR_FIELD_PIN] = 7;
  Sensors[8][SENSOR_FIELD_PIN] = 8;
  // Sensors[10][SENSOR_FIELD_PIN] = 10;
  // Sensors[11][SENSOR_FIELD_PIN] = 11;
}

void
SensorsPoll()
{
  int  i;
  int  j;
  char  temp[2];
  
  for ( i = 0; i < 16; i++ )
  {
      if(Sensors[i][SENSOR_FIELD_PIN] != SENSOR_ABSENT)
      {
        // It's a valid sensor so check it...
        Sensors[i][SENSOR_FIELD_CURRENT] = analogRead(Sensors[i][SENSOR_FIELD_PIN]);
                
        if ( (Sensors[i][SENSOR_FIELD_CURRENT] < SENSOR_THRESHOLD) && \
             (Sensors[i][SENSOR_FIELD_TICKER] == 0) )
        {
          Sensors[i][SENSOR_FIELD_TICKER] = SENSOR_DEBOUNCE_CNT;
          EventsHandleSensorTrigger(i);
        }
        if ( Sensors[i][SENSOR_FIELD_TICKER] > 0 )
        {
          Sensors[i][SENSOR_FIELD_TICKER]--;
        }
      }
      else
      {
        // marked as absent
      }
  }
}

#define  NUM_SECTIONS  8

#define  SECTIONS_START  22
#define  SECTIONS_INC    2

void
SectionsInit()
{
  int  i;
  
  for(i = 0; i < NUM_SECTIONS; i++ )
  {
    pinMode(SECTIONS_START + (i*SECTIONS_INC), OUTPUT);
    digitalWrite(SECTIONS_START + (i*SECTIONS_INC), HIGH);
  }


  for(i = 1; i <= NUM_SECTIONS; i++ )
  {
    SectionIsolate(i);
    delay(150);
    SectionActivate(i);
  }
}


// Sections are numbered 1-8
// Relays are turned ON by setting LOW
// Sections are isolated by relayes being ON
void
SectionIsolate(int  s)
{
  digitalWrite(SECTIONS_START + ((s-1)*SECTIONS_INC) , LOW);
  DisplayPrint(s-1,1,"O");
}

void
SectionActivate(int  s)
{
  digitalWrite(SECTIONS_START + ((s-1)*SECTIONS_INC), HIGH);
  DisplayPrint(s-1,1,"*");
}



#define  NUM_MOTORS  2

typedef struct {
  int  in1_pin;
  int  in2_pin;
  int  enable_pin;
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
  Motors[1].in1_pin = 5;
  Motors[1].in2_pin = 6;
  Motors[1].enable_pin = 8;

/*
TIMER 4
Value                             Divisor                      Frequency
0×01                                  1                           31250 hz
0×02                                  8                            3926.25 hz
0×03                                 32                             976.5625 hz
0×04                                 64                             488.28125 hz               // default
0×05                                 128                            244.140625 hz
Code:                 TCCR4B = (TCCR4B & 0xF8) | value ;
*/
  TCCR4B = (TCCR4B & 0xF8) | 0x04 ;  // set PWM frequency up to 31kHz so I can't hear it!
  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  
}

// Motors are 1 indexed.
// A negative speed implies reverse (-255 to +255)

#define  MOTOR_ZERO_PT  30

void
MotorSetSpeed(int  motor, int  spd)
{
  boolean reverse = false;
  char  temp[5];
  
  if(motor > NUM_MOTORS)
    return; //idiot
  
  if(spd < 0)
    reverse = true;
    
  spd = abs(spd);
   
  if(spd > 255)
    spd = 255;  // clip it just in case!

  if(reverse)
  {
    digitalWrite(Motors[motor-1].in1_pin,HIGH);
    digitalWrite(Motors[motor-1].in2_pin,LOW);    
  }
  else
  {
    digitalWrite(Motors[motor-1].in1_pin,LOW);
    digitalWrite(Motors[motor-1].in2_pin,HIGH);    
  }

  if (spd <= MOTOR_ZERO_PT)
  {
    digitalWrite(Motors[motor-1].in1_pin,HIGH);
    digitalWrite(Motors[motor-1].in2_pin,HIGH);    
  }
  analogWrite(Motors[motor-1].enable_pin, spd);
}

void
MotorLoop()
{
  int   j, k;
  char  temp[17];
  
  j = analogRead(10);
  j = map(j,0,1023,-255,255);
  
  //k = analogRead(11);
  //k = map(k,0,1023,-255,255);
  
  MotorSetSpeed(1, j);
  //MotorSetSpeed(2, k);  

  if( (ticker % 50) == 0 )
  {
    sprintf(temp,"%04d",j);
    DisplayPrint(0,0,temp);
  }
}

//DISPLAY
void
DisplayInit()
{
  Serial2.begin(9600);

  Serial2.write("DC");
  Serial2.write((byte)0x00);
  Serial2.write("CL");
  DisplayPrint(0,0,"Initialising...");
}

void
DisplayPrint(int  x, int  y, char *str)
{
  Serial2.write("TP");
  Serial2.write((byte)x);
  Serial2.write((byte)y);  
  Serial2.write("TT");
  Serial2.print(str);
  Serial2.write(0x0D);  
  Serial2.flush();
}

void
DisplayGetSize(int  *x, int  *y)
{
  *x = 20;
  *y = 5;
}


// APPLICATION

// mapping of sensors to meaningful events (sensors are the numbers on the right).
#define  E_TRAM_AT_VILLAGE  0
#define  E_TRAM_AT_STATION  1
#define  E_TRAIN_X6  2
#define  E_TRAIN_X7  7
#define  E_TRAIN_X8  8
#define  E_TRAIN_X1A 4
#define  E_TRAIN_X3A 3
#define  E_TRAIN_X1  5
#define  E_TRAIN_X2  6


#define  E_MOTOR_SET_PT_1  10
#define  E_MOTOR_SET_PT_1  11
#define  E_UNDEFINED  1000

#define  DEST_PLATFORM_1  0
#define  DEST_PLATFORM_2  1
#define  DEST_OUTER_LOOP  3
#define  DEST_INNER_LOOP  4

int  destination;
int  inner_loop_ctr;
int  outer_loop_ctr;

void
AppInit()
{
  destination = DEST_PLATFORM_1;
  inner_loop_ctr = 0;
  outer_loop_ctr = 0;
}

int
MapSensorToEvent(int  s)
{
  switch(s)
  {
    case 0:
            DisplayPrint(0,4,"TRAM @ VILLAGE  ");
            return E_TRAM_AT_VILLAGE;
            break;
    case 1:
            DisplayPrint(0,4,"TRAM @ STATION  ");
            return E_TRAM_AT_STATION;
            break;
    case 2:
            DisplayPrint(0,4,"APPROACH STATION");
            return E_TRAIN_X6;
            break;
    case 3:
            DisplayPrint(0,4,"INNER LINE PTS  ");
            return E_TRAIN_X3A;
            break;
    case 4:
            DisplayPrint(0,4,"OUTER LINE PTS  ");
            return E_TRAIN_X1A;
            break;
    case 5:
            DisplayPrint(0,4,"OUTER LINE      ");
            return E_TRAIN_X1;
            break;
    case 6:
            DisplayPrint(0,4,"STATION LINE    ");
            return E_TRAIN_X2;
            break;
    case 7:
            DisplayPrint(0,4,"2nd PLATFORM    ");
            return E_TRAIN_X7;
            break;
    case 8:
            DisplayPrint(0,4,"1st PLATFORM    ");
            return E_TRAIN_X8;
            break;
    default:
            return E_UNDEFINED;
            break;
  }
}

void
e_delay(unsigned long  d)
{
  unsigned long  t;
  
  t = millis() + d;
  
  while( millis() < t)
  {
    SensorsPoll();
    MotorLoop();
    ServoLoop();
    TrainsLoop();
    delay(1);
  }
}


// i is the sensor as labelled on the layout
void
EventsHandleSensorTrigger(int  i)
{
  int   e,s,d;
  char  temp[16];
  
  e = MapSensorToEvent(i);

  switch(e)
  {
    case  E_TRAM_AT_VILLAGE:
          if(TrainsGetDest(0) != E_TRAM_AT_VILLAGE)
            break;
          TrainsSetDest(0, E_TRAM_AT_STATION);
          d = 10000;  //10 seconds.
          TrainsStop(0);
          TrainsWait(0, d);
          // I've hacked in code to make it run in reverse!!
          break;
    case  E_TRAM_AT_STATION:
          if(TrainsGetDest(0) != E_TRAM_AT_STATION)
            break;
          TrainsSetDest(0, E_TRAM_AT_VILLAGE);
          d = 10000 ; // analogRead(11) * 2;
          TrainsStop(0);
          TrainsWait(0, d);
          // I've hacked in code to make it run in reverse!!
          break;
    case  E_TRAIN_X6:
          if( destination == DEST_PLATFORM_1)
          {
            PointsMove(1, POINT_STRAIGHT);
            PointsMove(2, POINT_STRAIGHT);
            PointsMove(3, POINT_STRAIGHT);
            PointsMove(8, POINT_STRAIGHT);
            PointsFlush();
            DisplayPrint(0,3,"NEXT -> P2   ");
            //delay(3000);
            //SectionActivate(4);
          }
          if ( destination == DEST_PLATFORM_2)
          {
            PointsMove(8, POINT_SWITCH);
            PointsMove(3, POINT_SWITCH);
            PointsMove(1, POINT_SWITCH);
            PointsMove(2, POINT_SWITCH);
            PointsFlush();
            DisplayPrint(0,3,"NEXT -> P1   ");
          }          
          break;
    case  E_TRAIN_X7: //Platform 2
          destination = DEST_PLATFORM_1;
          PointsMove(3, POINT_SWITCH);
          PointsFlush();
          DisplayPrint(0,2,"NEXT: Platform 1");
          break; 
    case  E_TRAIN_X8:
          destination = DEST_PLATFORM_2;
          PointsMove(1, POINT_STRAIGHT); 
          PointsFlush();
          DisplayPrint(0,2,"NEXT: Platform 2");   
          break;
    case  E_TRAIN_X1A:
          outer_loop_ctr++;
          sprintf(temp, "%1d", outer_loop_ctr);
          DisplayPrint(7,0,temp);

          if(outer_loop_ctr == 2)
          {
            DisplayPrint(0,3,"NEXT -> INNER");
            PointsMove(4, POINT_SWITCH);
            PointsMove(5, POINT_SWITCH);
            outer_loop_ctr = 0;
            DisplayPrint(7,0,"0");
          }
          PointsMove(7, POINT_STRAIGHT);
          PointsMove(6, POINT_STRAIGHT);
          PointsFlush();          
          break;
    case  E_TRAIN_X3A:
          inner_loop_ctr++;
          sprintf(temp, "%1d", inner_loop_ctr);
          DisplayPrint(6,0,temp);
          
          if(inner_loop_ctr == 2)
          {
            DisplayPrint(0,3,"NEXT -> OUTER");
            PointsMove(6, POINT_SWITCH);
            PointsMove(7, POINT_SWITCH);
            PointsFlush();
            SectionIsolate(8);
            inner_loop_ctr = 0;
            DisplayPrint(6,0,"0");
            e_delay(5000);
            SectionActivate(8);
          }
          PointsMove(2, POINT_STRAIGHT);
          PointsMove(3, POINT_STRAIGHT);
          PointsMove(4, POINT_STRAIGHT);
          PointsMove(5, POINT_STRAIGHT);
          PointsFlush();
          break;
     case E_TRAIN_X1:
          PointsMove(6, POINT_STRAIGHT);
          PointsMove(7, POINT_STRAIGHT);
          PointsFlush();
          break;
     case E_TRAIN_X2:
          if( destination == DEST_PLATFORM_1)
          {
            PointsMove(8, POINT_STRAIGHT);
            PointsFlush();
          }
          if ( destination == DEST_PLATFORM_2)
          {
            PointsMove(8, POINT_SWITCH);
            PointsFlush();
          }          
          break;
     default:
          DisplayPrint(0,3,"WTF? -> RESET");
          PointsMove(1, POINT_STRAIGHT);
          PointsMove(2, POINT_STRAIGHT);
          PointsMove(3, POINT_STRAIGHT);
          PointsMove(4, POINT_STRAIGHT);
          PointsMove(5, POINT_STRAIGHT);
          PointsMove(6, POINT_STRAIGHT);
          PointsMove(7, POINT_STRAIGHT);
          PointsMove(8, POINT_STRAIGHT);
          PointsFlush();
          break;
  }
}

typedef enum { stopped, stopping, starting, running , waiting_timer, waiting_controller } TrainStates;

typedef struct {
  char  *name;
  TrainStates  state;
  int   start_speed;
  int   cruise_speed;
  int   crawl_speed;
  int   current_speed;
  int   set_speed;
  int   current_section;
  int   current_sensor;
  unsigned long  timer;
  int   startup_steps;
  unsigned long  platform1_lag;
  unsigned long  platform2_lag;
  unsigned long  village_lag;
  unsigned long  station_lag;
  int   motor;
  int   destination;
} Train;

#define  NUM_TRAINS  1

Train  Trains[NUM_TRAINS];

void
TrainsInit()
{
  Trains[0].name = "Tram Green";
  Trains[0].state = starting;
  Trains[0].start_speed = 150;
  Trains[0].cruise_speed = 100;
  Trains[0].crawl_speed = 70;
  Trains[0].current_speed = 0;
  Trains[0].set_speed = Trains[0].cruise_speed;
  Trains[0].current_section = 0; // NEEDS definition!!
  Trains[0].current_sensor = 0;  // NEEDS definition!!
  Trains[0].timer = 0;
  Trains[0].startup_steps = 20;
  Trains[0].platform1_lag = 0;
  Trains[0].platform2_lag = 0;
  Trains[0].village_lag = 0;
  Trains[0].station_lag = 0;
  Trains[0].motor = 2;
  Trains[0].destination = E_TRAM_AT_STATION;
}

void
TrainsStart(int  t)
{
  Trains[t].state = starting;
}

void
TrainsStop(int  t)
{
  Trains[t].state = stopping;
}

void
TrainsWait(int  t, long  d)
{
  Trains[t].timer = d;
}

void
TrainsSetDest(int  t, int  d)
{
  Trains[t].destination = d;
}

int
TrainsGetDest(int  t)
{
  return Trains[t].destination ;
}


void
TrainsLoop()
{
  int  i,d;
  char  temp[8];
  
  if(ticker % 20 == 0 )
    d = 1;
  else
    d = 0;
  
  for(i = 0; i < NUM_TRAINS; i++)
  {
    switch(Trains[i].state)
    {
      case stopped:
        Trains[i].startup_steps = 10;
        if(Trains[i].timer)
          Trains[i].state = waiting_timer;
        break;
        
      case stopping:
        if( Trains[i].current_speed != 0)
        {
          if( Trains[i].current_speed < 0)
            Trains[i].current_speed += d;
          else
            Trains[i].current_speed -= d;      
        }
        else
          Trains[i].state = stopped;
        MotorSetSpeed(Trains[i].motor,  Trains[i].current_speed);
        break;
        
      case starting:
        // assumes set_speed set by a function elsewhere
        if(Trains[i].startup_steps > 0)
        {
          Trains[i].startup_steps--;
          
          if( Trains[i].set_speed < 0)
          {
            MotorSetSpeed(Trains[i].motor,  -1 * Trains[i].start_speed);
            Trains[i].current_speed = -1 * Trains[i].crawl_speed;
          }
          else
          {
            MotorSetSpeed(Trains[i].motor,  Trains[i].start_speed);
            Trains[i].current_speed = Trains[i].crawl_speed;
          }     
        }
        else
        {
          if( Trains[i].set_speed < 0)
          {
            if(Trains[i].set_speed < Trains[i].current_speed)
              Trains[i].current_speed -= d;
          }
          else
          {
            if(Trains[i].set_speed > Trains[i].current_speed)
              Trains[i].current_speed += d;
          }    
          MotorSetSpeed(Trains[i].motor,  Trains[i].current_speed);
        }
        
        if( Trains[i].set_speed == Trains[i].current_speed)
          Trains[i].state = running;
          
        break;
        
      case running:
        break;
        
      case waiting_timer:
        if(Trains[i].timer > 0)
          Trains[i].timer--;
        else
        {
          Trains[i].state = starting;
          // FILTHY TRAM HACK
          if (i == 0)
            Trains[i].set_speed = -1 * Trains[i].set_speed;
        }
        break;
        
      case waiting_controller:
        break;
      default:
        break;
        
    }

  }
}

void
PointsExercise()
{
  int  i;
  
  for( i = 0; i < 8 ; i ++)
  {
    PointsMove(i+1,POINT_SWITCH);
    PointsFlush();
    delay(2000);
  }
  for( i = 0; i < 8 ; i ++)
  {
    PointsMove(i+1,POINT_STRAIGHT);
    PointsFlush();
    delay(2000);
  }

}
void
SectionsExercise()
{
  int  i;
  
  for( i = 0; i < 8 ; i ++)
  {
    SectionIsolate(i+1);
    delay(200);
  }
  for( i = 0; i < 8 ; i ++)
  {
    SectionActivate(i+1);
    delay(200);
  }

}

void
setup()
{
  Serial.begin(9600);

  DisplayInit();
  MotorsInit();
  SensorsInit();
  PointsInit();
  SectionsInit();
  TrainsInit();
  AppInit();
  DisplayPrint(0,0,"               ");
}

void
loop()
{ 
  char  temp [16];
  
  SensorsPoll();
  MotorLoop();
  ServoLoop();
  TrainsLoop();
  PointsExercise();
  //SectionsExercise();
  
  ticker++;
  if( (ticker % 100) == 0 )
  {
    ltoa(ticker,temp,8);
    DisplayPrint(9,0,temp);
  }
  
}

