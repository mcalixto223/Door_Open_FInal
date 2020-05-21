// Michael Calixto
/* Version 1. Here's the testing for
 *  Heartbeat LED
 *  Ultrasonic
 *  LCD Display
 *  Motor
 *  Encoder
 */

// Libraries
  #include <LiquidCrystal.h> // Library for LCD Display
  #include <NewPing.h> // Library for Ultrasonic Sensor

// Input Pins
  // LED Heartbeat
    const int LED1=13;

  // Pins for Ultrasonic
    #define Trigger_Pin 6 // Insert in pin 6
    #define Echo_Pin 6  // Insert in pin 6
    #define Max_Distance 100 // Maximum Distance for Sensor to read

  // Pins for LCD
    LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
    
  // Pins for Motor
    #define motor_open_pin 10
    #define motor_close_pin 9

  // Encoder
    #define ch_A_pin 7
    #define ch_B_pin 8

  // Timers Statements
    unsigned long int Ultrasonic_timer1;
    unsigned long int LCD_timer2;
    unsigned long int door_control_timer3;
    unsigned long int update_encoder_timer_in_ms_timer4;
    unsigned long int print_timer5;
    unsigned long int Heartbeat_timer6;
  
  
  // void Statements
    void Heartbeat(void);
    void ultrasonic(void);
    void update_lcd(void);
    void door_control(void);
    void read_encoder(void);
    void timers(void);

  // void Status for Door
    void motor_open_door(void);
    void motor_close_door(void);
    void motor_stop(void);


  // Ultrasonic Sensor Reading Values
    NewPing sonar(Trigger_Pin, Echo_Pin, Max_Distance);
    float duration_sensor, distance, user;
    
  // Ultrasonic Average Values
    int iterations = 5;

  // Door status
    int is_door_closed(void);
    int is_door_open(void);
    int is_user_detected(void);
    int user_detected = 0;

  // Encoder readings
    unsigned int encoder_state = 0;
    unsigned long encoder_pulse_duration_in_ms;
    unsigned int motor_position = 0;

  // Door State
  int  door_state = 0;
// put your setup code here, to run once:
void setup() 
{
  // Heartbeat LED
    pinMode(LED1,OUTPUT);
  
  // Read from Ultrasonic
    Serial.begin(9600);
  
  // LCD First Line statement
    lcd.begin(16,2);
    lcd.print(distance);
   
   // Motor Outputs
    pinMode(motor_open_pin,OUTPUT);
    pinMode(motor_close_pin,OUTPUT);

   // Encoder Inputs
    pinMode(ch_A_pin,INPUT);
    pinMode(ch_B_pin,INPUT);
}

// put your main code here, to run repeatedly:
void loop()
{

  ultrasonic();
  update_lcd();
  door_control();
  read_encoder();
  Heartbeat();
  timers();
  
}
// Program for Ultrasonic
void ultrasonic()
{
  if(Ultrasonic_timer1>=1)
  {
    Ultrasonic_timer1=0;
    duration_sensor = sonar.ping_median(iterations);
    distance = (duration_sensor/2)*0.0343;
    Serial.print("Distance = ");
    user=distance;
    if (distance>=100 || distance <= 5)
    {
      Serial.println("Not In Range");
      user_detected = 0;
    }
    else
    {
      Serial.print(distance);
      Serial.println(" cm"); 
      user_detected = 1;
    }
  }
}
// Program for LCD
void update_lcd()
{
  if(LCD_timer2>=2)
  {
    LCD_timer2=0;
     lcd.begin(16,2);

     lcd.print("Dist.(cm)=  ");
     lcd.print(distance);
     
     // Status for Door
      lcd.setCursor(0,1);
      lcd.print("Door ");
     
      // Added cases ****
      switch(door_state)
      {
        case 0:// closed
        lcd.setCursor(5,2);
        lcd.print("Closed ");
        break;

        case 1: // opening
        lcd.setCursor(5,2);
        lcd.print("Opening");
        break;

        case 2: // open
        lcd.setCursor(5,2);
        lcd.print("Open   ");
        break;

        case 3: // closing
        lcd.setCursor(5,2);
        lcd.print("Closing");
        break;
      }

      lcd.setCursor(13,1);
      if(is_user_detected() == 1)
      {
        lcd.print("X");
      }
      else
      {
      lcd.print("-");
      }

      lcd.setCursor(14,1);
      
      if(is_door_open() ==1)
      {
      lcd.print("O");
      }
      else
      {
      lcd.print("-");
      }

      lcd.setCursor(15,1);
      
      if(is_door_closed() == 1)
      {
      lcd.print("C");
      }
      else
      {
      lcd.print("-");
               }  
      
      /*
      if (is_door_closed == 1)
      {
        lcd.setCursor(8,14);
        lcd.print("C");
      }
      if (is_door_open == 1)
      {
        lcd.setCursor(8,14);
        lcd.print("O");
      }
      if (is_user_detected == 1)
      {
        lcd.setCursor(8,14);
        lcd.print("X");
      }
      
     */
  }
}

// READ ENCODER
void read_encoder(void)
{
  static int encoder_case;
  int encoder_case_temp = 0;

  if(digitalRead(ch_A_pin))
    encoder_case_temp+=1;
  if(digitalRead(ch_B_pin))
    encoder_case_temp+=2;
  
    if(encoder_case != encoder_case_temp)
    {
      Serial.print("case = ");
      Serial.print(encoder_case_temp);
      Serial.print("  position = ");
      Serial.println(motor_position);
    }
      

  switch(encoder_case)
  {
    case 0:
      if(encoder_case_temp == 2)
        motor_position--;
      if(encoder_case_temp == 1)
        motor_position++;
      break;
    case 1:
      if(encoder_case_temp == 0)
        motor_position--;
      if(encoder_case_temp == 3)
        motor_position++;
      break;
        case 3:
      if(encoder_case_temp == 1)
        motor_position--;
      if(encoder_case_temp == 2)
        motor_position++;
      break;
      
    case 2:
      if(encoder_case_temp == 3)
        motor_position--;
      if(encoder_case_temp == 0)
        motor_position++;
      break;

  }
  

  encoder_case = encoder_case_temp;
}

// Question: Is the door closed?
int is_door_closed(void)
{
  if(motor_position <=10)
  return 1;
  // Do this
 else
 return 0;
}
// Question: Is door open?
int is_door_open(void)
{
  if(motor_position >=90)
  return 1;
  else
  return 0;
  // Do This
 
 return 1;
}
// Question: Is the User Detected?

int is_user_detected (void)
{
  if(user_detected == 0)
  return 0;
  if(user_detected == 1)
  return 1;
}


// Door Control
void door_control(void)
{
  switch(door_state)
  {
    case 0:  // close
    
      motor_stop();
      if(is_user_detected() == 1);
      {       
        // go to opening state
          door_state = 1;
                 
      }
      break;

     case 1:  // opening
      motor_open_door();
      if(is_door_open() == 1)
      {

          door_state = 2;
          door_control_timer3 = 0;
      }
      break;
      
      case 2:  // open position
        motor_stop();
        if(door_control_timer3 >= 50)
        
          door_state = 3;
        
        if(is_user_detected() == 1)
        
          door_control_timer3 = 0;

          break;
        
       case 3: // closing the door
        motor_close_door();
        if(is_door_closed() == 1)
        
          door_state = 0;
        
        if(is_user_detected() ==1);
        {
          door_state = 1;
        }
        break;

        default:
          door_state = 0;
          break;
        
  }
  
}



// (Void) Door Position
void motor_open_door(void)
{
  digitalWrite(motor_open_pin, HIGH);
  digitalWrite(motor_close_pin, LOW);
  
}

void motor_close_door(void)
{
  digitalWrite(motor_open_pin, LOW);
  digitalWrite(motor_close_pin, HIGH);
}

void motor_stop(void)
{
  digitalWrite(motor_open_pin, LOW);
  digitalWrite(motor_close_pin, LOW);
}


// LED blinking
void Heartbeat()
{
  if(Heartbeat_timer6<10)
    {
      digitalWrite(LED1,HIGH);
    }
  else
    {
      digitalWrite(LED1,LOW);
        if(Heartbeat_timer6>=20)
          {
              Heartbeat_timer6=0;
          }
  
    }
}

// Timers
void timers(void)
{
  static unsigned long ms_runtime;
  static unsigned int one_ms_timer;

  if(millis() > ms_runtime)
  {
    ms_runtime++;
    one_ms_timer++;  
  }
 else if(ms_runtime > millis())
    ms_runtime = millis();

  if(one_ms_timer > 99) // every 100 ms
  {
    one_ms_timer = 0;
    Ultrasonic_timer1++;
    LCD_timer2++;
    door_control_timer3;
    update_encoder_timer_in_ms_timer4++;
    print_timer5++;
    Heartbeat_timer6++;
  }
}
