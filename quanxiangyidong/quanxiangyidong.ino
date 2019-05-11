#include <Kinematics.h>
#include <PS2X_lib.h>  //for v1.6
#define PS2_DAT        A15  //13   
#define PS2_CMD        11  //11
#define PS2_SEL        10  //10
#define PS2_CLK        12  //12
#define rumble      false
#define pressures   false
int pwm1 = 5, pwm11 = 4, pwm2 = 3, pwm22 = 2, pwm3 = 9, pwm33 = 8, pwm4 = 7, pwm44 = 6;
#define MOTOR_MAX_RPM 250        // motor's maximum rpm
#define WHEEL_DIAMETER 0.038      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.128   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.128   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
int h=0;//h为程序开关

Kinematics::output rpm;
Kinematics::output pwm;
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);
float linear_vel_x = 0;  // 1 m/s
float linear_vel_y = 0;  // 0 m/s
float angular_vel_z = 0; // 1 m/s

PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;
// Reset func 
void (* resetFunc) (void) = 0;
void Motor_refresh()
{
  pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);
  //pwm.motor1 = -200;
 //pwm.motor2 = -200;
 // pwm.motor3 = -200;
  //pwm.motor4 = -200;
  
  if (pwm.motor1 < 0) {
    int pwm_1 = -1 * pwm.motor1;
        if(pwm_1>255)
    pwm_1=255;
    analogWrite(pwm11, pwm_1);
    analogWrite(pwm1,0);
    Serial.print("pwm1=");
    Serial.println(pwm.motor1);
  }
  else
  {
     if(pwm.motor1>255)
    pwm.motor1=255;
    analogWrite(pwm11, 0);
    analogWrite(pwm1,pwm.motor1);
    Serial.print("pwm1=");
    Serial.println(pwm.motor1);
  }
  if (pwm.motor2 < 0)
  {
    int pwm_2 = -1 * pwm.motor2;
    if(pwm_2>255)
    pwm_2=255;
    analogWrite(pwm2, 0);
    analogWrite(pwm22, pwm_2);
    Serial.print("pwm2=");
    Serial.println(pwm.motor2);
  }
  else
  {
     if(pwm.motor2>255)
    pwm.motor2=255;
    analogWrite(pwm22, 0);
    analogWrite(pwm2, pwm.motor2);
    Serial.print("pwm2=");
    Serial.println(pwm.motor2);
  }
  if (pwm.motor3 < 0)
  {
    int pwm_3 = -1 * pwm.motor3;
   if(pwm_3>255)
    pwm_3=255;
    analogWrite(pwm33, pwm_3);
    analogWrite(pwm3, 0);
    Serial.print("pwm3=");
    Serial.println(pwm.motor3);
  }
  else
  {
    if(pwm.motor3>255)
    pwm.motor3=255;
    analogWrite(pwm33, 0);
    analogWrite(pwm3, pwm.motor3);
    Serial.print("pwm3=");
    Serial.println(pwm.motor3);
  }
  if (pwm.motor4 < 0)
  {
    int pwm_4 = -1 * pwm.motor4;
   if(pwm_4>255)
    pwm_4=255;
    analogWrite(pwm4, 0);
    analogWrite(pwm44, pwm_4);
    Serial.print("pwm4=");
    Serial.println(pwm.motor4);
  }
  else
  {
     if(pwm.motor4>255)
    pwm.motor4=255;
    analogWrite(pwm4, pwm.motor4);
    analogWrite(pwm44, 0);
    Serial.print("pwm4=");
    Serial.println(pwm.motor4);
  }
}
void  Go_forward()
{
  linear_vel_x = 0;  // 1 m/s
  linear_vel_y = 0.3;  // 0 m/s
  angular_vel_z = 0; // 1 m/s
  Motor_refresh();
}

void Go_back()
{
  linear_vel_x = 0;  // 1 m/s
  linear_vel_y = -0.3;  // 0 m/s
  angular_vel_z = 0; // 1 m/s
    Motor_refresh();
}

void Go_left()
{
  linear_vel_x = 0.3;  // 1 m/s
  linear_vel_y = 0;  // 0 m/s
  angular_vel_z = 0; // 1 m/s
    Motor_refresh();
}

void Go_right()
{
  linear_vel_x = -0.3;  // 1 m/s
  linear_vel_y = 0;  // 0 m/s
  angular_vel_z = 0; // 1 m/s
    Motor_refresh();
}
void MOTOR_stop()
{
  linear_vel_x = 0;  // 1 m/s
  linear_vel_y = 0;  // 0 m/s
  angular_vel_z = 0; // 1 m/s
    Motor_refresh();
}
void rotateright()
{
  linear_vel_x = 0;  // 1 m/s
  linear_vel_y = 0;  // 0 m/s
  angular_vel_z = 3; // 1 m/s
    Motor_refresh();
}
void rotateleft()
{
  linear_vel_x = 0;  // 1 m/s
  linear_vel_y = 0;  // 0 m/s
  angular_vel_z = -3; // 1 m/s
    Motor_refresh();
}
void rocker()
{
  double y,x,z;
  y=ps2x.Analog(PSS_LY);
    Serial.print("y=");
  Serial.print(y);
  y=((y-128)/128)*0.3;
    Serial.print("y=");
  Serial.print(y);
  x=ps2x.Analog(PSS_LX);
  x=(x-128)/128*0.3;
  Serial.print("x=");
  Serial.print(x);
  z=ps2x.Analog(PSS_RY);
  z=(z-128)/128*0.3;
  Serial.print("z=");
  Serial.print(z);
  linear_vel_x = x;  // 
  linear_vel_y = y;  // 
  angular_vel_z = z; // 
  Motor_refresh();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if(error == 0){
    Serial.print("Found Controller, configured successful ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
  }
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.println("GuitarHero Controller found ");
      break;
  case 3:
      Serial.println("Wireless Sony DualShock Controller found ");
      break;
   }  
}

void loop() {
    Kinematics::output rpm;
    Kinematics::output pwm;
    //if(h==1)
    rocker();
      if(error == 1){ //skip loop if no controller found
    resetFunc();
  }
  
  if(type == 2){ //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller 
   
    
  }
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
    {
      Serial.println("Select is being held"); 
    }     

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Go_forward();
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Go_right();
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Go_left();
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      Go_back();
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L2)){
        MOTOR_stop();
        Serial.println("L2 pressed");
      }
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    {
      Serial.println("Circle just pressed");
      if(h==0)
      h=1;
      if(h==1)
      h=0;
    }
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    {
      Serial.println("X just changed");
      rotateleft();
    }
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    {
      Serial.println("Square just released");     
      rotateright();
    }

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
    }     
  }
}
