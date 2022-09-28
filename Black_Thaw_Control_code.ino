#include <Servo.h>
#include <math.h>
#include <stdint.h>

////////////////////////Motor setting
#define Motor_L 8 // Motor_L, Pin 2
#define Motor_R 10 // Motor_R, Pin 3

#define ML_Dir 24 // Motor_L_Direction, Pin 43
#define MR_Dir 26 // Motor_R_Direction, Pin 45

#define Brake 30 // Brake, Pin 35

//#define Max_angle_deviation 
#define Max_target_angle 47
#define Min_target_angle -24

#define min_speed 100
#define max_speed 180

int motor_speed[2] = {50, 56}; // R,L
short Motor_State = 0;           // 0:off 1:go 2:back

/////////////////////////Encoder setting
#define encpin1 13
#define encpin2 15

#define wheel_ratio 1.49

int degree1, degree2 = 0;
int prev_degree1, prev_degree2 = 0;
float encgap_nz1, encgap_nc1 = 0;
float encgap_nz2, encgap_nc2 = 0;

float enccount1 = 0;
float enccount2 = 0;

float deg_int1; // 각도 차이
float deg_int2;

float enc2cpl; //엔코더 회전각 > 커플링 회전각
float enc2cp2;

float distance1; // l=r*theta
float distance2;

float angle_velo1;
float angle_velo2;

/////////////////////////PID setting
#define DELAY_TIME 10
//#define dt (DELAY_TIME/1000.0)
unsigned long curt_time = 0;
unsigned long prev_time = 0;
double timeint = 0;

#define veloref 7

float Kp = 0.5;
float Ki = 0.2;
float Kd = 0.2;

float Pterm[2] = {0, 0};
float Iterm[2] = {0, 0};
float Dterm[2] = {0, 0};

float u[2] = {0, 0};
float prev_u[2] = {0, 0};
float error[2] = {0, 0};
float prev_error[2] = {0, 0};

/////////////////////////Cylincder Setting
#define Cylinder 6   // Cylinder, Pin
#define Cyl_Dir 22   // Cylinder_Direction, Pin
#define Poten A0     //가변저항 Analog 0 
#define Straight 540 // Potentiometer Straight value

int cylDuty[2] = {200, 204}; // pull,push

int target_angle = 0;
int past_target_angle = 0;
int target_resistance_L = 0;
int target_resistance_R = 0;

/////////////////////////Servo Setting
Servo myservo;

#define Servo_Pin 11
#define Standard 3
#define Servo_height 74
#define Servo_TF_Between 9.5

/// Bottom angle ///

String TF_data;

int Servo_PWM_B = 0;
int TF_length_B = 0;
float TF_angle_B = 0;
double TF_angle_rad_B = 0;
double angle_rad_B = 0;
double target_angle_Servo_B = 0;
double angle_deg_B = 0;

String TF_angle_T_str;
String TF_length_T_str;

/// Top angle ///
int Servo_PWM_T = 0;
int TF_length_T = 0;
float TF_angle_T = 0;
double TF_angle_rad_T = 0;
double angle_rad_T = 0;
double target_angle_Servo_T = 0;
double angle_deg_T = 0;

 String TF_angle_B_str;
 String TF_length_B_str;


/// WaterPump Setting///
#define WaterPump 4
#define WaterPump_Dir 20
#define WaterPump_Duty 255

int Check = 0;
int angle = 0;

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);

    // Motor
    pinMode(Motor_L, OUTPUT); // Motor_L, Output
    pinMode(Motor_R, OUTPUT); // Motor_R, Output

    analogWrite(Motor_L, 0); // Motor_L, Duty 0%
    analogWrite(Motor_R, 0); // Motor_R, Duty 0%

    pinMode(ML_Dir, OUTPUT);   // ML_Dir, Output
    digitalWrite(ML_Dir, LOW); // ML_Dir, '0'

    pinMode(MR_Dir, OUTPUT);    // MR_Dir, Output
    digitalWrite(MR_Dir, HIGH); // MR_Dir, '0'

    // Brake
    pinMode(Brake, OUTPUT);   // Brake, Output
    digitalWrite(Brake, LOW); // Brake, '0'

    // Encoder
    pinMode(encpin1, INPUT);
    pinMode(encpin2, INPUT);

    // Poten
    pinMode(Poten, INPUT); //가변저항

    // Cylinder
    pinMode(Cylinder, OUTPUT); // Cylinder, Output
    analogWrite(Cylinder, 0);  // Cylinder, Duty 0%

    pinMode(Cyl_Dir, OUTPUT);   // Cyl_Dir, Output
    digitalWrite(Cyl_Dir, LOW); // Cyl_Dir, '0'

    /// Servo&Waterpump setup///

    // Servo setup
    myservo.attach(Servo_Pin);
    myservo.write(Standard);

    // WaterPump PWM
    pinMode(WaterPump, OUTPUT);
    analogWrite(WaterPump, 0);

     target_angle = 0;
     angle_straight();

     motor_run();
}

void loop()
{

    getpwm();
    motor_run();

        Serial.print(" poten_val : ");
        Serial.println(analogRead(Poten));
        
    ///////////조향각도 제어///////////

    if (Serial2.available())
    {    
            Roadangle();
    }

    if (Serial1.available())
    {
      Cylinder_stop();
    TF_data = Serial1.readString();
    Serial.print("TF_data : ");
    Serial.print(TF_data);

    if(TF_data == "a"){
       motor_stop();
       Cylinder_stop();
       Serial.println("if OK");

    }else {
        Cylinder_stop();
        ReadTF(); // motor available 기능 
        //motor run
        motor_run();
    }
  }
     delay(DELAY_TIME);
}

void getpwm(void)
{
    ////////////////////////////엔코더 값 읽기
    encgap_nz1 = (float)pulseIn(encpin1, HIGH, 3000);
    encgap_nc1 = 879.8 - (float)pulseIn(encpin1, LOW, 3000);

    encgap_nz2 = (float)pulseIn(encpin2, HIGH, 3000);
    encgap_nc2 = 879.8 - (float)pulseIn(encpin2, LOW, 3000);

    /////////////////////////////엔코더 값 각도 변환
    if (encgap_nz1 < 400)
    {
        degree1 = constrain(360 * (encgap_nz1 - 4) / 873, 0, 360);
    }
    else
    {
        degree1 = constrain(360 * (encgap_nc1 - 4) / 873, 0, 360);
    }

    if (encgap_nz2 < 400)
    {
        degree2 = constrain(360 * (encgap_nz2 - 4) / 873, 0, 360);
    }
    else
    {
        degree2 = constrain(360 * (encgap_nc2 - 4) / 873, 0, 360);
    }
    /////////////////////////////엔코더 각도로 위치구하기
    getpos();
}

void getpos(void)
{ // 엔코더 각도가 536번 변하면 ( 360' * 엔코더-커플링 기어비 ) = 커플링 1회전

    enccount1 = enccount1 > 359 ? 0 : enccount1;
    enccount1 += (abs(prev_degree1 - degree1) > 0 ? 1 : 0);

    //  wheelpos1 = float(enccount1)*(PI/180);  

    enccount2 = enccount2 > 359 ? 0 : enccount2;
    enccount2 += (abs(prev_degree2 - degree2) > 0 ? 1 : 0);

    //  wheelpos2 = float(enccount2)*(PI/180);

    pid_control();

    prev_degree1 = degree1;
    prev_degree2 = degree2;
}

//////////////////////////////////////////////pid func
int pid_control(void)
{

    deg_int1 = prev_degree1 - degree1; // 각도 차이
    deg_int2 = prev_degree2 - degree2;

    enc2cpl = deg_int1 * (360 / 242); //엔코더 회전각 > 커플링 회전각
    enc2cp2 = deg_int2 * (360 / 242);

    distance1 = 130 * deg_int1; // l=r*theta
    distance2 = 130 * deg_int2;

    curt_time = millis();
    timeint = (double)(curt_time - prev_time);

    angle_velo1 = abs(distance1 / timeint);
    angle_velo2 = abs(distance2 / timeint);

    error[0] = (veloref - angle_velo1);
    error[1] = (veloref - angle_velo2);

#ifdef serialcheck
    Serial.print(" |timeint : ");
    Serial.print(timeint);
#endif

    // u[0] = prev_u[0] + error[0] * (Kp + timeint * Ki) - Kp * prev_error[0];
    // u[1] = prev_u[1] + error[1] * (Kp + timeint * Ki) - Kp * prev_error[1];

    // pid 0
    Pterm[0] = Kp * error[0];
    Iterm[0] += Ki * error[0] * timeint;
    Dterm[0] = Kd * (error[0] - prev_error[0]) / timeint;
    u[0] = Pterm[0] + Iterm[0] + Dterm[0];

    // pid 1
    Pterm[1] = Kp * error[1];
    Iterm[1] += Ki * error[1] * timeint;
    Dterm[1] = Kd * (error[1] - prev_error[1]) / timeint;
    u[1] = Pterm[1] + Iterm[1] + Dterm[1];

    u[0] = abs(u[0]) > 100000 ? 0 : u[0];
    u[1] = abs(u[1]) > 100000 ? 0 : u[1];

    prev_u[0] = u[0];
    prev_u[1] = u[1];

#ifdef serialcheck
    Serial.print(" |unit0 : ");
    Serial.print(u[0]);
    Serial.print(" |unit1 : ");
    Serial.print(u[1]);
#endif

    prev_error[0] = error[0];
    prev_error[1] = error[1];
    prev_time = curt_time;
}

////////////////////////////////////////////////////////////motor func
int motor_run(void)
{

    motor_speed[0] += u[0];
    motor_speed[1] += u[1];

    motor_speed[1] = motor_speed[1] > max_speed ? max_speed : motor_speed[1];
    motor_speed[1] = motor_speed[1] < min_speed ? min_speed : motor_speed[1];
    motor_speed[0] = motor_speed[0] > max_speed ? max_speed : motor_speed[0];
    motor_speed[0] = motor_speed[0] < min_speed ? min_speed : motor_speed[0];
#ifdef serialcheck
    Serial.print(" |motor_s R ");
    Serial.print(motor_speed[0]);
    Serial.print(" |motor_s L ");
    Serial.println(motor_speed[1]);
#endif
                                      // motor forward
        analogWrite(Motor_L, motor_speed[0]); 
        digitalWrite(ML_Dir, HIGH);           
        analogWrite(Motor_R, motor_speed[1]); 
        digitalWrite(MR_Dir, LOW);          

}

void motor_stop(void)
{
     analogWrite(Motor_L, 0); // PWM, Duty
     analogWrite(Motor_R, 0); // PWM, Duty
}
//////////////////////////////////////////////////cylinder

void Cylinder_stop(void){
       analogWrite(Cylinder, 0);   // PWM, Duty
       digitalWrite(Cyl_Dir, LOW); // Direction, Pull
}

void Roadangle(void)
{
    
    target_angle = Serial2.parseInt(); //각도값 저장(int형)
    Serial.print("target_angle : ");
    Serial.println(target_angle);

    target_resistance_L = Straight + round(abs(target_angle) * 3.64); //목표 좌회전 저항값 계산
    target_resistance_R = Straight - round(abs(target_angle) * 4.10); //목표 우회전 저항값 계산

    Serial.print("target_resistance_L : ");
    Serial.print(target_resistance_L);
    Serial.print(" | target_resistance_R : ");
    Serial.println(target_resistance_R);
    
    //직진일 때 저항값 + (목표 각도값*3.64 or 4.10)을 반올림한 값  = 좌/우회전 저항값 ( + : 좌 / - : 우 )

    // Straight
  if((Min_target_angle < target_angle) && (target_angle < Max_target_angle)&&(target_angle != 0)){
    
    if ((-1 <= target_angle) && (target_angle <= 1))
    {
        Serial.println("Straight");
        angle_straight();
       Serial.println("Straight Done.");
    }

    else if ( 1 < target_angle )
    { //input angle == positive -> left turn -> Pull
        Serial.println("Left");
        angle_left();
        Serial.println("Lefting Done.");
    }

    else if ( target_angle < -1)
    { //input angle == negative -> right turn -> Push
        Serial.println("Right");
        angle_right();
        Serial.println("Righting Done.");
    }
  }
   else {       
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
        }
}

void angle_straight(void)
{
    if (analogRead(Poten) > Straight)
    { //wheel state : left turn

        Serial.println("if_1_OK");

        while ((analogRead(Poten) != Straight + 1) && (analogRead(Poten) != Straight - 1) && (analogRead(Poten) != Straight))
        {
            analogWrite(Cylinder, cylDuty[1]); // PWM, Duty
            digitalWrite(Cyl_Dir, HIGH);       // Direction, Push
            Serial.print("while_1_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);   // PWM, Duty
        digitalWrite(Cyl_Dir, LOW); // Direction, Pull
    }

    else if (analogRead(Poten) < Straight)
    { //wheel state : right turn

        while ((analogRead(Poten) != Straight + 1) && (analogRead(Poten) != Straight - 1) && (analogRead(Poten) != Straight))
        {
            analogWrite(Cylinder, cylDuty[0]); // PWM, Duty
            digitalWrite(Cyl_Dir, LOW);        // Direction, Pull
            Serial.print("while_2_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }

    if (analogRead(Poten) == Straight)
    {
        Serial.println("if_2_OK");
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }
}

void angle_left(void)
{ // Left
    if (analogRead(Poten) > target_resistance_L)
    {

        Serial.println("if_3_OK");

        while ((analogRead(Poten) != target_resistance_L - 1) && (analogRead(Poten) != target_resistance_L + 1) && (analogRead(Poten) != target_resistance_L))
        {
            analogWrite(Cylinder, cylDuty[1]); // PWM, Duty
            digitalWrite(Cyl_Dir, HIGH);       // Direction, Push
            Serial.print("while_3_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);   // PWM, Duty
        digitalWrite(Cyl_Dir, LOW); // Direction, Pull
    }

    else if (analogRead(Poten) < target_resistance_L)
    {

        while ((analogRead(Poten) != target_resistance_L - 1) && (analogRead(Poten) != target_resistance_L + 1) && (analogRead(Poten) != target_resistance_L))
        {
            analogWrite(Cylinder, cylDuty[0]); // PWM, Duty
            digitalWrite(Cyl_Dir, LOW);        // Direction, Pull
            Serial.print("while_4_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }

    if (analogRead(Poten) == target_resistance_L)
    {
        Serial.println("if_4_OK");
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }
}

void angle_right(void)
{
    if (analogRead(Poten) > target_resistance_R)
    {

        Serial.println("if_5_OK");

        while ((analogRead(Poten) != target_resistance_R - 1) && (analogRead(Poten) != target_resistance_R + 1) && (analogRead(Poten) != target_resistance_R))
        {
            analogWrite(Cylinder, cylDuty[1]); // PWM, Duty
            digitalWrite(Cyl_Dir, HIGH);       // Direction, Push
            Serial.print("while_5_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);   // PWM, Duty
        digitalWrite(Cyl_Dir, LOW); // Direction, Pull
    }

    else if (analogRead(Poten) < target_resistance_R)
    {

        while ((analogRead(Poten) != target_resistance_R - 1) && (analogRead(Poten) != target_resistance_R + 1) && (analogRead(Poten) != target_resistance_R))
        {
            analogWrite(Cylinder, cylDuty[0]); // PWM, Duty
            digitalWrite(Cyl_Dir, LOW);        // Direction, Pull
            Serial.print("while_6_OK");
            Serial.print(" poten_val : ");
            Serial.println(analogRead(Poten));
        }
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }

    if (analogRead(Poten) == target_resistance_R)
    {
        Serial.println("if_6_OK");
        analogWrite(Cylinder, 0);    // PWM, Duty
        digitalWrite(Cyl_Dir, HIGH); // Direction, Push
    }
}

void ReadTF(void)
{
    myservo.write(0);

    TF_angle_T_str = TF_data.substring(0,2);
    TF_angle_T = TF_angle_T_str.toInt();
    TF_length_T_str = TF_data.substring(2,5);
    TF_length_T = TF_length_T_str.toInt();

    TF_angle_B_str = TF_data.substring(5,7);
    TF_angle_B = TF_angle_B_str.toInt();
    TF_length_B_str = TF_data.substring(7,10);
    TF_length_B = TF_length_B_str.toInt();

    Serial.print(" | TF_angle_T : ");
    Serial.print(TF_angle_T);
    Serial.print(" | TF_length_T : ");
    Serial.print(TF_length_T);
    
    Serial.print(" | TF_angle_B : ");
    Serial.print(TF_angle_B);
    Serial.print(" | TF_length_B : ");
    Serial.println(TF_length_B);

    // T,B angle calculate//
    Botangle_calc();
    Topangle_calc();
    
    waterpump();
}

void Botangle_calc(void)
{
    TF_angle_rad_B = TF_angle_B * (PI / 180);                                                    //alpha to double
    angle_rad_B = atan(((sin(TF_angle_rad_B) * TF_length_B) + Servo_TF_Between) / Servo_height); //rad theta -> double to rad
    angle_deg_B = (angle_rad_B * (180 / PI));                                                    //arctan return value -> rad to deg
    target_angle_Servo_B = 90 - round(angle_deg_B);                                              //angle value to int

    Servo_PWM_B = map(target_angle_Servo_B, 0, 90, Standard, 102);

    Serial.print("target_angle_Servo_B : ");
    Serial.print(target_angle_Servo_B);
    Serial.print(" | Servo_PWM_B : ");
    Serial.print(Servo_PWM_B);
}

void Topangle_calc(void)
{
    TF_angle_rad_T = TF_angle_T * (PI / 180);                                                    //alpha to double
    angle_rad_T = atan(((sin(TF_angle_rad_T) * TF_length_T) + Servo_TF_Between) / Servo_height); //rad theta -> double to rad
    angle_deg_T = (angle_rad_T * (180 / PI));                                                    //arctan return value -> rad to deg
    target_angle_Servo_T = 90 - round(angle_deg_T);                                              //angle value to int

    Servo_PWM_T = map(target_angle_Servo_T, 0, 90, Standard, 102);

    Serial.print(" | target_angle_Servo_T : ");
    Serial.print(target_angle_Servo_T);
    Serial.print(" | Servo_PWM_T : ");
    Serial.println(Servo_PWM_T);
}

void waterpump(void)
{

    myservo.write(Servo_PWM_B);
    analogWrite(WaterPump, WaterPump_Duty);
    Serial.println("Water_Start");

    spray();

    analogWrite(WaterPump, 0);
    Serial.println("Water_End");
    myservo.write(Standard);

    Check = 0;
}

int spray(void)
{
    int waterloadtime = 2000;
    int spraycount = 2;

    delay(waterloadtime);
    for (int i = 0; i < spraycount; i++)
    {
        for (int angle = Servo_PWM_B; angle > Servo_PWM_T; angle--)
        {
            myservo.write(angle);
            delay(50);
        }

        for (angle = Servo_PWM_T; angle < Servo_PWM_B; angle++)
        {
            myservo.write(angle);
            delay(50);
        }
    }
    delay(1000);
}
