#include <SPI.h>
#include "df_can.h"
#include <Wire.h> 

/*** constant parameter ***/

static int User_stop = 0;

/*** Arduino pin assignment ***/
/// out ///
#define PUSHER_BRAKE_RELAY_1 37   
#define PUSHER_BRAKE_RELAY_2 35   
#define PUSHER_BRAKE_RELAY_3 33  
#define PUSHER_BRAKE_RELAY_4 31  
#define WALKING_FORWARD_RELAY 32
#define WALKING_BACKWARD_RELAY 34
#define Break_light_RELAY 41
#define Left_Turn_light_RELAY 45
#define Right_Turn_light_RELAY 43
#define Front_main_light_RELAY 39

#define BRAKE_THROTTLE_VR NaN
#define STEERING_CW_PWM 5 // (coonnect to PWM1)
#define STEERING_CCW_PWM 6 // (coonnect to PWM2)
#define STEERING_PWM_ENABLE 7
#define SpeedKeyin 9            //speed output to curtis 1234
#define STEERING_VR A5          //potentiometer Level input port


/*** Parameters and Variables in CANBUS ***/
const int SPI_CS_PIN = 10;
MCPCAN CAN(SPI_CS_PIN);// Set CS pin
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char ID = 0;
unsigned char buf[8];
int r1_data = 0;
int r2_data = 0; 
double linearV = 0;
double angularV = 0;
//////////////////////////////////////////////////current status////////////////////////////////////////////
static int pusher_brake_current_status = 0;  // 0:up  1:down  2:uping  3:downing
static int walking_dir_current_status = 0;  // 0:off  1:forward  2:backward

static int User_command = 0;
static int Previous_Command = 0;  // only used in brake
//////////////////////////////////////////////////Steering//////////////////////////////////////////////////


//////////////////////////////////////////////////Travel Motor//////////////////////////////////////////////
static int Privious_Direction= 0;
static int Now_Direction= 0;
static int Speed = 0;
static int Previous_Speed = 0;
//////////////////////////////////////////////////Brake////////////////////////////////////////////////////
static int pusher_brake_objective_status = 0;  // 0:up  1:down
static int walking_dir_objective_status = 0;  // 0:off  1:forward  2:backward
static int a = 0;
static int previous_a = 0;

/*** variables in sterringPID() ***/
const double PID_tolerance = 5;
const double Kp = 9.0;
const double Ki = 0.0;
const double Kd = 0.0;
int VRvalue;
int setVRpoint;
int last_error = 0;
int error = 0;
int changeError = 0;
int totalError = 0;
int pidTerm = 0;
int VR_uplimit;
int VR_downlimit;
int VRzero;
int TotalSteeringVR=0;

/*** variables for time counting ***/
unsigned long start_time;
unsigned long loop_time;
unsigned long current_time;
unsigned long mark_time = 0;
unsigned long cross_time;

/*** variables for go() ***/
double wheelRPM;
double throttlePercent;
double wheelAngle;
double sterringAngle;

void sterringPID(){  
  //---------------------------------------- 20181014, VRvalue is accumulated average of recent 4 points
  //VRvalue = analogRead(STEERING_VR);
  
  int i;
  i = (TotalSteeringVR+2)>>2;     // i=TotalSteeringVR/4, rounded
  TotalSteeringVR -= i;           // TotalSteeringVR=TotalSteeringVR*(3/4)
  i = analogRead(STEERING_VR);
  TotalSteeringVR += i;          // TotalSteeringVR=(3/4)*TotalSteeringVR+analogRead(STEERING_VR)
  VRvalue=(TotalSteeringVR+2)>>2;   //VRvalue=TotalSteeringVR/4, rounded

  
	error = setVRpoint - VRvalue;
  //Serial.print("PIDerror: "); Serial.println(error); 
	if(error > PID_tolerance || error < -1*PID_tolerance)
	{
		changeError = error - last_error; // derivative term
		totalError += error; //accumalate errors to find integral term
		pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
		pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
		if (pidTerm > 0)
		{
			analogWrite(STEERING_CCW_PWM, 0);
			analogWrite(STEERING_CW_PWM, pidTerm);
		}
		else
		{
			analogWrite(STEERING_CW_PWM, 0);
			analogWrite(STEERING_CCW_PWM, -1*pidTerm);
		}
	}
  else
  {
    analogWrite(STEERING_CCW_PWM, 0);
    analogWrite(STEERING_CW_PWM, 0);
  }
	last_error = error;
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//Travel Motor go forward/backward function
void CAR_direction(){

	if(linearV>0){Now_Direction=1;}
	else if(linearV<0){Now_Direction=2;}
	else if(linearV==0){Now_Direction=3;}

	if(Privious_Direction==Now_Direction)
	{
		if(Now_Direction==1)
		{
			digitalWrite(WALKING_FORWARD_RELAY, HIGH);
			digitalWrite(WALKING_BACKWARD_RELAY, LOW);
			//Serial.println("BACKWARD");
		}
		else if(Now_Direction==2)
		{
			digitalWrite(WALKING_FORWARD_RELAY, LOW);
			digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
			//Serial.println("FORWARD");   
		}
		else if(Now_Direction==3)
		{
			digitalWrite(WALKING_FORWARD_RELAY, HIGH);
			digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
		//	Serial.println("Direction offline");
		}
		else{}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

/*** make it to objective status***/
void pusher_brake_make_objective(){
if (pusher_brake_objective_status == 1){  // objective is 0 (up)
digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_1, LOW);
digitalWrite(PUSHER_BRAKE_RELAY_4, LOW);
//pusher_brake_objective_status=5;
Serial.println("state : Rising");    
}
else if (pusher_brake_objective_status == 2){  // objective is 1 (down)
digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_2, LOW);
digitalWrite(PUSHER_BRAKE_RELAY_3, LOW);
//pusher_brake_objective_status=6;
Serial.println("state : Going down");
}
else{}


if(pusher_brake_objective_status == 3){
pusher_brake_current_status = 7;    // 0:immidiantly stop
digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH);
//pusher_brake_objective_status=8;
Serial.println("  state : brake off");

}

else 
{
Serial.println("");
}
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void go(double linearV, double angularV)
{
	/*
行走馬達最高轉速 7000 rpm                                     
行走馬達減速比 6:1                                            
輪胎半徑 10 吋 = 25.4 cm                                     
V(m/sec) = rpm * 2 * pi * 0.254 (m) ÷ 6 ÷ 60              
rpm = V(m/sec) * 6 * 60 ÷ (2 * pi * 0.254 (m))            
0~3000rpm map to 0~5V map to 0~255PWM                     
PWM = 255 * rpm ÷ 3000                                    
*/

///////////////////////////////////////////////////20181017
	wheelRPM = abs(linearV) * 1237.7;
///////////////////////////////////////////////////20181017

	throttlePercent = 100 * wheelRPM / 7000;
	Speed = 255 * wheelRPM / 7000;
	Speed = constrain(Speed, 0, 255);//constraining to appropriate value
	/*
W(rad/sec)角速度 = 線速度 * sin(輪胎角rad) ÷ 半軸距0.525 (m)
輪胎角rad = arcsin_rad[ W(rad/sec)角速度 * 半軸距0.525 (m) ÷ 線速度  ]
右一半 254
差84
右一零 338
差171
正中間 509
差171
左一零 680
差83
左一半 763
差82
左二零 845

170 VR = 1 圈方向盤 = 15度輪胎角 = 0.261799 rad輪胎角
VR = zero-offset + 170 * 輪胎角rad ÷ 0.261799
*/
	if(linearV != 0)
	{ 
		wheelAngle = asin( angularV * 0.525 / linearV );
		sterringAngle = 360 * wheelAngle / 0.261799;
		setVRpoint = VRzero - 170 * wheelAngle / 0.261799;
		setVRpoint = constrain(setVRpoint, VR_downlimit, VR_uplimit);//constraining to appropriate value
	}
	else
	{
		setVRpoint = VRzero;
	}
}  // end go


///////////////////////////////////////////////////////////////////////////////////////////
void Speedkeyin() 
{
  current_time = millis();
  cross_time = current_time - mark_time;  
  
///////////////////////////////////////////////////20181017
if(Speed>Previous_Speed && cross_time>20 )
{ 
  mark_time = millis();
  Previous_Speed = Previous_Speed+1;

  analogWrite(SpeedKeyin, Previous_Speed);
}
else if(Speed<Previous_Speed && cross_time>20 )
{ 
  mark_time = millis();
  Previous_Speed = Previous_Speed-1;

  analogWrite(SpeedKeyin, Previous_Speed);
}
  analogWrite(SpeedKeyin, Previous_Speed);
///////////////////////////////////////////////////20181017

}

///////////////////////////////////////////////////////////////////////////////////////////
/*** Arduino start ***/
void setup() {
	Serial.begin(115200);   
	pinMode(SpeedKeyin, OUTPUT); 
	pinMode(PUSHER_BRAKE_RELAY_1, OUTPUT);
	pinMode(PUSHER_BRAKE_RELAY_2, OUTPUT);
	pinMode(PUSHER_BRAKE_RELAY_3, OUTPUT);
	pinMode(PUSHER_BRAKE_RELAY_4, OUTPUT);  
	pinMode(WALKING_FORWARD_RELAY, OUTPUT);
	pinMode(WALKING_BACKWARD_RELAY, OUTPUT);
	pinMode(STEERING_PWM_ENABLE, OUTPUT);
	pinMode(Break_light_RELAY, OUTPUT);
	pinMode(Left_Turn_light_RELAY, OUTPUT);
	pinMode(Right_Turn_light_RELAY, OUTPUT);
	pinMode(Front_main_light_RELAY, OUTPUT);

	digitalWrite(WALKING_FORWARD_RELAY, HIGH);
	digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH); 
	digitalWrite(Break_light_RELAY, HIGH);
	digitalWrite(Left_Turn_light_RELAY, HIGH);
	digitalWrite(Right_Turn_light_RELAY, HIGH);
	digitalWrite(Front_main_light_RELAY, HIGH);    
	digitalWrite(STEERING_PWM_ENABLE, LOW);
	analogWrite(STEERING_CW_PWM, 0);
	analogWrite(STEERING_CCW_PWM, 0);
	delay(100);
	VRzero = analogRead(STEERING_VR);
	VR_uplimit = VRzero + 300;
	VR_downlimit = VRzero - 300;
	setVRpoint = VRzero;

	delay(100);
	Serial.println("Arduino is starting up!");
	int count = 50;    
	// the max numbers of initializint the CAN-BUS, if initialize failed first!.  


  
	do {
		CAN.init();   //must initialize the Can interface here! 
		if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
		{
			Serial.println("DFROBOT's CAN BUS Shield init ok!");
			break;
		}
		else
		{
			delay(100);
			if (count <= 1)
			Serial.println("Please give up trying!, trying is useless!");
		}
	}while(count--);
  
	attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt


}  

///////////////////////////////////////////////////////////////////////////////////////////
void MCP2515_ISR()
{
	flagRecv = 1;
}
///////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
	start_time = millis();
	Privious_Direction = Now_Direction;

	// pusher_brake_make_objective();
	if(flagRecv) 
	{                                   // check if get data
		flagRecv = 0;                   // clear flag
		while (CAN_MSGAVAIL == CAN.checkReceive()) 
		{
			CAN.readMsgBuf(&len, buf); 
			ID=CAN.getCanId();
			if (ID==10){
				r1_data = (buf[0] << 8) | buf[1];
				linearV = r1_data/100.0; 
			}
			else if (ID==11){
				r2_data = (buf[0] << 8) | buf[1];
				angularV = r2_data/100.0;
			}
			else{
				Serial.println("////////////////Error: Unexpect Input////////////////////////////////");
			}     
		} 
	} 


/* 
  if (Serial.available() > 0) 
  {
    User_stop = Serial.read();
    if(User_stop==' ')
    { linearV=0;} 
    else if (User_stop=='q')
    {linearV=2; }
    else if (User_stop=='w')
    {linearV=-2; }
    else if (User_stop=='z')
    {linearV=5; }
    else if (User_stop=='x')
    {linearV=-5; }
    else if (User_stop=='e')
    {digitalWrite(Left_Turn_light_RELAY,LOW);} 
    else if (User_stop=='r')
    {digitalWrite(Right_Turn_light_RELAY,LOW);}       
    else if (User_stop=='u')
    {pusher_brake_objective_status = 1;}
    else if (User_stop=='i')
    {pusher_brake_objective_status = 2;} 
    else if (User_stop=='o')
    {pusher_brake_objective_status = 3;}    
    else if (User_stop=='h')
    {digitalWrite(Front_main_light_RELAY,LOW);  }
    else if (User_stop=='j')
    {digitalWrite(Break_light_RELAY,LOW);}
    else if (User_stop=='k')
    {digitalWrite(Left_Turn_light_RELAY,LOW);} 
    else if (User_stop=='l')
    {digitalWrite(Right_Turn_light_RELAY,LOW);}       
    else if (User_stop=='m')
    { 
      digitalWrite(Break_light_RELAY, HIGH);
      digitalWrite(Left_Turn_light_RELAY, HIGH);
      digitalWrite(Right_Turn_light_RELAY, HIGH);
      digitalWrite(Front_main_light_RELAY, HIGH);     
      }
  }
*/

  
  pusher_brake_make_objective();
	CAR_direction();  
	go(linearV, angularV);
	sterringPID();
	Speedkeyin();
	


  
  Serial.print("linearV: ");
  Serial.print(linearV);
  Serial.print("  wheelRPM: ");
  Serial.print(wheelRPM);
  Serial.print("  throttlePercent: ");
  Serial.print(throttlePercent);
  Serial.print("  PWMSpeed: ");
  Serial.println(Speed,DEC);

  Serial.print("angularV: ");
  Serial.print(angularV);
  Serial.print("  wheelAngle: ");
  Serial.print(wheelAngle);
  Serial.print("  sterringAngle: ");
  Serial.print(sterringAngle);
  Serial.print("  setVRpoint: ");
  Serial.println(setVRpoint);

  Serial.print("VRvalue is "); Serial.print(VRvalue); 
  Serial.print("  pidTerm is "); Serial.println(pidTerm);

  Serial.print("r is "); Serial.print(VRvalue); 
  
   
  loop_time = millis() - start_time;

  Serial.print("loop time(ms): "); Serial.println(loop_time);
  Serial.print("\n\n\n\n\n\n");

  
  /*
  Serial.print(VRvalue);
  Serial.print("\t");
  Serial.print(600);
  Serial.print("\t");
  Serial.print(400);
  Serial.print("\t");
   */
   
}  // end loop
