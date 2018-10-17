#include <SPI.h>
#include "df_can.h"
#include <Wire.h> 
/*** constant parameter ***/
#define Break_Threshold -5   //Break Ampier threshold value 
#define STEERING_VR_TOLERANCE 5


/*** Arduino pin assignment ***/
/// out ///
#define PUSHER_BRAKE_RELAY_1 47   //47
#define PUSHER_BRAKE_RELAY_2 49   //49
#define PUSHER_BRAKE_RELAY_3 51  //51
#define PUSHER_BRAKE_RELAY_4 53  //53
#define WALKING_FORWARD_RELAY 22
#define WALKING_BACKWARD_RELAY 24
#define SpeedKeyin 9
#define BRAKE_THROTTLE_VR NaN
#define STEERING_CW_PWM 5 // (coonnect to PWM1)
#define STEERING_CCW_PWM 6 // (coonnect to PWM2)
#define STEERING_PWM_ENABLE 7
/// in ///
#define PUSHER_BRAKE_AMPERE A0
#define STEERING_VR A1
#define SPEED_METER_MAGNET A2
#define VR_ZERO 500
/// light control ///
#define Break_light_RELAY 28
#define Left_Turn_light_RELAY 30
#define Right_Turn_light_RELAY 32
#define Front_main_light_RELAY 34
const int SPI_CS_PIN = 10;
MCPCAN CAN(SPI_CS_PIN);// Set CS pin
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char ID = 0;
unsigned char buf[8];
char str[20];
int r1_data = 0;
int r2_data = 0; 
double linearV = 0;
double angularV = 0;

static int User_stop = 0;
/*** 方向盤PWM控制方式 ***//*
EN=0  PWML1=PWM PWML2=1   電機正轉（改變佔空比就可調速）
EN=0  PWML1=1   PWML2=PWM 電機反轉（改變佔空比就可調速）
EN=0  PWML1=1   PWML2=1   電機剎車
EN=1  PWML1=1   PWML2=1   電機自由 
*/
static int test_variable = 0;



/*** current status ***/
static int pusher_brake_current_status = 0;  // 0:up  1:down  2:uping  3:downing
static int walking_dir_current_status = 0;  // 0:off  1:forward  2:backward


static int User_command = 0;
static int Previous_Command = 0;  // only used in brake
//////////////////////////////////////////////////Steering//////////////////////////////////////////////////
double Voltage = 0;
double ampere = 0;
//////////////////////////////////////////////////Travel Motor//////////////////////////////////////////////////
static int Privious_Direction= 0;
static int Now_Direction= 0;
static int Speed = 0;
//////////////////////////////////////////////////Brake//////////////////////////////////////////////////
static int pusher_brake_objective_status = 0;  // 0:up  1:down
static int walking_dir_objective_status = 0;  // 0:off  1:forward  2:backward
static int a = 0;
static int previous_a = 0;
//////////////////////////////////////////////////PID sterring//////////////////////////////////////////////////
int VRvalue;
int setVRpoint;//I am setting it to move through 100 degrees
double Kp = 7;// 0.32;  you can set these constants however you like depending on trial & error
double Ki = 0.0;  //0.1;
double Kd = 0.0; // 0.3;

double angle = 0;

int last_error = 0;
int error = 0;
int changeError = 0;
int totalError = 0;
double pidTerm = 0;
double pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|


int VR_uplimit;
int VR_downlimit;
int VRzero;

unsigned long start_time;
unsigned long loop_time;

double wheelRPM;
double throttlePercent;
double wheelAngle;
double sterringAngle;

void PIDcalculation(){
	angle = (2.0 - 1.0*2 + 1.0 * VRvalue);//count to angle conversion
	error = setVRpoint - angle;

	changeError = error - last_error; // derivative term
	totalError += error; //accumalate errors to find integral term
	pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
	pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
	pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

	last_error = error;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//Travel Motor user setup_speed check(0-255)(0-20km/hr)
void speed_check(){
	if(Speed>255)
	Speed=255;
	else if(Speed<0)
	Speed=0;
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//Travel Motor go forward/backward function
void DC_direction(){
	if(Privious_Direction==Now_Direction){
		if(Now_Direction==1)
		{
			digitalWrite(WALKING_FORWARD_RELAY, HIGH);
			digitalWrite(WALKING_BACKWARD_RELAY, LOW);
			Serial.println("BACKWARD");
		}
		else if(Now_Direction==2)
		{
			digitalWrite(WALKING_FORWARD_RELAY, LOW);
			digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
			Serial.println("FORWARD");   
		}
		else if(Now_Direction==3)
		{
			digitalWrite(WALKING_FORWARD_RELAY, HIGH);
			digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
			Serial.println("Direction offline");
		}
		else{}
	}
	else
	{  Serial.println("===============================================Speed return to zero===============================================");
		while(Speed>0){
			Speed-=60;
			delay(500);
			speed_check();
			Serial.print(Speed,DEC);
			Serial.print(Speed,DEC);
			analogWrite(SpeedKeyin, Speed);  // LED �@�b�G��(0-255)
		}
		digitalWrite(WALKING_FORWARD_RELAY, HIGH);
		digitalWrite(WALKING_BACKWARD_RELAY, HIGH);
	}

}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/*** updating current status ***/
/*
void update_ampere_sensor(){
for(int i = 0; i < 500; i++) {
	Voltage = (Voltage + (.0049 * analogRead(A0))); // (5 V / 1024 (Analog) = 0.0049) which converter Measured analog input voltage to 5 V Range
}
Voltage = Voltage /500;
ampere = (Voltage -2.5)/ 0.185;

if(a==0 || previous_a==0)
{  
	Serial.print("Current(A) = ");
	Serial.println(ampere,2);
	Serial.println("Starting Current");
	ampere =-0.1;   

}
else
{
	Serial.print("Current(A) = ");
	Serial.println(ampere,2);
	Serial.print("VRvalue is ");
	Serial.print(VRvalue);
	Serial.print("  setVRpoint is ");
	Serial.print(setVRpoint);
	Serial.print("  pidTerm is ");
	Serial.println(pidTerm);

}

}
*/


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

/*** make it to objective status***/
/*void pusher_brake_make_objective(){
if (pusher_brake_objective_status == 5){  // objective is 0 (up)
	digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_1, LOW);
	digitalWrite(PUSHER_BRAKE_RELAY_4, LOW);
	pusher_brake_objective_status=5;
	Serial.println("state : Rising");    
}
else if (pusher_brake_objective_status == 6){  // objective is 1 (down)
	digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_2, LOW);
	digitalWrite(PUSHER_BRAKE_RELAY_3, LOW);
	pusher_brake_objective_status=6;
	Serial.println("state : Going down");
}
else{}


// update_ampere_sensor();

if(User_command == 'a' && ampere > 0){
	pusher_brake_current_status = 7;    // 0:immidiantly stop
	digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH);
	pusher_brake_objective_status=8;
	Serial.println("  state : Upward");

}
else if((User_command == 's' && ampere < -1)||User_command == 's' && ampere >0){
	pusher_brake_current_status = 7;    // 0:immidiantly stop
	digitalWrite(PUSHER_BRAKE_RELAY_1, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_2, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_3, HIGH);
	digitalWrite(PUSHER_BRAKE_RELAY_4, HIGH);
	pusher_brake_objective_status=8;
	Serial.println("  state : Downward");

}
else 
{
	Serial.println("");
}
}
*/
#define Break_light_RELAY 28
#define Left_Turn_light_RELAY 30
#define Right_Turn_light_RELAY 32
#define Front_main_light_RELAY 34




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
	wheelRPM = linearV * 6 * 60 / (2 * 3.141592 * 0.254);
	throttlePercent = 100 * wheelRPM / 3000;
	Speed = 255 * wheelRPM / 3000;
	Speed = constrain(Speed, 0, 255);//constraining to appropriate value
	/*
W(rad/sec)角速度 = 線速度 * sin(輪胎角rad) ÷ 半軸距0.525 (m)
輪胎角rad = arcsin_rad[ W(rad/sec)角速度 * 半軸距0.525 (m) ÷ 線速度  ]
右一半 254
84
右一零 338
171
正中間 509
171
左一零 680
83
左一半 763
82
左二零 845

170 VR = 1 圈方向盤 = 15度輪胎角 = 0.261799 rad輪胎角
VR = zero-offset + 170 * 輪胎角rad ÷ 0.261799
*/
	if(linearV != 0)
	{	
		wheelAngle = asin( angularV * 0.525 / linearV );
		sterringAngle = 360 * wheelAngle / 0.261799;
		setVRpoint = VRzero + 170 * wheelAngle / 0.261799;
		setVRpoint = constrain(setVRpoint, VR_downlimit, VR_uplimit);//constraining to appropriate value
		
	}
	else
	{
		setVRpoint = VRzero;
	}
}  // end go




/*
//https://en.wikipedia.org/wiki/Taylor_series#Trigonometric_functions
double arcsin_rad(double c)
{
double out;
out= (c
+(c*c*c)/6
+(3*c*c*c*c*c)/40
+(5*c*c*c*c*c*c*c)/112
+(35*c*c*c*c*c*c*c*c*c)/1152 
+(c*c*c*c*c*c*c*c*c*c*c*0.022)
+(c*c*c*c*c*c*c*c*c*c*c*c*c*.0173)
+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*.0139)
+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.0115)
+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.01)
);


if(c>=.96 && c<.97)
{out=1.287+(3.82*(c-.96)); }
if(c>=.97 && c<.98)
{out=(1.325+4.5*(c-.97));} // arcsin
if(c>=.98 && c<.99)
{out=(1.37+6*(c-.98));}
if(c>=.99 && c<=1)
{out=(1.43+14*(c-.99));}

return out;}
*/







///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/*** Arduino start ***/
void setup() {
	Serial.begin(115200);   
	pinMode(SpeedKeyin, OUTPUT); // �]�w pin 3 ����X
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
	int count = 50;                                     // the max numbers of initializint the CAN-BUS, if initialize failed first!.  
	do {
		CAN.init();   //must initialize the Can interface here! 
		if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
		{
			Serial.println("DFROBOT's CAN BUS Shield init ok!");
			break;
		}
		else
		{
			Serial.println("DFROBOT's CAN BUS Shield init fail");
			Serial.println("Please Init CAN BUS Shield again");

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
	
	
	Previous_Command = User_command;

	if(flagRecv) 
	{                                   // check if get data

		flagRecv = 0;                   // clear flag

		while (CAN_MSGAVAIL == CAN.checkReceive()) 
		{
			// read data,  len: data length, buf: data buf
			CAN.readMsgBuf(&len, buf); 
			ID=CAN.getCanId();
			//  Serial.println(ID);
			// print the data
			/*
			Serial.print("Data Length: "); Serial.print(len); Serial.print("  ID: "); Serial.println(ID);
			Serial.print("CAN Receive Data: ");
			for(int i = 0; i<len; i++)
			{
				Serial.print(buf[i]); Serial.print(" ");
			}
			Serial.println("");
	*/
			if (ID==10)
			{
				r1_data = (buf[0] << 8) | buf[1];
				linearV = r1_data/100.0; 
			}
			
			else if (ID==11)
			{
				r2_data = (buf[0] << 8) | buf[1];
				angularV = r2_data/100.0;
			}

			else
			{
				Serial.println("////////////////Error: Unexpect Input////////////////////////////////");
			}   
			//Serial.print("r_data: "); Serial.println(r_data);  
			
		} // end while (CAN_MSGAVAIL == CAN.checkReceive())
	} // end if(flagRecv)

	if (Serial.available() > 0) 
	{
		User_stop = Serial.read();
		if(User_stop==' ')
		{ linearV=0;} 
		else if (User_stop=='q')
		{linearV=2; }
		else if (User_stop=='w')
		{linearV=3; }
		else if (User_stop=='e')
		{linearV=4; }
		else if (User_stop=='r')
		{linearV=5; }
		else if (User_stop=='u')
		{linearV=6; }
		else if (User_stop=='i')
		{linearV=7; } 
		else if (User_stop=='o')
		{linearV=8; }    
		else if (User_stop=='p')
		{linearV=9; }       
	}
	go(linearV, angularV);
	delay(100);
	/////////////////////////////////////////////////////////////////////////////
	VRvalue = analogRead(STEERING_VR);
	PIDcalculation();// find PID value
	if (angle < setVRpoint) {
		analogWrite(STEERING_CW_PWM, 0);
		analogWrite(STEERING_CCW_PWM, pidTerm_scaled);
	}
	else
	{
		analogWrite(STEERING_CCW_PWM, 0);
		analogWrite(STEERING_CW_PWM, pidTerm_scaled);
	}

	/////////////////////////////////////////////////////////////////////////////
	analogWrite(SpeedKeyin, Speed);
	
	
	DC_direction();

	// if(pusher_brake_objective_status == 8)  
	// {Serial.println("Status :Brake Standby");}
	// else{}

	///////////////////////////////////////////////////////////////////////////////////////////

	previous_a = a;
	if(Previous_Command==User_command) 
	{
		a=1;  
	}
	else
	{
		a=0;
		Serial.println("//////////////Status : Changing//////////////////");
	}

	
	
	
	Serial.println("");
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

	loop_time = millis() - start_time;

	Serial.print("loop time(ms): "); Serial.println(loop_time);
	Serial.print("\n\n");

}  // end loop
