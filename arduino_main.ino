/*
이 코드는 초음파센서/PID 제어기가 적용된 최종 버전이 아닙니다. 
해당 코드는 분실되어 초음파 센서 없이 P제어기만을 사용하는 코드입니다. 
*/

#define FRONT LOW
#define BACK HIGH
#define MOTORMIN 0
#define MOTORMAX 255
#define RIGHT_CORR 1.3
#define LEFT_CORR 1

#define DESIRED_RANGE 100
#define DESIRED_ANGLE 0
#define DISTANCE_MARGIN 5
#define LATERAL_MARGIN 5
#define LATERAL_MAX 70

typedef struct {
	float raw;
	float pre_filtered;
	float cur_filtered;
} SIGNAL;

typedef struct {
	int trig;
	int echo;
	SIGNAL signals;
} SONAR;

typedef struct {
	int dir;
	int pwm;
	int dirVal;
	int runVal;
} Motor;

typedef struct {
	float cur;
	float pre;
	float differential;
	float integral;
} ERROR_VAL;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PID_GAIN;

Motor left = {8, 9, HIGH, 0};
Motor right = {10, 11, HIGH, 0};

ERROR_VAL rangeError, angleError;
PID_GAIN r = {3, 0, 0};
PID_GAIN a = {1.5, 0, 0};


String state;
void setup(){
	pinMode(left.dir, OUTPUT);
	pinMode(left.pwm, OUTPUT);
	pinMode(right.dir, OUTPUT);
	pinMode(right.pwm, OUTPUT);
	Serial.begin(9600);
	Serial.setTimeout(10);
}

float string2double(String & str){
	return atof(str.c_str());
}

float LPF(SIGNAL* signals, float tau){
	signals->cur_filtered = tau*signals->raw+(1-tau)*signals->pre_filtered;
	signals->pre_filtered = signals->cur_filtered;
	return signals->cur_filtered;
}

float distance(SONAR* sensor, int isLPF){
	digitalWrite(sensor->trig, LOW);
	digitalWrite(sensor->echo, LOW);
	delayMicroseconds(2);
	digitalWrite(sensor->trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(sensor->trig, LOW);
	unsigned long duration = pulseIn(sensor->echo, HIGH, 2000);
	sensor->signals.raw = ((float)(duration*340)/10000)/2;
	if(isLPF){
		return LPF(&(sensor->signals), 0.35);
	}
	else return sensor->signals.raw;
}

float DistanceGap(float desired, float range){
	if(abs(desired-range) <= DISTANCE_MARGIN) return 0;
	else return range-desired;
}

float LateralGap(float desired, float range){
	if(abs(desired-range) <= LATERAL_MARGIN) return 0;
	else {
    float tmpVal = 0;    	
    	if(range > 0) tmpVal = (range-LATERAL_MARGIN)-desired;
    	else tmpVal = (range+LATERAL_MARGIN)-desired;
		return constrain(tmpVal, -LATERAL_MAX, LATERAL_MAX);
	}
}

float CtrlVal(ERROR_VAL err, PID_GAIN pid){
	return pid.Kp*err.cur+pid.Ki*err.integral+pid.Kd*err.differential;
}

void MotorRun(int leftVal, int rightVal){
  if(leftVal >= 0) left.dirVal = FRONT;
  else left.dirVal = BACK;
  if(rightVal >= 0) right.dirVal = FRONT;
  else right.dirVal= BACK;

  left.runVal = constrain(abs(LEFT_CORR*leftVal), MOTORMIN, MOTORMAX);
  right.runVal = constrain(abs(RIGHT_CORR*rightVal), MOTORMIN, MOTORMAX);
  
  digitalWrite(left.dir, left.dirVal);
  digitalWrite(right.dir, right.dirVal);
  analogWrite(left.pwm, left.runVal);
  analogWrite(right.pwm, right.runVal); 
  return;
}

float found, range, angle;
float straightVal = 0, rotateVal = 0;

void loop(){	

	if(Serial.available()){
		String msg = Serial.readString();
    int idxSep, jdxSep;
    idxSep = msg.indexOf(',');
    jdxSep = msg.indexOf(',', idxSep+1);
		if(idxSep != -1 && jdxSep != -1){
      String msg_found = msg.substring(0, idxSep);
      String msg_dist= msg.substring(idxSep+1, jdxSep);
      String msg_angle = msg.substring(jdxSep+1);
      found = string2double(msg_found);
      range = string2double(msg_dist);
      range = (found==1)?range:DESIRED_RANGE;
      angle = string2double(msg_angle);
      angle = (found==1)?angle:DESIRED_ANGLE;

      Serial.println("Found : "+String(found)+"\tRange : "+String(range)+"\tAngle : "+String(angle));
		}
		angleError.cur = LateralGap(DESIRED_ANGLE, angle);
		rangeError.cur = DistanceGap(DESIRED_RANGE, range);
		straightVal = CtrlVal(rangeError, r);
		rotateVal = CtrlVal(angleError, a);
    Serial.println("Straight : "+String(straightVal)+"\tRotate : "+String(rotateVal));
	}

  MotorRun(straightVal+rotateVal, straightVal-rotateVal);
	delay(1);	
  	
}
