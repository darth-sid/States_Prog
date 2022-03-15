#include "main.h"
#include <map>
//================================================================================================================
//   | _____|_|  |___  | |   |_____ | |   |_____ | |   ____     ____
//   | |_|___     ___| | |   _____| | |        / / /   \ \ \   / / /
//   |____ \ \   |___  | |   | _____|_|       / / /     \ \ \ / / /
//    ___| | |    ___| | |   | |_|__ _       / / /       \ \ / / /
//   |_____/_/   |_____| |   |______|_|     /_/_/         \___/_/
//================================================================================================================

//PORTS===========================================================================================================
#define DRIVELEFT1PORT 18//ok
#define DRIVELEFT2PORT 6//rev
#define DRIVELEFT3PORT 7//rev
#define DRIVELEFT4PORT 8//ok
#define DRIVERIGHT1PORT 10//rev
#define DRIVERIGHT2PORT 9//ok
#define DRIVERIGHT3PORT 3//ok
#define DRIVERIGHT4PORT 1//rev

#define LEFT_ROT_PORT 20
#define RIGHT_ROT_PORT 13
#define BACK_ROT_PORT 14

#define PNEUM_PORT_1 5
#define PNEUM_PORT_2 6

#define INERTIAL_PORT 12

#define ENC_TO_INCH (2.75*M_PI/36000)
#define DEG_TO_RAD M_PI/180

#define TRACKER_DISTANCE 3.375
#define TRACKER_DISTANCE_BACK 1

#define MAX_VEL 127.0
#define TURN_VEL_K 1
#define T 16
//UI==============================================================================================================
//buttons
lv_obj_t * autonButton;
lv_obj_t * autonAWPButton;
lv_obj_t * confirmationButton;
//labels
lv_obj_t * autonButtonLabel;
lv_obj_t * autonAWPButtonLabel;
lv_obj_t * confirmationLabel;
lv_obj_t * selectionLabel;
//styles
lv_style_t buttonStyle1REL;
lv_style_t buttonStyle1PR;
lv_style_t buttonStyle2REL;
lv_style_t buttonStyle2PR;
lv_style_t confirmButtonStyleREL;
lv_style_t confirmButtonStylePR;
//COMPONENETS=====================================================================================================
pros::Motor driveLeft1(DRIVELEFT1PORT , MOTOR_GEARSET_06, false);
pros::Motor driveLeft2(DRIVELEFT2PORT , MOTOR_GEARSET_06, true);
pros::Motor driveLeft3(DRIVELEFT3PORT , MOTOR_GEARSET_06,true);
pros::Motor driveLeft4(DRIVELEFT4PORT , MOTOR_GEARSET_06,false);
pros::Motor driveRight1(DRIVERIGHT1PORT , MOTOR_GEARSET_06, true);
pros::Motor driveRight2(DRIVERIGHT2PORT , MOTOR_GEARSET_06, false);
pros::Motor driveRight3(DRIVERIGHT3PORT , MOTOR_GEARSET_06,false);
pros::Motor driveRight4(DRIVERIGHT4PORT , MOTOR_GEARSET_06,true);

pros::Controller Vcontroller(pros::E_CONTROLLER_MASTER);

pros::Imu inertial(INERTIAL_PORT);

pros::ADIDigitalOut FrontClamp(PNEUM_PORT_1, true);
pros::ADIDigitalOut BackClamp(PNEUM_PORT_2);

pros::Rotation left_rot(LEFT_ROT_PORT);
pros::Rotation right_rot(RIGHT_ROT_PORT);
pros::Rotation back_rot(BACK_ROT_PORT);
//MISC===========================================================================================================
float tare_right;
float tare_left;
float tare_back;

char text1[100];
std::map<int,std::string> auton_names;
int auton = 0;
bool change = true;

bool p1 = false;
bool p2 = false;
bool p3 = false;

float prev_y_encoder = 0;
float prev_x_encoder = 0;
float prev_angle = 0;

//GLOBAL X
float pos_x = 0;
//GLOBAL Y
float pos_y = 0;
//FUNCS===========================================================================================================
//auton selector [WIP]
static lv_res_t btn_action(lv_obj_t * btn) {
	int btn_id = lv_obj_get_free_num(btn);
	if(change){
		if (auton < auton_names.size()) auton++;
		else auton = 1;
		sprintf(text1,"e%01d",auton);
		lv_label_set_text(selectionLabel, text1);
	}
	//printf("q",auton);
	return LV_RES_OK;
}

//run motors
void setDriveMotors(int l, int r) {
	driveLeft1.move(l);
	driveLeft2.move(l);
	driveLeft3.move(l);
	driveLeft4.move(l);
	driveRight1.move(r);
	driveRight2.move(r);
	driveRight3.move(r);
	driveRight4.move(r);
	}
// control motors with controller joystick
void setDrive() {
	int leftJoystickYInput = Vcontroller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int leftJoystickXInput = Vcontroller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);//*dir;
	pros::lcd::set_text(1, std::to_string(leftJoystickYInput));
  //Deadzones (10,10)
  if (abs(leftJoystickYInput) < 10) {
    leftJoystickYInput = 0;
  }
  if (abs(leftJoystickXInput) < 10) {
    leftJoystickXInput = 0;
  }

  setDriveMotors((leftJoystickYInput+leftJoystickXInput), (leftJoystickYInput+(-leftJoystickXInput)));
	//setDrive (leftJoystickYInput, rightJoystickYInput);
}
//trigger clamps on button events
void clamps() {
	if(Vcontroller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		p1 = !p1;
		FrontClamp.set_value(p1);
	}
	if(Vcontroller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		p2 = !p2;
		BackClamp.set_value(p2);
	}
}

//brakemodes .-.
void brake_coast(){
	driveLeft1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeft2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeft3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeft4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRight1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRight2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRight3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRight4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void brake_hold(){
	driveLeft1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeft2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeft3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveLeft4.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRight1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRight2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRight3.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveRight4.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void brake_brake(){
	driveLeft1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveLeft2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveLeft3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveLeft4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveRight1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveRight2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveRight3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	driveRight4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

//scuffed rot sensor stuff
void tare_rot(){
	while (tare_right == 0){
		tare_right = right_rot.get_position();
		pros::delay(1);
	}
	while (tare_left == 0){
		tare_left = left_rot.get_position();
		pros::delay(1);
	}
	while (tare_back == 0){
		tare_back = back_rot.get_position();
		pros::delay(1);
	}
}
float right_rot_pos(){
 	return right_rot.get_position() - tare_right;
}
float left_rot_pos(){
 	return left_rot.get_position() - tare_left;
}
float back_rot_pos(){
	return back_rot.get_position() - tare_back;
}

double getAngle(){
	double TRACKER_SCALAR = 1.057;
	double encoder_angle = ((right_rot_pos()*ENC_TO_INCH - left_rot_pos()*ENC_TO_INCH)/(TRACKER_DISTANCE))*TRACKER_SCALAR;
  return inertial.get_rotation();
  //return encoder_angle;
}

//odom :D
void updateCoords(){
	double angle = getAngle();
	double delta_angle = angle-prev_angle;
	//tracked values in inches(ideal centidegrees to inches conversion: 2.75*pi/36000)
	double curr_y_encoder = (left_rot_pos()+right_rot_pos())/2;
	double curr_x_encoder = back_rot_pos();
	int delta_local_y = (curr_y_encoder-prev_y_encoder);
	int delta_local_x = (curr_x_encoder-prev_x_encoder);// - TRACKER_DISTANCE_BACK*getAngle();
	int distance = sqrt(pow(delta_local_y,2)+pow(delta_local_x,2));
	//account for backwards movement
	if(delta_local_y < 0) angle += M_PI;
	double adjustment = atan((delta_local_x)/(delta_local_y));
	adjustment = (std::isnan(adjustment)) ? M_PI/2 : adjustment;
	pros::lcd::set_text(1, std::to_string((angle-adjustment)));
	pos_x += cos(angle - adjustment)*distance*ENC_TO_INCH;
	pos_y += sin(angle - adjustment)*distance*ENC_TO_INCH;
	prev_x_encoder = curr_x_encoder;
	prev_y_encoder = curr_y_encoder;
	prev_angle = angle;
}

void setAngle(double target, double tolerance, double kP, double kI, double kD, double time){
	brake_hold();
	double preverror = 0;
	double error = target-getAngle();
	double integral = 0;
	double ref = pros::millis();
	while (fabs(error) > tolerance && driveLeft1.is_stopped()){
		error = target-getAngle();
		double derivative = error-preverror;
		double pwr = error*kP + integral*kI + derivative*kD;
		setDriveMotors(pwr, -pwr);
		pros::lcd::set_text(1, std::to_string(error));
		preverror = error;
		integral += error;
		if (pros::millis()-ref > time){break;}
		pros::delay(10);
	}
	setDriveMotors(0,0);
	brake_coast();
}

void driveRel(double target, double tolerance, double kP, double kI, double kD, double time){
	//tune constants
//	setAngle(target_angle,10,2,0,0);
	brake_hold();
	double preverror = 0;
	double error = target;//sqrt(pow(targety,2)+pow(targetx,2))-sqrt(pow(back_rot_pos(), 2)+pow((left_rot_pos()+right_rot_pos())/2, 2));
	double integral = 0;
	double refT = pros::millis();
	double ref = (left_rot_pos()+right_rot_pos())/2;
	while(fabs(error)>tolerance  && driveLeft1.is_stopped()){
		error = target-((left_rot_pos()+right_rot_pos())/2 - ref);//sqrt(pow(targety,2)+pow(targetx,2))-sqrt(pow(back_rot_pos(), 2)+pow((left_rot_pos()+right_rot_pos())/2, 2));
		double derivative = error-preverror;
		double pwr = error*kP + integral*kI + derivative*kD;
		setDriveMotors(pwr,pwr);
		pros::lcd::set_text(1, std::to_string(error));
		preverror = error;
		integral += error;
		if (pros::millis()-refT > time){break;}
		pros::delay(10);
	}
	setDriveMotors(0,0);
	brake_coast();
}

void drive(double targetx, double targety, double tolerance, double kP, double kI, double kD, double time){
	double target_angle = atan(targety/targetx);
	//tune constants
//	setAngle(target_angle,10,2,0,0);

	brake_hold();
	double preverror = 0;
	double error = sqrt(pow(targety,2)+pow(targetx,2))-sqrt(pow(back_rot_pos(), 2)+pow((left_rot_pos()+right_rot_pos())/2, 2));
	double integral = 0;
	double ref = pros::millis();
	while(fabs(error)>tolerance  && driveLeft1.is_stopped()){
		error = sqrt(pow(targety,2)+pow(targetx,2))-sqrt(pow(back_rot_pos(), 2)+pow((left_rot_pos()+right_rot_pos())/2, 2));
		double derivative = error-preverror;
		double pwr = error*kP + integral*kI + derivative*kD;
		setDriveMotors(pwr,pwr);
		pros::lcd::set_text(1, std::to_string(error));
		preverror = error;
		integral += error;
		if (pros::millis()-ref > time){break;}
		pros::delay(10);
	}
	setDriveMotors(0,0);
	brake_coast();
}

void drive(double targetx, double targety, double tolerance, double kP, double kI, double kD){
	drive(targetx,targety,tolerance,kP,kI,kD,100000);
}

//weeeeee nonsense
float curv(float pts[][2], int pti){
	float x1 = pts[pti-1][0];
	float y1 = pts[pti-1][1];
  x1 += 0.0000001;
  float x2 = pts[pti][0];
	float y2 = pts[pti][1];
	float x3 = pts[pti+1][0];
	float y3 = pts[pti+1][1];
  float k1  = 0.5*(x1*x1-x2*x2+y1*y1-y2*y2)/(x1-x2);
  float k2 = (y1-y2)/(x1-x2);
  float b = 0.5*(x2*x2 - 2*x2*k1 + y2*y2 - x3*x3 + 2*x3*k1 - y3*y3)/(x3*k2 - y3 + y2 - x2*k2);
  float a = k1-k2*b;
  float r_sq = (x1-a)*(x1-a) + (y1-b)*(y1-b);
  float r = pow(r_sq,0.5);
  return 1/r;
}

void driveTurn(double x, double y, double end_angle, double tolerance, double scalar, double time){

	x += 0.000000001;
	while(d > tolerance){
		float d = sqrt(pow(x-pos_x,2)+pow(y-pos_y,2));
		float dx = cos(atan((y-pos_y)/(x-pos_x))+getAngle()*DEG_TO_RAD)*d;
		float c = (2*dx)/pow(d,2);
		float pts[][2] = {{pos_x,pos_y},{x,y},{x*(1+cos(end_angle)),y*(1+sin(end_angle))}};
		float v = std::min((float)MAX_VEL, TURN_VEL_K/curv(pts,1));
		float l = (v*(2+c*T)/2)//*y/abs(y);
		float r = (v*(2-c*T)/2)//*y/abs(y);
		setDriveMotors(l*scalar,r*scalar);
		pros::lcd::set_text(1, "l:" + std::to_string(l));
		pros::lcd::set_text(2, "r:" + std::to_string(r));
	}
}

//vel = min(MAX_VEL,TURN_VEL_K/curv(i))
//nonsense ends

//side mogo
void auton1(){
	FrontClamp.set_value(false);
	pros::delay(200);
	drive(180000,0,500,.03,0,0);
	pros::lcd::set_text(2, std::to_string(5327));
	pros::delay(300);
	//drive(-60000,0,500,.02,0,0);
	/*pros::delay(500);
	drive(120000,0,500,.02,0,0);*/
	drive(-90000,0,500,.02,0,0);
	pros::delay(300);
	FrontClamp.set_value(true);
	/*pros::delay(1000);
	drive(-60000,0,500,.02,0,0);*/
	pros::delay(1500);
	setAngle(-80, 1, 3, 0, 0, 5000);
	pros::lcd::set_text(2, std::to_string(32));
}

//tall mogo
void auton2(){
	drive(60000,0,500,.008,0,0);
	pros::delay(200);
	setAngle(-45, 0.5, 4, 0, 0, 1000);
	//pros::delay(100);
	FrontClamp.set_value(false);
	pros::delay(200);
	drive(56/ENC_TO_INCH,0,1000,.01,0,0);
	pros::delay(400);

	drive(-60000,0,500,.008,0,0);
	FrontClamp.set_value(true);
}

//side mogo revised
void auton3(){
	FrontClamp.set_value(false);
	pros::delay(200);
	drive(185000,0,500,.015,0,0.082,2000);
	pros::delay(300);
	//drive(-60000,0,500,.02,0,0);
	/*pros::delay(500);
	drive(120000,0,500,.02,0,0);*/
	drive(-90000,0,500,.015,0,0.082,2000);
	pros::delay(300);
	//FrontClamp.set_value(true);
	/*pros::delay(1000);
	drive(-60000,0,500,.02,0,0);*/
	//pros::delay(1500);
	setAngle(-80, 0.1, 50, 0, 300, 2000);
	pros::delay(300);
	driveRel(-40000,500,.015,0,0.082,2000);
	BackClamp.set_value(true);
	pros::delay(300);
	setAngle(-100, 0.1, 50, 0, 300, 2000);
	pros::delay(300);
	driveRel(100000,500,.015,0,0.082,2000);

}

void initialize() {
	pros::lcd::initialize();
	tare_rot();
	inertial.reset();
	/*change=true;
	auton_names.insert(std::make_pair(1,"auton1"));
	auton_names.insert(std::make_pair(2,"auton2"));
	auton_names.insert(std::make_pair(3,"auton3"));
	auton_names.insert(std::make_pair(4,"auton4"));
	//styles
	lv_style_copy(&buttonStyle1REL, &lv_style_plain);
	lv_style_copy(&buttonStyle1PR, &lv_style_plain);

	buttonStyle1REL.body.main_color = LV_COLOR_MAKE(0,150,150);
	buttonStyle1REL.body.grad_color = LV_COLOR_MAKE(0,150,0);
	buttonStyle1REL.body.radius = 0;
	buttonStyle1REL.text.color = LV_COLOR_MAKE(255, 255, 255);

	buttonStyle1PR.body.main_color = LV_COLOR_MAKE(0,255,255);
	buttonStyle1PR.body.grad_color = LV_COLOR_MAKE(0,255,0);
	buttonStyle1PR.body.radius = 0;
	buttonStyle1PR.text.color = LV_COLOR_MAKE(255, 255, 255);

	//buttons
	autonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_obj_set_free_num(autonButton,1); //id
	lv_btn_set_action(autonButton, LV_BTN_ACTION_CLICK, btn_action); //click action
	lv_btn_set_style(autonButton, LV_BTN_STYLE_REL, &buttonStyle1REL); //unpressed style
	lv_btn_set_style(autonButton, LV_BTN_STYLE_PR, &buttonStyle1PR); //pressed style
	lv_obj_set_size(autonButton, 200, 50); //size
	lv_obj_align(autonButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); //pos

	autonButtonLabel = lv_label_create(autonButton, NULL);
	lv_label_set_text(autonButtonLabel, "Change Auton");

	selectionLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
	lv_label_set_text(selectionLabel, "None Selected"); //sets label text
	lv_obj_align(selectionLabel, NULL, LV_ALIGN_CENTER, 0, 0);*/
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	driveTurn(10, -10, 45, 1, 1, 0);
	//pidref:
	//setAngle(-180, 0.1, 50, 0, 300, 5000);
	//drive(200000,0,500,0.015,0,0.082,5000);
	//pros::lcd::set_text(7,std::to_string(getAngle()));
	//pros::lcd::set_text(7,std::to_string(100000-(left_rot_pos()+right_rot_pos())/2));
}

void opcontrol() {
	while(true){
		setDrive();
		clamps();
		pros::lcd::set_text(1, std::to_string(getAngle()));
		pros::lcd::set_text(2, "y: " + std::to_string(pos_y));
		pros::lcd::set_text(3, "x: " + std::to_string(pos_x));
		//updateCoords();
		pros::delay(10);
	}
}
