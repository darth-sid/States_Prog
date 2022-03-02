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
#define DRIVELEFT1PORT 9
#define DRIVELEFT2PORT 10
#define DRIVELEFT3PORT 18
#define DRIVELEFT4PORT 20
#define DRIVERIGHT1PORT 7
#define DRIVERIGHT2PORT 8
#define DRIVERIGHT3PORT 19
#define DRIVERIGHT4PORT 17

#define PNEUM_PORT_1 1
#define PNEUM_PORT_2 2
#define PNEUM_PORT_3 3
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
pros::Motor driveRight1(DRIVERIGHT1PORT , MOTOR_GEARSET_06, false);
pros::Motor driveRight2(DRIVERIGHT2PORT , MOTOR_GEARSET_06, true);
pros::Motor driveRight3(DRIVERIGHT3PORT , MOTOR_GEARSET_06,false);
pros::Motor driveRight4(DRIVERIGHT4PORT , MOTOR_GEARSET_06,true);

pros::Controller Vcontroller(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut FrontClamp(PNEUM_PORT_1);
pros::ADIDigitalOut BackClamp(PNEUM_PORT_2);
pros::ADIDigitalOut SideClamp(PNEUM_PORT_3);
//MISC===========================================================================================================
char text1[100];
std::map<int,std::string> auton_names;
int auton = 0;
bool change = true;

bool p1 = false;
bool p2 = false;
bool p3 = false;

//FUNCS===========================================================================================================
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

void clamps() {
	if(Vcontroller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		p1 = !p1;
		FrontClamp.set_value(p1);
	}
	if(Vcontroller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		p2 = !p2;
		BackClamp.set_value(p2);
	}
	if(Vcontroller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
		p3 = !p3;
		SideClamp.set_value(p3);
	}
}

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

void initialize() {
	change=true;
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
	lv_obj_align(selectionLabel, NULL, LV_ALIGN_CENTER, 0, 0);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	change=false;
	if(auton==0) auton=1;
	sprintf(text1,"a%01d",auton);
	lv_label_set_text(selectionLabel, text1);
}

void opcontrol() {
	setDrive();
	clamps();
}
