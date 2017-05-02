/*******************************************************************************
*
*	balancebot.c
*
*	pgaskell@umich.edu
*******************************************************************************/

#include <lcm/lcm.h>
#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../robocape/src/devices/motor.h"
//#include "../lcmtypes/odometry_t.h"
#include "../lcmtypes/balancebot_imu_t.h"
#include "../lcmtypes/robot_path_t.h"
#include "../lcmtypes/pose_xyt_t.h"

#define 	SAMPLE_RATE_HZ 			200	  // main filter and control loop speed
#define 	SAMPLE_DELTA_T 			1.0/(float)SAMPLE_RATE_HZ // 1/sample_rate

#define		PRINTF_HZ				20
#define 	GEARBOX					34
#define 	ENCODER_RES				48
#define		WHEEL_BASE				0.217
#define 	VEL_SENSITIVITY			0.25
#define 	TURN_SENSITIVITY		3
#define     PI_val                  3.1415926
#define		MAX_FOWARD_VEL			1.6  //in meter
#define		MAX_ANG_VEL				15
#define     ERR_TOL_X               0.01
#define     ERR_TOL_Y               0.01
#define     ERR_TOL_DIST            0.05
#define     ERR_TOL_THETA           0.2

#define		BALANCE_ANG				 -1.5523//(-1.6 + 0.0323)
#define		ERR_TOL_DIST_STAY		0.01
#define		ODOM_GYRO_THRESHOLD 	0.0018//0.0018
#define		ODOM_GYRO_THRESHOLD_max	1
#define		TURN_SPEED				-1.5//1.2//4 or 4.2 for odom//1 for wheel//

#define     TILT_ANGLE_CONST              2

typedef enum bot_motion_state_t {
	IDLE,
	READY,
	TURN,
	RUN,
	RC,
} bot_motion_state_t;


//typedef struct set_vel_t {
//	float set_forward_vel;
//	float set_angular_vel;
//} set_vel_t;

// TODO: add PID and state variables
typedef struct botState// bot kinematic state
{
	//current pos values
	float pitch;	//up and down
	float yaw;		//turn left or right
	float prev_yaw;
	float alpha;	//foward speed
	float alpha_left;
	float alpha_right;
	//speed values = current value - last value
	float pitchSpeed;
	float yawSpeed;
	float alphaSpeed;
	//forward and angular velocity
	float set_forward_vel;
	float set_forward_vel_balance;
	float set_angular_vel;
	float set_heading;
	// feedback velocity
	float fb_forward_vel;
	float fb_angular_vel;
	float fb_wheel_vel_l;
	float fb_wheel_vel_r;
	//error
	float delta_theta_gyro;
	float delta_theta;
} botState;

typedef struct PID//this is for PID
{
	float kp;
	float ki;
	float kd;
	float last_err;
	float accumulate; //for I: accu = accu + ki*err*DT

} PID;

typedef struct world_odometry//this is for odometry
{
	float x;
	float y;
	float theta;
	float prev_encoder_val_l;
	float prev_encoder_val_r;
	float prev_theta;
}world_odometry;

typedef struct ADC_val//this is for PID reading from ADC port
{
	float p;
	float i;
	float d;
	float p_scale;
	float i_scale;
	float d_scale;
}ADC_val;
ADC_val adc_PID;
// Global Variables
imu_data_t imu_data;

pose_xyt_t pose_xyz_stable_point;

pose_xyt_t pose_xyz = {
	.utime = 0,
	.x = 0,
	.y = 0,
	.theta = 0,
};

motor_t right_motor;
motor_t left_motor;
botState bot_state;

//PID pid;
world_odometry world_odom;
balancebot_imu_t balancebot_imu_data;

PID balance_PID = {
	.kp = 9.5,//8.23,//4 * 0.6,
	.ki = 0.0,//4 * 0.6 * 1 / 2,
	.kd = 0.2,//0.192,//4 * 0.6 * 1 / 2, 
	.last_err = 0.0,
	.accumulate = 0.0,
};

PID heading_PID = {
	.kp = 0,
	.ki = 0,
	.kd = 0.000,
	.last_err = 0.0,
	.accumulate = 0.0,
};


PID balance_angle_PID = {
	.kp = 0.4464,//0.231,0.147, 0.005
	.ki = 0.0007,
	.kd = 0.0099,
	.last_err = 0,
	.accumulate = 0,
};

PID forwardPID = {
	.kp = 0.4 * 0.2,
	.ki = 0.4 * 0.2 * (0.3/5),
	.kd = 0.4 * 0.2 * (0.03/5),
	.last_err = 0,
	.accumulate = 0,
};

PID leftPID = {
	.kp = 0.4 * 0.1,
	.ki = 0.4 * 0.1 * (3/5),
	.kd = 0.4 * 0.1 * (0.03/5),
	.last_err = 0,
	.accumulate = 0,
};
PID rightPID = {
	.kp = 0.4 * 0.1,
	.ki = 0.4 * 0.1 * (3/5),
	.kd = 0.4 * 0.1 * (0.03/5),
	.last_err = 0,
	.accumulate = 0,
};

PID turnPID = {
	.kp = 2 * 0.02,//0.04 * 0.5,
	.ki = 2 * 0.02 * (0.01 /2),//0.04 * 0.5 * (3/15),
	.kd = 2 * 0.02 * (0.01 /8),//0.04 * 0.5 * (0.03/20),
	.last_err = 0,
	.accumulate = 0,
};

PID pathPID = {
	.kp = 4,
	.ki = 0,
	.kd = 1 * (0.01),
	.last_err = 0,
	.accumulate = 0,
};

int balance_count = 0;
int odom_count = 0;

float vCommand = 0;
float tCommand = 0;

float lCommand = 0;
float rCommand = 0;


float cur_leftSpeed = 0;
float cur_rightSpeed = 0;

float tilt_angle = 0;
float stay_x = 0;
float stay_y = 0;
float stay_dist = 0;
float stay_dist_prev = 0;
float show_some_data = 0.01;
PID demo_data = {
	.kp = 9.5,//8.23,//4 * 0.6,
	.ki = 0.0,//4 * 0.6 * 1 / 2,
	.kd = 0.2,//0.192,//4 * 0.6 * 1 / 2, 
	.last_err = 0.0,
	.accumulate = 0.0,
};

d_filter_t velocity_filter,angular_filter,delta_theta_gyro_filter,delta_theta_odom_filter;

bot_motion_state_t bot_motion_state = IDLE;
unsigned int path_idx;
float prev_err_dist = 5; //randomly high value set to initialise this variable
robot_path_t *path_list;
// LCM Global Variables
lcm_t * lcm;
static const char ODOMETRY_CHANNEL[] = "ODOMETRY_POSE"; //odometry channel
static const char ROBOT_PATH_T_CHANNEL[] = "CONTROLLER_PATH"; //robot_path_t channel
static const char TRUEPOSE_CHANNEL[] = "TRUE_POSE"; //robot_path_t channel
// IMU interrupt routine
int balancebot_controller();

//other functions
void setup_motors();
float velocity_regulate(float vel);
float heading_pose_botgui(pose_xyt_t gui_pose, world_odometry bot_pose);
void bot_motion_control();
void odometry_get(int freq_div);
void imu_get(int freq_div);
float PID_Cal(PID* pid, float err, float DT);
void PID_ADC_read();
float path_vel_control(float err_dist, float ref_vel, float dist_cut, float delta_vel);
float theta_regulate(float theta);
void cascade_control_with_caster();
//threads
void* printf_loop(void* ptr);
void* lcm_publish_loop(void* ptr);

void init_parameters(){
	bot_state.set_forward_vel = 0;
	bot_state.set_forward_vel_balance = 0;
	bot_state.set_angular_vel = 0;
	/*adc_PID.p_scale = 1.8 / 100;
	adc_PID.i_scale = 1.8 *10;
	adc_PID.d_scale = 1.8 * 1;*/
	adc_PID.p_scale = 1.8 /2;
	adc_PID.i_scale = 1.8 *10;
	adc_PID.d_scale = 1.8 * 20;
	// adc_PID.p_scale = 1.8 / 100;
	// adc_PID.i_scale = 1.8 *10;
	// adc_PID.d_scale = 1.8 * 10;
	// pose_xyz_stable_point.x = 0;
	// pose_xyz_stable_point.y = 0;
}

void RC_Control(int freq_div){
	// if(bot_motion_state == 	READY || bot_motion_state == TURN || bot_motion_state == RUN ){
	// 	return;
	// }
	float forward_Command = get_dsm2_ch_normalized(3);
	float turn_Command = get_dsm2_ch_normalized(2);
	
	// if(signal<0.01){
	// 	bot_motion_state = IDLE;
	// 	stay_x = pose_xyz.x;
	// 	stay_y = pose_xyz.y;
	// 	stay_dist_prev = 0;
	// }
	// else{
	// 	bot_motion_state = RC;
	// }
	
	// show_some_data = forward_Command;
	bot_state.set_forward_vel = forward_Command * 0.2;
	// bot_state.set_angular_vel = turn_Command;
	bot_state.set_angular_vel = PID_Cal(&turnPID, turn_Command * TURN_SENSITIVITY - bot_state.fb_angular_vel, freq_div);
}


void robot_path_handler(const lcm_recv_buf_t *rbuf, const char * channel, const robot_path_t * msg, void * user){

// your code here.  Try printing the message, accessing each element with msg->element
// alternatively, pass it a pointer to a local lcmtype struct and print in the main loop
	if(bot_motion_state != IDLE)
		return;
	int i =0;
	//printf("Receive LCM msg from channel CONTROLLER_PATH.\n");
	for(i=0;i<msg->path_length;i++){
		float temp_x = msg->path[i].y;
		float temp_y = msg->path[i].x;
		msg->path[i].x = temp_x;
		msg->path[i].y = temp_y;
		printf("Point index No.: %d, x: %6.2f, y: %6.2f\n", (i+1), msg->path[i].x, msg->path[i].y);
	}
	path_list = robot_path_t_copy(msg);// copy const robot_path_t * msg to global pointer path_list
	if(msg->path_length < 2)
		return;
	path_list->path[0].x = world_odom.x;
	path_list->path[0].y = world_odom.y;
	bot_motion_state = RUN;// set up state of bot motion, enum bot_motion_state_t
	path_idx = 0;// loop from the first/second point until the last point
}


int main(int argc, char *argv[]){
	init_parameters();
	//initialize robocape

	printf("main start\n");
	initialize_cape();
	set_cpu_frequency(FREQ_1000MHZ);
    lcm = lcm_create(NULL);
    
	velocity_filter   = create_first_order_lowpass(SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	angular_filter   = create_first_order_lowpass(SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	delta_theta_gyro_filter = create_first_order_lowpass(SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	delta_theta_odom_filter = create_first_order_lowpass(SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother

	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}

	//start lcm publish thread
	pthread_t  lcm_publish_thread;
	pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);

	//initialize motors
	setup_motors();

	// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Y_UP;

	// start imu
	if(initialize_imu_dmp(&imu_data, imu_config)){
		printf("ERROR: can't talk to IMU\n");
		return -1;
	}

	bot_state.set_forward_vel = 0;
	bot_state.set_angular_vel = 0;
	// leave time for imu to initialize
	world_odom.theta = -PI_val*3/2;
	usleep(4000000);

    // IMU interrupt timer used to also time controller, control all the time sequence
	//attach control routine to imu interrupt
	set_imu_interrupt_func(&balancebot_controller);
	
    if(initialize_dsm2()){
        // if init returns -1 if there was a problem 
        // most likely no calibration file found
        printf("run calibrate_dsm2 first\n");
        return -1;
    }

	robot_path_t_subscribe(lcm, ROBOT_PATH_T_CHANNEL, &robot_path_handler, NULL);
	// start in the RUNNING state
	set_state(RUNNING);

    set_motor_on(left_motor);
    set_motor_on(right_motor);
	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// always sleep at some point
		// lcm_handle(lcm);
		lcm_handle_timeout(lcm,10000);
		// usleep(10000);
	}
	cleanup_cape(); // exit cleanly
    uninit_motor(left_motor);
    uninit_motor(right_motor);
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}
/*******************************************************************************
* balance_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*******************************************************************************/

int control_counter = 0;
float heading_vel = 0;

void tune_PID(PID *p){
	PID_ADC_read();
    p->kp = adc_PID.p;
    p->ki = adc_PID.i;
    p->kd = adc_PID.d;	
}

void cascade_control(){

	imu_get(1);
	odom_count+=1;
	if(odom_count == 1){
		odom_count = 0;
		odometry_get(1);		
	}
	control_counter += 1;
	if(control_counter = 20){
		RC_Control(15);
		//bot_motion_control(20);
		control_counter = 0;
	}
	balance_count += 1;
 	//bot_state.set_forward_vel =0 ;
 	// bot_state.set_angular_vel =5;
	//tune_PID(&balance_angle_PID);

	if(balance_count <= 5){
		tilt_angle = PID_Cal(&balance_angle_PID, bot_state.set_forward_vel - bot_state.fb_forward_vel, SAMPLE_DELTA_T*5);
		balance_count = 0; 
	}

	bot_state.set_forward_vel_balance = -PID_Cal(&balance_PID, tilt_angle+BALANCE_ANG - bot_state.pitch, SAMPLE_DELTA_T*1);


	float rightMotorSpeed = (bot_state.set_forward_vel_balance ) * 1 + bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	float leftMotorSpeed = (bot_state.set_forward_vel_balance ) * 1 - bot_state.set_angular_vel * WHEEL_BASE / 2.0;

	// float rightMotorSpeed = (bot_state.set_forward_vel ) * 1 + bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	// float leftMotorSpeed = (bot_state.set_forward_vel ) * 1 - bot_state.set_angular_vel * WHEEL_BASE / 2.0;

	leftMotorSpeed = velocity_regulate(leftMotorSpeed);
	rightMotorSpeed = velocity_regulate(rightMotorSpeed);

	

	set_motor_speed(&left_motor, leftMotorSpeed);
	set_motor_speed(&right_motor, rightMotorSpeed);

}

void cascade_control_with_caster(){

	imu_get(1);
	odom_count+=1;
	if(odom_count == 1){
		odom_count = 0;
		odometry_get(1);		
	}
	control_counter += 1;
	if(control_counter = 20){
		//RC_Control(15);
		bot_motion_control(20);
		control_counter = 0;
	}
	balance_count += 1;
 	//bot_state.set_forward_vel =0 ;
 	// bot_state.set_angular_vel =5;
	//tune_PID(&balance_angle_PID);

/*	if(balance_count <= 5){
		tilt_angle = PID_Cal(&balance_angle_PID, bot_state.set_forward_vel - bot_state.fb_forward_vel, SAMPLE_DELTA_T*5);
		balance_count = 0; 
	}

	bot_state.set_forward_vel_balance = -PID_Cal(&balance_PID, tilt_angle+BALANCE_ANG - bot_state.pitch, SAMPLE_DELTA_T*1);

*/
	float rightMotorSpeed = (bot_state.set_forward_vel) * 1 + bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	float leftMotorSpeed = (bot_state.set_forward_vel ) * 1 - bot_state.set_angular_vel * WHEEL_BASE / 2.0;

	// float rightMotorSpeed = (bot_state.set_forward_vel ) * 1 + bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	// float leftMotorSpeed = (bot_state.set_forward_vel ) * 1 - bot_state.set_angular_vel * WHEEL_BASE / 2.0;
    demo_data.kp = rightMotorSpeed;
	leftMotorSpeed = velocity_regulate(leftMotorSpeed*1.9);
	rightMotorSpeed = velocity_regulate(rightMotorSpeed*1.9);

	

	set_motor_speed(&left_motor, leftMotorSpeed);
	set_motor_speed(&right_motor, rightMotorSpeed);

}

/*
void simple_control(){
	imu_get(1);
	odom_count+=1;
	if(odom_count == 5){
		odom_count = 0;
		odometry_get(5);		
	}
	control_counter += 1;
	if(control_counter = 10){
		// RC_Control(10);
		// bot_motion_control(10);
		control_counter = 0;
	}
	balance_count += 1;

	if(balance_count == 10){
		heading_vel = PID_Cal(&heading_PID, bot_state.set_forward_vel - bot_state.fb_forward_vel, SAMPLE_DELTA_T*10);
		balance_count = 0; 
	}

	bot_state.set_forward_vel_balance = -PID_Cal(&balance_PID, BALANCE_ANG - bot_state.pitch, SAMPLE_DELTA_T*1);
	float rightMotorSpeed = (heading_vel+ bot_state.set_forward_vel_balance ) * 1 + bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	float leftMotorSpeed= (heading_vel+ bot_state.set_forward_vel_balance ) * 1 - bot_state.set_angular_vel * WHEEL_BASE / 2.0;
	leftMotorSpeed = velocity_regulate(leftMotorSpeed);
	rightMotorSpeed = velocity_regulate(rightMotorSpeed);

	set_motor_speed(&left_motor, leftMotorSpeed);
	set_motor_speed(&right_motor, rightMotorSpeed);

}
*/
int balancebot_controller(){
	if((BALANCE_ANG - bot_state.pitch)>TILT_ANGLE_CONST ||(BALANCE_ANG - bot_state.pitch)< - TILT_ANGLE_CONST)
		return 0;
	cascade_control();
	// cascade_control_with_caster();
	// simple_control();

	if(get_state() == EXITING){
		return 0;
	}
	return 1;
}


float PID_Cal(PID* pid, float err, float DT){
	pid->accumulate += err*DT;//this is not pid.i************
	float response = pid->kp*err + pid->ki*pid->accumulate + pid->kd * (err - pid->last_err) / DT;
	pid->last_err = err;
	return response;
}

void PID_ADC_read(){
	adc_PID.p = get_adc_volt(0)/adc_PID.p_scale;
	adc_PID.i = get_adc_volt(1)/adc_PID.i_scale;
	adc_PID.d = get_adc_volt(2)/adc_PID.d_scale;
}

float heading_pose_botgui(pose_xyt_t target_pose, world_odometry bot_pose){

	    //float relative_theta = atan2(target_pose.y - bot_pose.y, target_pose.x - bot_pose.x);
        float theta_temp = atan2(target_pose.y - bot_pose.y, target_pose.x - bot_pose.x) - bot_pose.theta;
       // printf(" check line:%6.2f,%6.2f,%6.2f \n",relative_theta, bot_pose.theta, theta_temp);
        return theta_regulate(theta_temp);
    }                    



/*************************************************************
* path_control_read_cur_pose() and path_control_stay() are used to RC control
**************************************************************/

int rotate_count = 0;

void bot_motion_control(int freq_div){

	if(bot_motion_state == IDLE){
		return;
		float wx = world_odom.x;
		float wy = world_odom.y;
		float wt = world_odom.theta;

		float cx = stay_x;
		float cy = stay_y;
		stay_dist = sqrt((cx - wx)*(cx - wx) + (cy - wy)*(cy - wy));
		bot_state.set_forward_vel = -0.15*(stay_dist - stay_dist_prev);
		stay_dist_prev = stay_dist;
		return;
	}
/*	if(bot_motion_state == RC){
		return;
	}*/
	// if(path_control(path_list->path[path_idx], path_list->path[path_idx+1], freq_div)){
	// 	path_idx += 1;
	// 	if(path_idx == path_list->path_length - 1){
	// 		bot_motion_state = IDLE;
	// 	}
	// }

	float wx = world_odom.x;
	float wy = world_odom.y;
	float wt = world_odom.theta;

	//get target pos from command
	float cx = path_list->path[path_idx].x;
	float cy = path_list->path[path_idx].y;

	float err_dist = sqrt((cx - wx)*(cx - wx)   + (cy - wy)*(cy - wy));

	if(err_dist < ERR_TOL_DIST ){
		if(fabs(bot_state.fb_forward_vel) < 0.03){
			rotate_count += 1;
			if(rotate_count == 10){
				rotate_count = 0;
				bot_motion_state = TURN;
				path_idx += 1;
			}

		}
		bot_state.set_forward_vel = 0;
		bot_state.set_angular_vel = 0;
		pathPID.last_err = 0;
		pathPID.accumulate = 0;
		
		if(path_idx >= path_list->path_length){
			bot_motion_state = IDLE;
			stay_x = world_odom.x;
			stay_y = world_odom.y;
			stay_dist_prev = 0;
			return;
		}
	}

	//run the robot to desired pos and direction
	else{
		//when receive a new command target point, first make sure the robot is on the correct direction?
		//pure turn, no heading
		if( bot_motion_state == TURN){
			// printf(" start turn : wx:%6.2f  \n",wx);
			bot_state.set_forward_vel = 0;
			float theta_err = heading_pose_botgui(path_list->path[path_idx], world_odom);//cur_path->path[path_idx].theta;
			if(fabs(theta_err) < ERR_TOL_THETA){
				bot_state.set_angular_vel = 0;
				// printf(" finish Turn : wx:%6.2f  \n",wx);
				bot_motion_state = RUN;
			}
			else{
				// printf(" theta err ::%6.2f  \n",theta_err);
				bot_state.set_angular_vel = theta_err<0 ? TURN_SPEED:TURN_SPEED;//4 : 4;for odom//1:1 for bot caster
			}
		}
		if( bot_motion_state == RUN){
			//the velocity can be otherwise calculated by defining a velocity profile
			
			//bot_state.set_forward_vel = path_vel_control(err_dist,0.2,0.1,0.02);//float path_vel_control(float err_dist, float ref_vel, float dist_cut, float delta_vel)
			//bot_state.set_forward_vel = path_vel_control(err_dist,0.2,0.1,0.02);//0.02

			bot_state.set_forward_vel = 0.2;
			//calculate if the robot is right on the path
			float px = path_list->path[path_idx - 1].x;
			float py = path_list->path[path_idx - 1].y;
			float dir = (cx - px) * (wy - py) - (cy - py) * (wx - px);

			float dir_theta = 1.5;

			//result of atan func: -pi ~ pi. Dangerous range (pi + 1 = - pi -1)
			float target_theta = atan2(cy - py, cx - px) - dir_theta * dir;

			// printf(" run : dir:%6.4f  \n",target_theta);
			float des_theta = path_list->path[path_idx].theta; 
			//two method to make bot run on the line
			//PID method
			bot_state.set_angular_vel = PID_Cal(&pathPID, theta_regulate(target_theta - world_odom.theta),freq_div);

		}
	}
	prev_err_dist = err_dist;


}

/*************************************************************
* path_vel_control() is used to give ramp forward velocity while runing the square
**************************************************************/

float path_vel_control(float err_dist, float ref_vel, float dist_cut, float delta_vel){
	float temp_vel = 0;
	if (err_dist > 1-dist_cut && err_dist <= 1)
		temp_vel = (1-err_dist)*ref_vel+delta_vel;
	else if (err_dist <= dist_cut)
		temp_vel = err_dist*temp_vel+delta_vel;
	else if (err_dist>1)
		temp_vel = delta_vel;
	else 
		temp_vel = 0.16+delta_vel;
	return temp_vel;
}

/*************************************************************
* path_control_read_cur_pose() and path_control_stay() are used to RC control
**************************************************************/
// read the current point and save
int path_control_read_cur_pose(){
	pose_xyz_stable_point = pose_xyz;
}
// make bot stable in a given point, only move forward/backward
int path_control_stay(pose_xyt_t target_xyz, int freq_div){

	float sample_motion_Hz = SAMPLE_RATE_HZ/freq_div;	
	float wx = world_odom.x;
	float wy = world_odom.y;
	float cx = target_xyz.x;
	float cy = target_xyz.y;

	float err_dist = sqrt((cx - wx)*(cx - wx)   + (cy - wy)*(cy - wy));
	if(err_dist < ERR_TOL_DIST_STAY){
			return 1;
	}
	else{
		bot_state.set_forward_vel = velocity_regulate(-0.5* err_dist); // This need to change to PID

			//bot_state.set_angular_vel += PID_Cal(&pathPID, 0 - dir, 1/sample_motion_Hz);
	}
}
/*************************************************************
* path_control() is used to path command control
**************************************************************/
int path_control(pose_xyt_t previous_xyz, pose_xyt_t target_xyz, int freq_div){
	float sample_motion_Hz = SAMPLE_RATE_HZ/freq_div;	
	float wx = world_odom.x;
	float wy = world_odom.y;
	float wt = world_odom.theta;
	//get target pos from command
	float cx = target_xyz.x;
	float cy = target_xyz.y;
	float px = previous_xyz.x;
	float py = previous_xyz.y;

	float theta_err = heading_pose_botgui(target_xyz, world_odom);//path_list->path[path_idx].theta;


	float err_dist = sqrt((cx - wx)*(cx - wx)   + (cy - wy)*(cy - wy));
	if(err_dist < ERR_TOL_DIST){
		return 1;
	}
	else{
		if(fabs(theta_err) < ERR_TOL_THETA){
			bot_state.set_forward_vel = 0.2;
			//bot_state.set_forward_vel += PID_Cal(&pathPID, 0 - err_dist, 1/sample_motion_Hz);
		}
		else{
			float dir = (cx - px) * (wy - py) - (cy - py) * (wx - px);
			printf(" run : dir:%6.2f  \n",dir);
			bot_state.set_angular_vel = PID_Cal(&pathPID, 0 - dir, 1/sample_motion_Hz);

			//bot_state.set_angular_vel = theta_err<0 ? 0.05 : -0.05;
			//bot_state.set_angular_vel += PID_Cal(&pathPID, 0 - dir, 1/sample_motion_Hz);
		}
	}

}




/*******************************************************************************
*imu_get()
*
*******************************************************************************/

void imu_get(int freq_div){
/**

    balancebot_imu_data.timestamp = micros_since_epoch();

    balancebot_imu_data.accel[0] = imu_data.accel[0];
    balancebot_imu_data.accel[1] = imu_data.accel[1];
    balancebot_imu_data.accel[2] = imu_data.accel[2];

    balancebot_imu_data.gyro[0] = imu_data.gyro[0];
    balancebot_imu_data.gyro[1] = imu_data.gyro[1];
    balancebot_imu_data.gyro[2] = imu_data.gyro[2];

    balancebot_imu_data.mag[0] = imu_data.mag[0];
    balancebot_imu_data.mag[1] = imu_data.mag[1];
    balancebot_imu_data.mag[2] = imu_data.mag[2];

    balancebot_imu_data.temp = imu_data.temp;

	balancebot_imu_data.TB_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    balancebot_imu_data.TB_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    balancebot_imu_data.TB_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];

    balancebot_imu_data.quat[0] = imu_data.dmp_quat[0];
    balancebot_imu_data.quat[1] = imu_data.dmp_quat[1];
    balancebot_imu_data.quat[2] = imu_data.dmp_quat[2];
    balancebot_imu_data.quat[3] = imu_data.dmp_quat[3];

**/

	float sample_imu_Hz = SAMPLE_RATE_HZ/freq_div;

	// bot_state.pitchSpeed = imu_data.gyro[0];
	bot_state.pitchSpeed = (imu_data.dmp_TaitBryan[TB_PITCH_X] - bot_state.pitch)/sample_imu_Hz;
	bot_state.pitch = imu_data.dmp_TaitBryan[TB_PITCH_X];

	// bot_state.yawSpeed = imu_data.gyro[2];
	bot_state.yawSpeed = (imu_data.dmp_TaitBryan[TB_YAW_Z] - bot_state.yaw)/sample_imu_Hz;
	bot_state.yaw = imu_data.dmp_TaitBryan[TB_YAW_Z];	
}

/*******************************************************************************
*odometry_get()
*
updte the forward and angular velocity and odometry x,y,theta
*******************************************************************************/

void odometry_get(int freq_div){
    // Read encoder, get difference data of two wheels in certain sampling time interval
	float cur_encoder_val_l = (float)get_encoder_pos(1);
	float cur_encoder_val_r = (float)get_encoder_pos(2);
	float diff_encoder_val_l = cur_encoder_val_l - world_odom.prev_encoder_val_l;
	float diff_encoder_val_r = cur_encoder_val_r - world_odom.prev_encoder_val_r;
	world_odom.prev_encoder_val_l = cur_encoder_val_l;
	world_odom.prev_encoder_val_r = cur_encoder_val_r;

    float sample_odom_Hz = SAMPLE_RATE_HZ/freq_div;

	//get encoder data that is difference of urrent and previous state
	float b = 0.217; //wheel base in meter
	float D = 0.08; //wheel diameter in meter
	float C = GEARBOX*ENCODER_RES; //counts/rev(encoder)*gear ratio
	world_odom.prev_theta = world_odom.theta;

	float d_L = ((PI_val*D)/C)*diff_encoder_val_l;
	float d_R = ((PI_val*D)/C)*diff_encoder_val_r;
	float delta_yr = 0;//no slide slip
	float delta_xr = (d_R + d_L)/2.0; //distance travelled by robot
	float delta_theta = (d_R - d_L)/b; //change in heading
	float forward_vel = delta_xr*sample_odom_Hz;
	float angular_vel = delta_theta*sample_odom_Hz;

	// update data in bot_state (global structure variable)
    // bot_state.fb_forward_vel = forward_vel;
    // bot_state.fb_angular_vel = angular_vel;

    bot_state.fb_forward_vel = march_filter(&velocity_filter, forward_vel);
    bot_state.fb_angular_vel = march_filter(&angular_filter, angular_vel);

	bot_state.fb_wheel_vel_l = d_L * sample_odom_Hz;
	bot_state.fb_wheel_vel_r = d_R * sample_odom_Hz;

    bot_state.delta_theta = delta_theta;

    float delta_theta_gyro = theta_regulate(theta_regulate(bot_state.yaw) - bot_state.prev_yaw);// notice:delta_theta and delta_theta_gyro have different signal
    bot_state.prev_yaw = theta_regulate(bot_state.yaw);

    bot_state.delta_theta_gyro = march_filter(&delta_theta_gyro_filter, delta_theta_gyro);
    bot_state.delta_theta = march_filter(&delta_theta_odom_filter, delta_theta);

/*	float diff_gyro_odom = theta_regulate(theta_regulate(bot_state.delta_theta_gyro) + theta_regulate(bot_state.delta_theta));
	if (fabs(diff_gyro_odom) > ODOM_GYRO_THRESHOLD && fabs(diff_gyro_odom) < ODOM_GYRO_THRESHOLD_max)
		world_odom.theta = theta_regulate(world_odom.theta - bot_state.delta_theta_gyro);
	else
		world_odom.theta = theta_regulate(world_odom.theta + delta_theta);
*/
	//world_odom.theta = theta_regulate(world_odom.theta - bot_state.delta_theta_gyro);
	world_odom.theta = theta_regulate(world_odom.theta + delta_theta);

	//updating world state 	
	world_odom.x = world_odom.x + delta_xr*cos(world_odom.theta) - delta_yr*sin(world_odom.theta);
	world_odom.y = world_odom.y + delta_xr*sin(world_odom.theta) - delta_yr*cos(world_odom.theta);


	// publishing lcm commands assigning theta after gryrodometry
 	pose_xyz.utime = micros_since_epoch();
	pose_xyz.x = world_odom.y;
	pose_xyz.y = world_odom.x;
	pose_xyz.theta = -world_odom.theta + PI_val/2;
	// pose_xyz.x = world_odom.x + delta_xr*cos(pose_xyz.theta) - delta_yr*sin(pose_xyz.theta);
	// pose_xyz.y = world_odom.y + delta_xr*sin(pose_xyz.theta) - delta_yr*cos(pose_xyz.theta);
	
}
/*******************************************************************************
* theta_regulate()
* check angle to make it range -Pi ~ +Pi
*******************************************************************************/
float theta_regulate(float theta){
	
    while ((theta<-PI_val)||(theta>PI_val)){
    	if (theta < -PI_val)
    		theta = theta + PI_val*2;
    	if (theta > PI_val)
    		theta = theta - PI_val*2;
    }
    return theta;
}
/*******************************************************************************
* velocity_regulate()
* check forward velocity to make it range -1 ~ +1
*******************************************************************************/
float velocity_regulate(float vel){

	if (vel < -0.99)
		vel = -0.99;
	if (vel > 0.99)
		vel = 0.99;
	// if (fabs(vel) < 0.05 && vel>0)
	// 	vel = 0.05;
	// if (fabs(vel) < 0.05 && vel<0)
	// 	vel = -0.05;
    return vel;
}

/*******************************************************************************
* setup_motors()
*
*******************************************************************************/
void setup_motors(){
// TODO: INITIALIZE THE MOTORS
// use set_encoder_pos and init_motor commands
	//init_motor, depend on the wiring, from test_motor.c
	left_motor = init_motor(0,'A',66,67);
	right_motor = init_motor(0,'B',68, 69);
	//motor econder pos must be 1 or 2;
	//init both to 0;
	set_encoder_pos(1,0);
	set_encoder_pos(2,0);
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
	state_t last_state, new_state; // keep track of last state 
	while(get_state()!=EXITING){
		new_state = get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("\n");
			// printf(" θ(Pitch)|");
			// printf(" ψ (Yaw) |");
			// printf(" α (avg) |");
			// printf("    θ'   |");
			// printf("    ψ'   |");
			// printf("    α'   |");

			// printf("    ox   |");
			// printf("    oy   |");
			// printf("    theta   |");
            // printf("    SP   |");
            // printf("  PTERM  |");
            // printf("  ITERM  |");
            // printf("  DTERM  |");
            // printf("  MOTOR  |");
            printf("\n");
            printf("\r");
            // printf(" %6.2f  ",bot_state.pitch);
            // printf(" %6.2f  ",bot_state.yaw);
            // printf(" %6.2f  ",bot_state.alpha);
            // printf(" %6.2f  ",bot_state.pitchSpeed);
            // printf(" %6.2f  ",bot_state.yawSpeed);
            // printf(" %6.2f  ",bot_state.alphaSpeed);

            printf("\n");


		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
        // TODO: output PID and state variables
		if(new_state == RUNNING){	
			//Add Print stattements here, do not follow with /n
			// printf("%s  |", "Print");
			// printf("%s  |", "PID Values");
			// printf("%s  |", "Here");
			            printf("\r");
            printf("  %6.5f  ",bot_state.yaw);
            printf("  %6.5f  ",adc_PID.i);
            printf("  %6.5f  ",adc_PID.d);
            printf("  %6.2f  ", demo_data.kp);
            printf("  %6.4f  ",bot_state.delta_theta_gyro);
            printf("  %6.4f  ",bot_state.delta_theta);
            printf("  %6.4f  ",bot_state.delta_theta_gyro + bot_state.delta_theta);
            // printf(" ang %6.4f  ",BALANCE_ANG);
            // printf(" p %6.4f  ",bot_state.pitch);
            // printf("  %6.2f  ",bot_state.set_forward_vel_balance);
            // printf("  %6.2f  ",bot_state.set_forward_vel);
            // printf("  %6.2f  ",bot_state.set_forward_vel_balance);
            // printf("%d", path_idx);
            // printf("  %6.2f  ",bot_state.set_forward_vel_balance);
            // printf("  %6.3f  ",bot_state.fb_forward_vel);
            // printf("  %6.3f  ", bot_state.fb_angular_vel);
			fflush(stdout);
			

		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
} 

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* lcm_publish_loop(void* ptr){
	printf("into lcm func\n");
	while(get_state()!=EXITING){
		//publish lcm messages here
		pose_xyt_t_publish(lcm, ODOMETRY_CHANNEL, &pose_xyz);
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

