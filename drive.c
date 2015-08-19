 /*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/


/*
	drive.c
	This is a more advanced version of drive_basic which includes the following
	features in addition to normal driving around
	- Steering ackerman based on measured rover geometry.
	- Balancing on 2 wheels
	- automatic arranging of wheels based on how the user orients
	- the rover in balancing mode
	- drive up walls and balance on 2 wheels
*/


/********************************************
* 			Includes & Constants			*
*********************************************/
#include <robotics_cape.h>
#include "drive_config.h"
#include "balance_logging.h"


#define IMU_SAMPLE_RATE_HZ 200	// balancing filter and control loop speed
#define DT 0.005       		// 1/sample_rate
#define SERVO_HZ 50			// drive_stack frequency for driving around
#define THETA_MIX_TC  2   // time constant on filter

/************************************************************************
* 	core_mode_t
*
*	ANGLE: Only body angle theta and steering is controlled, this lets you
*	push the MiP around. 
*
*
************************************************************************/
typedef enum core_mode_t{  
	ANGLE,
	DRIVE
}core_mode_t;

/************************************************************************
* 	arm_state_t
*	ARMED or DISARMED to indicate if the balance controller is running
************************************************************************/
typedef enum arm_state_t{
	DISARMED,
	ARMED
}arm_state_t;

/************************************************************************
* 	orientation_t
*	possible orientations
************************************************************************/
typedef enum orientation_t {
	FLAT,
	LEFT_DOWN,
	RIGHT_DOWN,
	NOSE_DOWN,
	NOSE_UP
}orientation_t;

/************************************************************************
* 	input_mode_t
*	possible modes of user control
*	these are ordered such that DSM2 has highest priority
************************************************************************/
typedef enum input_mode_t {
	NONE,
	MAVLINK,
	BLUETOOTH,
	DSM2
}input_mode_t;

/************************************************************************
* 	drive_mode_t
*	possible modes of driving around
*  note that balancing is dealt with separately
************************************************************************/
typedef enum drive_mode_t {
	LANECHANGE,
	NORMAL_4W,
	CRAB,
	SPIN,
	BALANCE
}drive_mode_t;

/************************************************************************
* 	core_setpoint_t
*	setpoint for the balance controller
*	This is controlled by the drive stack and read by the balance core	
************************************************************************/
typedef struct core_setpoint_t{
	core_mode_t core_mode;	// see core_state_t declaration
	arm_state_t arm_state;	// see arm_state_t declaration
	float theta;			// body lean angle (rad)
	float gamma;			// body turn angle (rad)
	float gamma_dot;		// rate of change of turning	//leave in for future yaw control
}core_setpoint_t;

/************************************************************************
* 	core_state_t
*	contains workings of the drive stack and information about motor
*	and servo control
*	Should only be written to by the drive_stack thread	
*	also contains information for the balance controller
************************************************************************/
typedef struct core_state_t{
	// outputs to actuators
	float servos[4];
	float motors[4];
	// time when core_controller has finished a step
	uint64_t time_us; 
	// feedback loop to control theta body angle
	float K;
	float theta[3];	
	float theta_ref[3];
	float current_theta;
	float d_theta;
	float eTheta[3];
	// steering controller for gamma
	float gamma[3];
	float current_gamma;
	float d_gamma;
	float egamma[3];
	// outputs of balance and steering controllers
	float u[3];
	float current_u;
	float duty_split;
	// battery voltage for scaling motor inputs.
	float vBatt; 
	// orientations set by the orientation detection thread
	orientation_t poss_orientation;
	orientation_t orientation;
} core_state_t;

/************************************************************************
* 	Variables for complementary filter.
*	May create a new struct of comp_filter_t later.
************************************************************************/
	const float HP_CONST = THETA_MIX_TC/(THETA_MIX_TC + DT);
	const float LP_CONST = DT/(THETA_MIX_TC + DT);
	unsigned short gyro_fsr; //full scale range of gyro
	float gyro_to_rad_per_sec;
	mpudata_t mpu; //struct to read IMU data into 
	// Filter Initialization sampling
	float sum_ax, sum_ay, sum_az, sum_gx, sum_gy;
typedef struct comp_filter_t{
	int xAccel, yAccel, zAccel;
	float accLP_LD, accLP_RD, accLP_NU, accLP_ND;
	float yGyro, xGyro, gyroHP;
	float theta, theta_LD, theta_RD, theta_NU, theta_ND;
} comp_filter_t;

/************************************************************************
* 	user_interface_t
*	represents current command by the user which may be populated from 
*	DSM2, mavlink, bluetooth, or any other communication you like
************************************************************************/
typedef struct user_interface_t{
	// written by the last input watching thread to update
	input_mode_t input_mode;
	drive_mode_t drive_mode;
		
	// All sticks scaled from -1 to 1
	float drive_stick; 	// positive forward
	float turn_stick;	// positive to the right, CW yaw
	float throttle_stick;	// positive forward
	float kill_switch;
}user_interface_t;

// set with microsSinceEpoch() when the core starts
uint64_t core_start_time_us;

/************************************************************************
* 	Function declarations in this c file
*	also check out functions in balance_config.h & balance_logging.h		
************************************************************************/
// hardware interrupt routines
int balance_core();	// IMU interrupt routine

//threads
void* drive_stack(void* ptr);
void* balance_stack(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* dsm2_watcher(void* ptr);
void* orientation_detector(void* ptr);

// regular functions
int main();
int balance_core();
int wait_for_starting_condition();
int saturate_number(float* val, float min, float max);		
int saturate_number_limit(float* val, float limit);			
int on_pause_press();
int print_drive_mode(drive_mode_t mode);
int on_mode_release();
int blink_green();
int blink_red();
int zero_out_controller();
int disarm_controller();
int arm_controller();
int print_orientation(orientation_t orient);
int print_arm_state(arm_state_t arm_state);
int sample_imu();


/************************************************************************
* 	Global Variables				
************************************************************************/
drive_config_t config;
core_state_t cstate;
user_interface_t user_interface;
core_setpoint_t setpoint;
comp_filter_t cfilter;


/***********************************************************************
*	main()
*	start all the threads, and wait till something 
*	triggers a shut down (sleep unless exiting)
***********************************************************************/
int main(){
	// initialize cape hardware
	if(initialize_cape()<0){
		blink_red();
		return -1;
	}
	setRED(HIGH);
	setGRN(LOW);
	set_state(UNINITIALIZED);
	
	// set up button handlers first
	// so user can exit by holding pause
	set_pause_pressed_func(&on_pause_press);
	set_mode_unpressed_func(&on_mode_release);
	
	// load data from disk.
	if(load_config(&config)==-1){
		printf("aborting, config file error\n");
		return -1;
	}
	
	// start a thread to slowly sample battery 
	pthread_t  battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	} 
	
	 // start listening for RC control from dsm2 radio
	if(config.enable_dsm2){
		if(initialize_dsm2()<0){
				printf("failed to start DSM2\n");
		}
		else{
			pthread_t  dsm2_thread;
			pthread_create(&dsm2_thread, NULL, dsm2_watcher, (void*) NULL);
		}
	} 

	// start logging thread if enabled
	if(config.enable_logging){
		if(start_log(IMU_SAMPLE_RATE_HZ, &cstate.time_us)<0){
			printf("failed to start log\n");
		}
		else{
			// start new thread to write the file occasionally
			pthread_t  logging_thread;
			pthread_create(&logging_thread, NULL, log_writer, (void*) NULL);
		}
	}
	
	// Finally start the real-time interrupt driven control thread
	// start IMU with equilibrium set with FLAT orientation 
	
	signed char imu_orientation[9] = ORIENTATION_FLAT;		
	if(initialize_imu(IMU_SAMPLE_RATE_HZ, imu_orientation)){
		// can't talk to IMU, all hope is lost
		// blink red until the user exits
		blink_red();
		return -1;
	};
	
	//start the interrupt handler this should be the last 
	//step in initialization to make sure other setup functions don't interfere
	printf("starting core IMU interrupt\n");
	core_start_time_us = microsSinceEpoch();
	set_imu_interrupt_func(&balance_core);
	
	// start balance stack to control setpoints
	pthread_t  balance_stack_thread;
	pthread_create(&balance_stack_thread, NULL, balance_stack, (void*) NULL);
	
	// start the orientation detection thread in the background
	pthread_t orientation_thread;
	pthread_create(&orientation_thread, NULL, orientation_detector, (void*) NULL);
	
	// this thread is in charge of arming and managing the core
	pthread_t  drive_stack_thread;
	pthread_create(&drive_stack_thread, NULL, drive_stack, (void*) NULL);
	
	// all threads have started, off we go
	set_state(RUNNING);
	setRED(LOW);
	setGRN(HIGH);
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	cleanup_cape(); // always end with cleanup to shut down cleanly
	return 0;
}

/***********************************************************************
*	drive_stack run at 50Hz
*	This is the medium between the user_interface struct and the 
*	physical servos and motors
************************************************************************/
void* drive_stack(void* ptr){
	int i; // general purpose counter for for loops
	float net_drive, net_turn, net_torque_split, net_turn_rad, R, net_turn_outer;

	// exiting condition is checked inside the switch case instead
	while(1){
		switch (get_state()){
		case EXITING:
			return NULL;
			
		case PAUSED:
			disable_motors();
			// not much to do if paused!
			break;
			
		// when running, drive_stack checks if an input mode
		// like mavlink, DSM2, or bluetooth is enabled
		// and moves the servos and motors corresponding to 
		// user input and current controller mode
		case RUNNING:
			enable_motors();
			// now send input to servos and motors based on drive mode and UI
			//  motor & servo orientation: 
			//	left	1 2   right
			// 			4 3	
			
			// if the orientation is flat, drive around on 4 wheels
			if(cstate.poss_orientation==FLAT){
				if(user_interface.input_mode == NONE){
					cstate.servos[0]=config.serv1_center-config.turn_straight;
					cstate.servos[1]=config.serv2_center+config.turn_straight;
					cstate.servos[2]=config.serv3_center-config.turn_straight;
					cstate.servos[3]=config.serv4_center+config.turn_straight;
					for (i=1; i<=4; i++){
						send_servo_pulse_normalized(i,cstate.servos[i-1]);
					}
					disable_motors();
					break;
				}
				switch(user_interface.drive_mode){
				case LANECHANGE:		// lane change maneuver
					net_drive = user_interface.drive_stick*config.motor_max;
					net_turn = user_interface.turn_stick*(0.5-config.turn_straight);
					cstate.motors[0]=net_drive*config.mot1_polarity;
					cstate.motors[1]=net_drive*config.mot2_polarity;
					cstate.motors[2]=net_drive*config.mot3_polarity;
					cstate.motors[3]=net_drive*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center-config.turn_straight+net_turn;
					cstate.servos[1]=config.serv2_center+config.turn_straight+net_turn;
					cstate.servos[2]=config.serv3_center-config.turn_straight+net_turn;
					cstate.servos[3]=config.serv4_center+config.turn_straight+net_turn;
					break;
					
				case NORMAL_4W:		// Normal 4W Steering
				net_drive = user_interface.drive_stick*config.motor_max;
				net_torque_split = user_interface.turn_stick*config.torque_vec_const*net_drive;
				net_turn = user_interface.turn_stick*config.normal_turn_range_straight;	//negative left, positive right
				if(net_turn<0.001 && net_turn>-0.001){
					net_turn = 0.001;
				}
				net_turn_rad = (config.servo_range_rad - (1-fabsf(net_turn))*config.servo_range_rad);	//convert to radians 
				R = config.wheelbase_over2/tanf(net_turn_rad) + config.track_width_over2;		//R=distance from center of wheelbase to center of curvature
				net_turn_outer = atanf(config.wheelbase_over2/(R+config.track_width_over2))/config.servo_range_rad;	//turn angle of outer wheel normalized
				cstate.motors[0]=(net_drive+net_torque_split)*config.mot1_polarity;
				cstate.motors[1]=(net_drive+net_torque_split)*config.mot2_polarity;
				cstate.motors[2]=(net_drive-net_torque_split)*config.mot3_polarity;
				cstate.motors[3]=(net_drive-net_torque_split)*config.mot4_polarity;
				if(net_turn<0.0){
					cstate.servos[0]=config.serv1_center-config.turn_straight+net_turn;
					cstate.servos[1]=config.serv2_center+config.turn_straight-net_turn_outer;
					cstate.servos[2]=config.serv3_center-config.turn_straight+net_turn_outer;
					cstate.servos[3]=config.serv4_center+config.turn_straight-net_turn;
				}
				else {
					cstate.servos[0]=config.serv1_center-config.turn_straight+net_turn_outer;
					cstate.servos[1]=config.serv2_center+config.turn_straight+net_turn;
					cstate.servos[2]=config.serv3_center-config.turn_straight-net_turn;
					cstate.servos[3]=config.serv4_center+config.turn_straight-net_turn_outer;
				}
				break;
				
				// crab, turn all wheels sideways and drive
				case CRAB:
				net_drive = user_interface.drive_stick*config.motor_max;
				net_torque_split = user_interface.turn_stick*config.torque_vec_const*net_drive;
				net_turn = user_interface.turn_stick*config.normal_turn_range_crab;	//negative left, positive right
				if(net_turn<0.001 && net_turn>-0.001){		// avoid singularity at R=0
					net_turn = 0.001;
				}
				net_turn_rad = (config.servo_range_rad - (1-fabsf(net_turn))*config.servo_range_rad);	//convert to radians 
				R = config.track_width_over2/tanf(net_turn_rad) + config.wheelbase_over2;		//R=distance from center of wheelbase to center of curvature
				net_turn_outer = atanf(config.track_width_over2/(R+config.wheelbase_over2))/config.servo_range_rad;	//turn angle of outer wheel normalized
				cstate.motors[0]=(net_drive+net_torque_split)*config.mot1_polarity;
				cstate.motors[1]=-(net_drive+net_torque_split)*config.mot2_polarity;
				cstate.motors[2]=(net_drive-net_torque_split)*config.mot3_polarity;
				cstate.motors[3]=-(net_drive-net_torque_split)*config.mot4_polarity;
				if(net_turn<0.0){
					cstate.servos[0]=config.serv1_center+config.turn_crab+0.02-net_turn;
					cstate.servos[1]=config.serv2_center-config.turn_crab+0.01+net_turn;
					cstate.servos[2]=config.serv3_center+config.turn_crab-0.03-net_turn_outer;
					cstate.servos[3]=config.serv4_center-config.turn_crab-0.02+net_turn_outer;
				}
				else {
					cstate.servos[0]=config.serv1_center+config.turn_crab+0.02-net_turn_outer;
					cstate.servos[1]=config.serv2_center-config.turn_crab+0.01+net_turn_outer;
					cstate.servos[2]=config.serv3_center+config.turn_crab-0.03+net_turn;
					cstate.servos[3]=config.serv4_center-config.turn_crab-0.02-net_turn;
				}
					break;
					
				case SPIN:
					net_drive = user_interface.turn_stick*config.motor_max;
					cstate.motors[0]=net_drive*config.mot1_polarity;
					cstate.motors[1]=-net_drive*config.mot2_polarity;  
					cstate.motors[2]=-net_drive*config.mot3_polarity;
					cstate.motors[3]=net_drive*config.mot4_polarity;
					cstate.servos[0]=config.serv1_center+config.turn_spin;
					cstate.servos[1]=config.serv2_center-config.turn_spin;
					cstate.servos[2]=config.serv3_center+config.turn_spin;
					cstate.servos[3]=config.serv4_center-config.turn_spin;
					break;
					
				default:
					printf("unknown drive_mode\n");
					disable_motors();
					for (i=1; i<=4; i++){
						cstate.motors[i-1]=0;
						cstate.servos[i-1]=0.5;
					}
					break;
				}// end of switch(drive_mode)
				
				// send pulses to servos and drive motors
				for (i=1; i<=4; i++){
					saturate_number(&cstate.servos[i-1],config.turn_min,config.turn_max);
					saturate_number(&cstate.motors[i-1],-config.motor_max,config.motor_max);
					set_motor(i,cstate.motors[i-1]);
					send_servo_pulse_normalized(i,cstate.servos[i-1]);
				}
			}
			// if not flat, orient the wheels for balancing
			else{
				switch(cstate.poss_orientation){
				case LEFT_DOWN:						//left side of the vehicle, not the cape
					cstate.servos[0]=config.serv1_center+config.turn_crab;
					cstate.servos[1]=config.serv2_center-config.turn_spin;
					cstate.servos[2]=config.serv3_center+config.turn_spin;
					cstate.servos[3]=config.serv4_center-config.turn_crab;
					break;
				case RIGHT_DOWN:
					cstate.servos[0]=config.serv1_center+config.turn_spin;
					cstate.servos[1]=config.serv2_center-config.turn_crab;
					cstate.servos[2]=config.serv3_center+config.turn_crab;
					cstate.servos[3]=config.serv4_center-config.turn_spin;
					break;
				case NOSE_DOWN:
					cstate.servos[0]=config.serv1_center-config.turn_straight;
					cstate.servos[1]=config.serv2_center+config.turn_straight;
					cstate.servos[2]=config.serv3_center-config.turn_spin;
					cstate.servos[3]=config.serv4_center+config.turn_spin;
					break;
				case NOSE_UP:
					cstate.servos[0]=config.serv1_center-config.turn_spin;
					cstate.servos[1]=config.serv2_center+config.turn_spin;
					cstate.servos[2]=config.serv3_center-config.turn_straight;
					cstate.servos[3]=config.serv4_center+config.turn_straight;
					break;
				default: // shouldn't get here
					break;
				}
				// send pulses to servos
				for (i=1; i<=4; i++){
					saturate_number(&cstate.servos[i-1],config.turn_min,config.turn_max);
					send_servo_pulse_normalized(i,cstate.servos[i-1]);
				}
			}
	
		default:
			break;
		} // end of switch get_state()
		
		// run about as fast as the core itself 
		usleep(1000000/SERVO_HZ); 
	}
	return NULL;
}

/***********************************************************************
*	print_drive_mode(drive_mode_t mode)
*	prints a readable text name of one of the 4 drive modes
************************************************************************/
int print_drive_mode(drive_mode_t mode){
	switch(mode){
		case LANECHANGE:
			printf("drive_mode: LANECHANGE\n");
			break;
			
		case NORMAL_4W:
			printf("drive_mode: NORMAL_4W\n");
			break;
			
		case CRAB:
			printf("drive_mode: CRAB\n");
			break;
			
		case SPIN:
			printf("drive_mode: SPIN\n");
			break;
			
		default:
			printf("unknown drive_mode\n");
			return -1;
	}
	return 0;
	
}

/************************************************************************
* 	int print_arm_state(arm_state_t arm_state)
*	prints a human readable version of the arm state
************************************************************************/
int print_arm_state(arm_state_t arm_state){
	switch(arm_state){
	case ARMED:
		printf("ARMED");
		break;
	case DISARMED:
		printf("DISARMED");
		break;
	default:
		printf("unknown arm_state");
		return -1;
		break;
	}
	return 0;
}

/***********************************************************************
*	saturate_num(float val, float min, float max) takes 3 arguments
*	bounds val to +- limit
*	return one if saturation occurred, otherwise zero
************************************************************************/
int saturate_number(float* val, float min, float max){
	if(*val>max){
		*val = max;
		return 1;
	}
	else if(*val<min){	
		*val = min;
		return 1;
	}
	return 0;
}

/************************************************************************
* 	int print_orientation(orientation_t orient)
*	prints a human readable version of the orientation enum
************************************************************************/
int print_orientation(orientation_t orient){
	switch(orient){
	case FLAT:
		printf("   FLAT  ");
		break;
	case LEFT_DOWN:
		printf("LEFT_DOWN");
		break;
	case RIGHT_DOWN:
		printf("RIGHT_DOWN");
		break;
	case NOSE_DOWN:
		printf(" NOSE_DOWN");
		break;
	case NOSE_UP:
		printf(" NOSE_UP  ");
		break;
	default:
		printf("unknown orientation");
		return -1;
		break;
	}
	return 0;
}

/************************************************************************
* 	void* orientation_detector(void* ptr) run at 50Hz
*	independent thread that monitors the imu data and determines
*	a possible orientation quickly and a definite orientation more 
* 	slowly with more certainty. 
************************************************************************/
void* orientation_detector(void* ptr){
	// local copies of roll and pitch read from IMU
	float roll, pitch;
	int counter = 0;
	
	// local orientation based on simple imu sample
	orientation_t immediate_orientation;
	
	// counter limits for certain orientation and possible orientation
	int counter_limit=lrint(config.det_time*config.orientation_rate);
	int counter_poss_limit=lrint(config.det_poss_time*config.orientation_rate);
	
	// run until the rest of the program closes
	while(get_state()!=EXITING){
		// make local copies of roll and pitch from IMU
		roll = mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE;
		pitch = mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE;
		
		// check for FLAT
		//from cape facing up
		if(fabs(roll)<(config.orient_tolerance)&& \
				fabs(pitch)<(config.orient_tolerance)){
			if (immediate_orientation != FLAT){
				counter = 0;
				immediate_orientation = FLAT;
			}
			counter ++;
		}
		//from cape facing down
		else if(fabs(roll)>(90+config.orient_tolerance)&& fabs(pitch)<(config.orient_tolerance)){
			if (immediate_orientation != FLAT){
				counter = 0;
				immediate_orientation = FLAT;
			}
			counter ++;
		}
	
		// check for NOSE_DOWN
		else if(pitch<90 && pitch>(90-config.orient_tolerance)){
			if (immediate_orientation != NOSE_DOWN){
				counter = 0;
				immediate_orientation = NOSE_DOWN;
			}
			counter ++;
		}
		
		// check for NOSE_UP
		else if(pitch>-90 && pitch<(-90+config.orient_tolerance)){
			if (immediate_orientation != NOSE_UP){
				counter = 0;
				immediate_orientation = NOSE_UP;
			}
			counter ++;
		}
		
		// check for RIGHT_DOWN
		//from cape facing up
		else if(roll>-90 && roll<(-90+config.orient_tolerance)){
			if (immediate_orientation != RIGHT_DOWN){
				counter = 0;
				immediate_orientation = RIGHT_DOWN;
			}
			counter ++;
		}
		//from cape facing down
		else if(roll<-90 && roll>(-90-config.orient_tolerance)){
			if (immediate_orientation != RIGHT_DOWN){
				counter = 0;
				immediate_orientation = RIGHT_DOWN;
			}
			counter ++;
		}

		// check for LEFT_DOWN
		//from cape facing up
		else if(roll<90 && roll>(90-config.orient_tolerance)){
			if (immediate_orientation != LEFT_DOWN){
				counter = 0;
				immediate_orientation = LEFT_DOWN;
			}
			counter ++;
		}
		//from cape facing down
		else if(roll>90 && roll<(90+config.orient_tolerance)){
			if (immediate_orientation != LEFT_DOWN){
				counter = 0;
				immediate_orientation = LEFT_DOWN;
			}
			counter ++;
		}
		
		// check for possible counter timeout
		if(counter>=counter_poss_limit){
			cstate.poss_orientation = immediate_orientation;
		}
		
		// check for certain counter timeout
		if(counter>=counter_limit){
			cstate.orientation = immediate_orientation;
			// to prevent the counter from overflowing, stop it here
			counter = counter_limit;
		}
		
		// sleep for roughly enough time to keep the sample rate
		usleep(1000000/config.orientation_rate);
	}
	
	return NULL;
}

/***********************************************************************
*	balance_stack run at 200Hz
*	This is the medium between the user_interface and setpoint structs.
*	dsm2, bluetooth, and mavlink threads may be unreliable and shouldn't
*	touch the controller setpoint directly. balance_stack and balance_core
*	should be the only things touching the controller setpoint.
************************************************************************/
void* balance_stack(void* ptr){
	
	// wait for IMU to settle
	disarm_controller();
	usleep(1000000);
	usleep(1000000);
	usleep(500000);
	set_state(RUNNING);
	setpoint.core_mode = ANGLE;		// start in balance mode
	setRED(LOW);
	setGRN(HIGH);
	
	// exiting condition is checked inside the switch case instead
	while(1){
		switch (get_state()){
		case EXITING:
			return NULL;
			
		case PAUSED:
			// not much to do if paused!
			break;
			
		
		// when running, balance_stack checks if an input mode
		// like mavlink, DSM2, or bluetooth is enabled
		// and moves the controller setpoints corresponding to 
		// user input and current controller mode
		case RUNNING:
			if(setpoint.arm_state==DISARMED){
				// check if the user has picked MIP upright before starting again
				wait_for_starting_condition();
				// user may have pressed the pause button or shut down while waiting
				// check before continuing
				if(get_state()!=RUNNING){
					break;
				}
				// write a blank log entry to mark this time
				log_blank_entry();
				// read config each time it's picked up to recognize new
				// settings user may have changed
				// only actually reads from disk if the file was modified
				load_config(&config);
				zero_out_controller();						
				arm_controller();
			}
		
		
			if(user_interface.input_mode == NONE){	
				// no user input, just keep the controller setpoint at zero
				setpoint.theta = 0;		///////////made change here//////////////////////////////////////////
				
				setpoint.gamma = 0;		//leave in for future yaw control
				setpoint.gamma_dot = 0;		//leave in for future yaw control
				break;
			} 
			if(setpoint.core_mode == ANGLE){
				// in angle mode, scale user input from -1 to 1 to
				// the minimum and maximum theta reference angles
				//setpoint.theta = config.theta_ref_max*(saturate_number_limit(&user_interface.drive_stick,1));
				setpoint.theta = -user_interface.drive_stick*.5;	// damp out drive_stick so easy to drive 
					if(setpoint.theta >= config.theta_ref_max){
						setpoint.theta = config.theta_ref_max;
					}
					if(setpoint.theta <= -config.theta_ref_max){
						setpoint.theta = -config.theta_ref_max;
					}
					setpoint.gamma = user_interface.turn_stick;
					setpoint.gamma_dot = cstate.d_gamma;
					if(setpoint.gamma_dot >= config.max_turn_rate){
						setpoint.gamma_dot = config.max_turn_rate;
					}
			}
			break; // end of RUNNING case
	
		default:
			break;
		} // end of switch get_state()
		
		// run about as fast as the core itself 
		usleep(1000000/IMU_SAMPLE_RATE_HZ); 
	}
	return NULL;
}

/************************************************************************
* 	int balance_core() (IMU interrupt function run at 200Hz)
************************************************************************/
int balance_core(){
	// local variables only in memory scope of balance_core
	static int Dz_saturation_counter = 0; 
	float compensated_Dz_output = 0;
	float dutyL = 0;
	float dutyR = 0;
	static log_entry_t new_log_entry;
	
	// if an IMU packet read failed, ignore and just return
	// the mpu9150_read function may print it's own warnings
	if (mpu9150_read(&mpu) != 0){
		return -1;
	}
	
	// start the complementary filter
	// read the gyro full-scale range
	mpu_get_gyro_fsr(&gyro_fsr);
	gyro_to_rad_per_sec = gyro_fsr*DEG_TO_RAD/32768;
	
	/***********************************************************************
	*	COMPLEMENTARY_FILTER
	*	get raw accelerometer and gyro readings and compute filter 
	*	values based on orientation
	************************************************************************/
	//raw integer accelerometer and gyro values
	cfilter.xAccel = mpu.rawAccel[VEC3_X]; 										 // note DMP reverses x and Z
	cfilter.zAccel = mpu.rawAccel[VEC3_Z]; 
	cfilter.yAccel = mpu.rawAccel[VEC3_Y];
	cfilter.yGyro = -(mpu.rawGyro[VEC3_Y]);
	cfilter.xGyro = -(mpu.rawGyro[VEC3_X]);
															 // initialize theta with accelerometer
	
	switch(cstate.orientation){
		case FLAT: {
		// Not much to do if FLAT!
		// initialize theta with accelerometer
		cfilter.accLP_LD = atan2(cfilter.zAccel, cfilter.yAccel);
		cfilter.theta_LD = cfilter.accLP_LD;
		cfilter.accLP_RD = atan2(cfilter.zAccel, -cfilter.yAccel);
		cfilter.theta_RD = cfilter.accLP_RD;
		cfilter.accLP_NU = atan2(cfilter.zAccel, -cfilter.xAccel);
		cfilter.theta_NU = cfilter.accLP_NU;
		cfilter.accLP_ND = atan2(cfilter.zAccel, cfilter.xAccel);
		cfilter.theta_ND = cfilter.accLP_ND; 
		cfilter.gyroHP = 0;		// zero out gyro
		cfilter.theta = 1.57;	// "zero" out theta (theta=1.57 when flat)
		
		break;
		}
		case LEFT_DOWN: {
			cfilter.accLP_LD = cfilter.accLP_LD \
							+ LP_CONST * (atan2(cfilter.zAccel,cfilter.yAccel) - cfilter.accLP_LD); 	 // first order filter LEFT_DOWN
			cfilter.gyroHP = HP_CONST*(cfilter.gyroHP + (DT*cfilter.xGyro*gyro_to_rad_per_sec)); // cfilter.gyroHP starts at zero
			cfilter.theta = cfilter.gyroHP + cfilter.accLP_LD + config.theta_tilt_roll; // tilt angle of BBB at equilibrium about roll axis
			cstate.K = config.KD_roll;											 // constant to adjust gain
			break;
		}
		case RIGHT_DOWN: {
			cfilter.accLP_RD = cfilter.accLP_RD \
							+ LP_CONST * (atan2(cfilter.zAccel,-cfilter.yAccel) - cfilter.accLP_RD);  // first order filter RIGHT_DOWN
			cfilter.gyroHP = HP_CONST*(cfilter.gyroHP + (DT*-cfilter.xGyro*gyro_to_rad_per_sec));  
			cfilter.theta = cfilter.gyroHP + cfilter.accLP_RD + config.theta_tilt_roll;					 
			cstate.K = config.KD_roll;		
			break;
		}
		case NOSE_UP: {
			cfilter.accLP_NU = cfilter.accLP_NU 
							+ LP_CONST * (atan2(cfilter.zAccel,-cfilter.xAccel) - cfilter.accLP_NU);  // first order filter NOSE_UP
			cfilter.gyroHP = HP_CONST*(cfilter.gyroHP + (DT*cfilter.yGyro*gyro_to_rad_per_sec));  
			cfilter.theta = cfilter.gyroHP + cfilter.accLP_NU + config.theta_tilt_pitch;			 
			cstate.K = config.KD_pitch;	
			break;
		}
		case NOSE_DOWN: {
			cfilter.accLP_ND = cfilter.accLP_ND \
							+ LP_CONST * (atan2(cfilter.zAccel,cfilter.xAccel) - cfilter.accLP_ND); 	 // first order filter NOSE_DOWN
			cfilter.gyroHP = HP_CONST*(cfilter.gyroHP + (DT*-cfilter.yGyro*gyro_to_rad_per_sec));  
			cfilter.theta = cfilter.gyroHP + cfilter.accLP_ND + config.theta_tilt_pitch;					 
			cstate.K = config.KD_pitch;	
			break;
		}
	}		// end switch cstate.orientation
	
		/***********************************************************************
	*	STATE_ESTIMATION
	*	read sensors and compute the state regardless of if the controller
	*	is ARMED or DISARMED
	************************************************************************/
	// angle theta is negative in the direction of forward tip (BBB facing away from user)
	// add mounting angle of BBB (theta_offset)
	cstate.theta[2] = cstate.theta[1]; cstate.theta[1] = cstate.theta[0];
	cstate.theta[0] = cfilter.theta;  		  
	cstate.current_theta = cstate.theta[0]; 
	
	// body turning estimation
	cstate.gamma[2] = cstate.gamma[1]; cstate.gamma[1] = cstate.gamma[0];
	cstate.gamma[0]= mpu.fusedEuler[VEC3_Z];
	cstate.d_gamma = (cstate.gamma[0]-cstate.gamma[1])/DT;
	cstate.current_gamma = cstate.gamma[0];
	
	/***********************************************************************
	*	Control based on the robotics_library defined state variable
	*	PAUSED: make sure the controller stays DISARMED
	*	RUNNING: Normal operation of controller.
	*		- check for tipover
	*		- wait for MiP to be within config.start_angle of upright
	*		- choose mode from setpoint.core_mode
	*		- evaluate difference equation and check saturation
	*		- actuate motors
	************************************************************************/
	switch (get_state()){
	
	// make sure things are off if program is closing
	case EXITING:
		disable_motors();
		return 0;
	
	// if the controller is somehow still ARMED, disarm it
	case PAUSED:
		if(setpoint.arm_state==ARMED){
			disarm_controller();
		}
		break;
	
	// normal operating mode
	case RUNNING:
		// exit if the controller was not armed properly
		if(setpoint.arm_state==DISARMED){
			return 0;
		}
		
		if(setpoint.core_mode==ANGLE){
		// check for a tip over before anything else
		if(fabs(cstate.current_theta)>config.tip_angle){	
			disarm_controller();
			//printf("tip detected \n");
			break;
		}
		
		// evaluate controller Dz
		cstate.eTheta[2] = cstate.eTheta[1]; 
		cstate.eTheta[1] = cstate.eTheta[0];
		cstate.eTheta[0] = setpoint.theta - cstate.current_theta;
		cstate.u[2] = cstate.u[1];
		cstate.u[1] = cstate.u[0];
		cstate.u[0] = \
				cstate.K * (config.numDz_0 * cstate.eTheta[0]	\
						+	config.numDz_1 * cstate.eTheta[1] 	\
						+	config.numDz_2 * cstate.eTheta[2])	\
						-  (config.denDz_1 * cstate.u[1] 		\
						+	config.denDz_2 * cstate.u[2]); 		
		
		// check saturation of inner loop knowing that right after
		// this control will be scaled by battery voltage
		if(saturate_number_limit(&cstate.u[0], config.v_nominal/cstate.vBatt)){
			Dz_saturation_counter ++;
			if(Dz_saturation_counter > IMU_SAMPLE_RATE_HZ/4){
				//printf("inner loop controller saturated\n");
				disarm_controller();
				Dz_saturation_counter = 0;
				break;
			}
		}
		else{
			Dz_saturation_counter = 0;
		}
		cstate.current_u = cstate.u[0];
		
		// scale output to compensate for battery charge level
		compensated_Dz_output = cstate.u[0] \
				* (config.v_nominal / cstate.vBatt);
		}		// end if statement for calculating balance controller
		
		//steering controller
		// move the controller set points based on user input
		setpoint.gamma += setpoint.gamma_dot * DT;
		cstate.egamma[1] = cstate.egamma[0];
		cstate.egamma[0] = setpoint.gamma - cstate.current_gamma;
		//cstate.duty_split = config.KP_steer*(cstate.egamma[0] + config.KD_steer*(cstate.egamma[0]-cstate.egamma[1]));
		cstate.duty_split = -user_interface.turn_stick*.5;
		
		// if the steering input would saturate a motor, reduce			
		// the steering input to prevent compromising balance input
		if(fabs(compensated_Dz_output)+fabs(cstate.duty_split) > 1){	
			if(cstate.duty_split > 0){
				cstate.duty_split = 1-fabs(compensated_Dz_output);
			}
			else cstate.duty_split = -(1-fabs(compensated_Dz_output));
		}  
		
		if(user_interface.throttle_stick > 0){	// cease to calculate balance controller
		setpoint.core_mode = DRIVE;
		compensated_Dz_output = 0;
		}
		if(user_interface.throttle_stick < 0){	// calculate balance controller again
		setpoint.core_mode = ANGLE;
		}
		
		// add Dz balance controller and steering control
		dutyL  = compensated_Dz_output + cstate.duty_split; 
		dutyR = compensated_Dz_output - cstate.duty_split;	
		
		// send to motors
		// one motor is flipped on chassis so reverse duty to L	include  
		if(cstate.orientation == LEFT_DOWN){
			set_motor(4,-dutyL); 				
			set_motor(1,dutyR);
		} 
		else if(cstate.orientation == RIGHT_DOWN){
			set_motor(2,-dutyL); 				
			set_motor(3,dutyR);
		}
		else if(cstate.orientation == NOSE_UP){
			set_motor(3,-dutyL); 				
			set_motor(4,dutyR);
		}
		else if(cstate.orientation == NOSE_DOWN){
			set_motor(1,-dutyL); 				
			set_motor(2,dutyR);
		}
		cstate.time_us = microsSinceEpoch();
		
		// pass new information to the log with add_to_buffer
		// this only puts information in memory, doesn't
		// write to disk immediately
		if(config.enable_logging){											//????????????????????????
			new_log_entry.time_us	= cstate.time_us-core_start_time_us;
			new_log_entry.theta		= cstate.current_theta;
			new_log_entry.theta_ref	= setpoint.theta; 
			new_log_entry.u 		= cstate.current_u;
			add_to_buffer(new_log_entry);
		}
		break;
		
		// end of normal balancing routine
		// last_state will be updated beginning of next interrupt
		break;
		
		default:
			break; // nothing to do if UNINITIALIZED
	}
	return 0;	
}

/************************************************************************
* 	sample_imu()
*	complementary filter initialization sampling
************************************************************************/
int sample_imu(){
	if (mpu9150_read(&mpu) == 0) {
		sum_ax += mpu.rawAccel[VEC3_X];
		sum_az += mpu.rawAccel[VEC3_Z];
		sum_ay += mpu.rawAccel[VEC3_Y];
		sum_gy += mpu.rawGyro[VEC3_Y];
	}
	return 0; 
}

/************************************************************************
* 	zero_out_controller()
*	clear the controller state and setpoint
*	especially should be called before swapping state to RUNNING
*	keep current_theta and vbatt since they may be used by 
*	other threads
************************************************************************/
int zero_out_controller(){
	// wipe setpoint
	setpoint.gamma = 0;		//leave in for future yaw control
	setpoint.theta = 0;		//////////////////made change here///////////////////////////////////////////////////
	cstate.u[0] = 0;
	cstate.u[1] = 0;
	cstate.u[2] = 0;
	cstate.theta_ref[0] = 0;
	cstate.theta_ref[1] = 0;
	cstate.theta_ref[2] = 0;
	cstate.eTheta[0] = 0;
	cstate.eTheta[1] = 0;
	cstate.eTheta[2] = 0;
	cstate.egamma[0] = 0;			//leave in for future yaw control
	cstate.egamma[1] = 0;
	cstate.egamma[2] = 0;
	
	
	return 0;
}

/************************************************************************
* 	disarm_controller()
*		- disable motors
*		- set the setpoint.core_mode to DISARMED to stop the controller
************************************************************************/
int disarm_controller(){
	disable_motors();
	setpoint.arm_state = DISARMED;
	return 0;
}

/************************************************************************
* 	arm_controller()
*		- zero out the controller
*		- set the setpoint.armed_state to ARMED
*		- enable motors
************************************************************************/
int arm_controller(){
	zero_out_controller();
	setpoint.arm_state = ARMED;
	enable_motors();
	return 0;
}

/***********************************************************************
*	saturate_num_limit(float val, float limit) takes two arguments
*	bounds val to limit
*	return one if saturation occurred, otherwise zero
************************************************************************/
int saturate_number_limit(float* val, float limit){
	if(*val>limit){
		*val = limit;
		return 1;
	}
	else if(*val<-limit){	
		*val = -limit;
		return 1;
	}
	return 0;
}

/***********************************************************************
*	wait_for_starting_condition()
*	wait for Rover to be held upright long enough to begin
************************************************************************/
int wait_for_starting_condition(){
	int checks = 0;
	
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(config.start_delay*check_hz);
	int wait_us = (1000000)/check_hz; 

	while(get_state()!=EXITING){
		// if within range, start counting
		if(fabs(cstate.current_theta)<config.start_angle){	
			checks++;
			// waited long enough, return
			if(checks>=checks_needed) return 0;
		}
		// fell out of range, restart counter
		else checks = 0;
		usleep(wait_us);
	}
	return 0;
}

/***********************************************************************
*	battery_checker() run every 3 seconds
*	super slow loop checking battery voltage
************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(get_state()!=EXITING){
		new_v = getBattVoltage();
		// check if there is a bad reading
		if (new_v>9.0 || new_v<5.0){
			// printf("problem reading battery\n");
			// use nominal for now
			new_v = config.v_nominal;
		}
		cstate.vBatt = new_v;
		usleep(1000000);
		usleep(1000000);
		usleep(1000000);
	}
	return NULL;
}

 /***********************************************************************
*	printf_loop() 
*	prints diagnostics to console
*   this only gets started if executing from terminal
************************************************************************/
/* void* printf_loop(void* ptr){
	// keep track of last global state variable
	state_t last_state;
	drive_mode_t last_drive_mode = user_interface.drive_mode;
	int print_header_flag = 1;
	print_drive_mode(last_drive_mode);
	
	while(1){
		// check if this is the first time since being paused
		if(get_state()==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			print_header_flag=1;
		}
		else if(get_state()==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause button again to start.\n");
		}
		if(user_interface.drive_mode != last_drive_mode){
			printf("\n\n");
			print_drive_mode(user_interface.drive_mode);
			print_header_flag=1;
		}
		last_state = get_state();
		last_drive_mode = user_interface.drive_mode;
		// decide what to print or exit
		switch (get_state()){	
		case RUNNING: { // show all the things
			if(print_header_flag){
				printf("    motors              servos  \n");
				printf(" 1   2   3   4       1   2   3   4\n");
				print_header_flag=0;
			}
			printf("\r");
			printf("%0.1f ", cstate.motors[0]);
			printf("%0.1f ", cstate.motors[1]);
			printf("%0.1f ", cstate.motors[2]);
			printf("%0.1f   ", cstate.motors[3]);
			printf("%0.2f ", cstate.servos[0]);
			printf("%0.2f ", cstate.servos[1]);
			printf("%.2f ", cstate.servos[2]);
			printf("%.2f ", cstate.servos[3]);
		
			if(setpoint.arm_state == ARMED)
				printf(" ARMED");
			else
				printf("DISARMED");
			print_orientation(cstate.orientation);
			printf("   "); // clear remaining characters
			fflush(stdout);
			break;
			}
		case PAUSED: {
			break;
			}
		case EXITING:{
			return NULL;
			}
		default: {
			break; // this is only for UNINITIALIZED state
			}
		}
		usleep(200000);
	}
	return NULL;
} */  

/***********************************************************************
*	printf_loop() run at 5 Hz
*	prints diagnostics to console
*   this only gets started if executing from terminal
************************************************************************/
void* printf_loop(void* ptr){
	state_t last_state, new_state; // keep track of last state 
	while(1){
		new_state = get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf(" theta t_ref d_split  d_stick   thr   u    orient   arm_state \n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_state = new_state;
		
		// decide what to print or exit
		switch (new_state){	
		case RUNNING: { // show all the things
			printf("\r");
			printf(" %0.2f ", cstate.current_theta);
			printf(" %0.2f ", setpoint.theta);
			printf(" %0.2f ", cstate.duty_split);
			printf(" %0.2f ", user_interface.drive_stick);
			printf(" %0.2f ", user_interface.throttle_stick);
			//printf(" %0.2f ", cstate.current_gamma);
			printf(" %0.2f ", cstate.current_u);
			print_orientation(cstate.orientation);
			
			if(user_interface.input_mode == DSM2)
				printf(" DSM2 ");
			else
				printf("NONE");
			printf("   ");
			
			if(setpoint.core_mode == ANGLE)
				printf(" ANGLE ");
			
			printf("   "); 
			
			if(setpoint.arm_state == ARMED)
				printf(" ARMED ");
			else
				printf("DISARMED");
			printf("   "); // clear remaining characters
			fflush(stdout);
			break;
			}
		case PAUSED: { // only print theta when paused
			printf("\rtheta: %0.2f   ", cstate.current_theta);
			break;
			}
		case EXITING:{
			return NULL;
			}
		default: {
			break; // this is only for UNINITIALIZED state
			}
		}
		usleep(200000);
	}
	return NULL;
} 

/***********************************************************************
*	on_pause_press() 
*	Disarm the controller and set system state to paused.
*	If the user holds the pause button for 2 seconds, exit cleanly
************************************************************************/
int on_pause_press(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	switch(get_state()){
	// pause if running
	case EXITING:
		return 0;
	case RUNNING:
		set_state(PAUSED);
		setRED(HIGH);
		setGRN(LOW);
		break;
	case PAUSED:
		set_state(RUNNING);
		setGRN(HIGH);
		setRED(LOW);
		break;
	default:
		break;
	}
	// now wait to see if the user wants to shut down the program
	while(i<samples){
		usleep(us_wait/samples);
		if(get_pause_button_state() == UNPRESSED){
			return 0; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	blink_red();
	set_state(EXITING);
	return 0;
}

/***********************************************************************
*	on_mode_release()
*	placeholder, blinks green led for now
***********************************************************************/
int on_mode_release(){
	blink_green();
	return 0;
}

/***********************************************************************
*	blink_green()
*	nothing exciting, just blink the GRN LED for half a second
*	then return the LED to its original state
***********************************************************************/
int blink_green(){
	// record if the led was on or off so we can return later
	int old_state = getGRN();
	
	const int us_to_blink = 700000; // 0.7 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		setGRN(!old_state);
		usleep(delay);
		setGRN(old_state);
	}
	return 0;
}

/***********************************************************************
*	blink_red()
*	used to warn user that the program is exiting
***********************************************************************/
int blink_red(){
	const int us_to_blink = 2000000; // 2 seconds
	const int blink_hz = 10;
	const int delay = 1000000/(2*blink_hz); 
	const int blinks = blink_hz*us_to_blink/1000000;
	int i;
	for(i=0;i<blinks;i++){
		usleep(delay);
		setRED(HIGH);
		usleep(delay);
		setRED(LOW);
	}
	return 0;
}


/***********************************************************************
*	dsm2_watcher() run at 200Hz
*	listen for RC control for driving around
***********************************************************************/
void* dsm2_watcher(void* ptr){
	const int timeout_frames = 10; // after 10 missed checks, consider broken
	const int check_us = 5000; // dsm2 packets come in at 11ms, check faster
	drive_mode_t temp_drive_mode; // new drive mode selected by user switches
	int missed_frames;
	float turn, drive, throttle, switch1, switch2;
	
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			
			// Read normalized (+-1) inputs from RC radio right stick
			// positive means turn right or go forward
			turn = config.dsm2_turn_polarity * \
					get_dsm2_ch_normalized(config.dsm2_turn_ch);
			drive = config.dsm2_drive_polarity * \
					get_dsm2_ch_normalized(config.dsm2_drive_ch);
			throttle = config.dsm2_throttle_polarity * \
					get_dsm2_ch_normalized(config.dsm2_throttle_ch);
			switch1 = config.dsm2_switch1_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch1_ch);
			switch2 = config.dsm2_switch2_polarity * \
					get_dsm2_ch_normalized(config.dsm2_switch2_ch);
			if(switch1>0 && switch2>0){
				temp_drive_mode = NORMAL_4W;
			}
			else if(switch1>0 && switch2<0){
				temp_drive_mode = LANECHANGE;
			}
			else if(switch1<0 && switch2>0){
				temp_drive_mode = CRAB;
			}
			else if(switch1<0 && switch2<0){
				temp_drive_mode = SPIN;
			}
			else{
				printf("could not interpret DSM2 switches\n");
			}
			
			if(fabs(turn)>1.1 || fabs(drive)>1.1){
				// bad packet, ignore
			}
			else{
				missed_frames = 0;
				// dsm has highest interface priority so take over
				user_interface.input_mode = DSM2;
				user_interface.drive_stick = drive;
				user_interface.turn_stick  = turn;
				user_interface.throttle_stick  = throttle;
				user_interface.drive_mode = temp_drive_mode;
			}
			
		}
		// if no new data and currently operating in DSM2 mode, 
		// count a missed frame
		else if(user_interface.input_mode == DSM2){
			missed_frames ++;
			// if enough frames were missed and in DSM2 mode, 
			// this thread relinquishes control 
			if(missed_frames >= timeout_frames){
				user_interface.input_mode = NONE;
			}
		}
		// wait for the next frame
		usleep(check_us); 
	}
	return 0;
}
