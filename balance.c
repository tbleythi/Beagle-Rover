/*  code for balancing the Rover


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

/********************************************
* 			Includes & Constants			*
*********************************************/
#include <robotics_cape.h>

#define SAMPLE_RATE_HZ 200	// main filter and control loop speed
#define DT 0.005       		// 1/sample_rate

#include "balance_logging.h"
#include "balance_config.h"

/************************************************************************
* 	core_mode_t
*
*	ANGLE: Only body angle theta and steering is controlled, this lets you
*	push the MiP around. 
*
*	POSITION: Additionally the wheel position is controlled so the MiP
*	will stay still on a table.
************************************************************************/
typedef enum core_mode_t{  
	ANGLE
	//POSITION
}core_mode_t;

/************************************************************************
* 	arm_state_t
*
*	ARMED or DISARMED to indicate if the controller is running
************************************************************************/
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/************************************************************************
* 	core_setpoint_t
*	setpoint for the balance controller
*	This is controlled by the balance stack and read by the balance core	
************************************************************************/
typedef struct core_setpoint_t{
	
	core_mode_t core_mode;	// see core_state_t declaration
	arm_state_t arm_state;	// see arm_state_t declaration
	
	float theta;	// body lean angle (rad)
	float gamma;	// body turn angle (rad)
	float gamma_dot;
	
}core_setpoint_t;

/************************************************************************
* 	core_state_t
*	contains workings of the feedback controller
*	Should only be written to by the balance core after initialization		
************************************************************************/
typedef struct core_state_t{
	// time when core_controller has finished a step
	uint64_t time_us; 
	// inner feedback loop to control theta body angle
	float theta[3];
	float theta_ref[3];
	float current_theta;
	float d_theta;
	float eTheta[3];
	// outer loop to control wheel angle Phi
	float phi[3];
	//float current_phi;
	//float d_phi;
	//float ePhi[3];
	// steering controller for gamma
	float gamma[3];
	float current_gamma;
	float d_gamma;
	float egamma[3]; 
	// outputs
	float u[3];
	float current_u;
	float duty_split;
	//battery voltage for scaling motor inputs.
	float vBatt; 
} core_state_t;

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
* 	user_interface_t
*	represents current command by the user which may be populated from 
*	DSM2, mavlink, bluetooth, or any other communication you like
************************************************************************/
typedef struct user_interface_t{
	// written by the last input watching thread to update
	input_mode_t mode;
	
	// All sticks scaled from -1 to 1
	float drive_stick; 	// positive forward
	float turn_stick;	// positive to the right, CW yaw
}user_interface_t;

// set with microsSinceEpoch() when the core starts
uint64_t core_start_time_us;

/************************************************************************
* 	Function declarations in this c file
*	also check out functions in balance_config.h & balance_logging.h		
************************************************************************/
// hardware interrupt routines
int balance_core();

//threads
void* balance_stack(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* dsm2_listener(void* ptr);
void* mavlink_listener(void* ptr);
void* mavlink_sender(void* ptr);

// regular functions
int main();
int balance_core(); // IMU interrupt routine
int zero_out_controller();
int disarm_controller();
int arm_controller();
int saturate_number(float* val, float limit);
int wait_for_starting_condition();
int on_pause_press();
int on_mode_release();
int blink_green();
int blink_red();

/************************************************************************
* 	Global Variables				
************************************************************************/
balance_config_t config;
core_state_t cstate;
core_setpoint_t setpoint;
user_interface_t user_interface;
// mavlink socket and socket address
int* udp_sock;
struct sockaddr_in gcAddr;


/***********************************************************************
*	main()
*	initialize the IMU, start all the threads, and wait still something 
*	triggers a shut down
***********************************************************************/
int main(){
	initialize_cape();
	setRED(HIGH);
	setGRN(LOW);
	set_state(UNINITIALIZED);
	
	// set up button handlers first
	// so user can exit by holding pause
	set_pause_pressed_func(&on_pause_press);
	//set_mode_unpressed_func(&on_mode_release);
	
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
			pthread_create(&dsm2_thread, NULL, dsm2_listener, (void*) NULL);
		}
	}
	
	// start logging thread if enabled
	if(config.enable_logging){
		if(start_log(SAMPLE_RATE_HZ, &cstate.time_us)<0){
			printf("failed to start log\n");
		}
		else{
			// start new thread to write the file occasionally
			pthread_t  logging_thread;
			pthread_create(&logging_thread, NULL, log_writer, (void*) NULL);
		}
	}
	
	// Finally start the real-time interrupt driven control thread
	// start IMU with equilibrium set with upright orientation 
	// for MiP with Ethernet pointing relatively up
	signed char orientation[9] = ORIENTATION_UPRIGHT; 
	if(initialize_imu(SAMPLE_RATE_HZ, orientation)){
		// can't talk to IMU, all hope is lost
		// blink red until the user exits
		blink_red();
		return -1;
	}
	// this should be the last step in initialization 
	// to make sure other setup functions don't interfere
	printf("starting core IMU interrupt\n");
	core_start_time_us = microsSinceEpoch();
	set_imu_interrupt_func(&balance_core);
	
	// start balance stack to control setpoints
	pthread_t  balance_stack_thread;
	pthread_create(&balance_stack_thread, NULL, balance_stack, (void*) NULL);
	
	printf("\nHold your MIP upright to begin balancing\n");
	set_state(RUNNING);
	
	// chill until something exits the program
	while(get_state()!=EXITING){
		usleep(100000);
	}
	
	// close(*udp_sock); 	// close network socket
	cleanup_cape(); // always end with cleanup to shut down cleanly
	return 0;
}

/***********************************************************************
*	balance_stack
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
	setpoint.core_mode = ANGLE; //we do not currently have position control
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
		
		
			if(user_interface.mode == NONE){
				// no user input, just keep the controller setpoint at zero
				setpoint.theta = 0;
				setpoint.gamma = 0;
				setpoint.gamma_dot = 0; 
				break;
			}
			if(setpoint.core_mode == ANGLE){
				// in angle mode, scale user input from -1 to 1 to
				// the minimum and maximum theta reference angles
				//setpoint.theta = config.theta_ref_max*(saturate_number(&user_interface.drive_stick,1));
				setpoint.theta = user_interface.drive_stick;
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
		usleep(1000000/SAMPLE_RATE_HZ); 
	}
	return NULL;
}

/************************************************************************
* 	balance_core()
*	discrete-time balance controller operated off IMU interrupt
*	Called at SAMPLE_RATE_HZ
************************************************************************/
int balance_core(){
	// local variables only in memory scope of balance_core
	static int D1_saturation_counter = 0; 
	float compensated_D1_output = 0;
	float dutyL = 0;
	float dutyR = 0;
	static log_entry_t new_log_entry;
	
	// if an IMU packet read failed, ignore and just return
	// the mpu9150_read function may print it's own warnings
	if (mpu9150_read(&mpu) != 0){
		return -1;
	}
	
	/***********************************************************************
	*	STATE_ESTIMATION
	*	read sensors and compute the state regardless of if the controller
	*	is ARMED or DISARMED
	************************************************************************/
	// angle theta is positive in the direction of forward tip
	// add mounting angle of BBB
	cstate.theta[2] = cstate.theta[1]; cstate.theta[1] = cstate.theta[0];
	cstate.theta[0] = mpu.fusedEuler[VEC3_X] + config.bb_mount_angle;
	cstate.current_theta = cstate.theta[0];
	cstate.d_theta = (cstate.theta[0]-cstate.theta[1])/DT;
	
	// body turning estimation
	cstate.gamma[2] = cstate.gamma[1]; cstate.gamma[1] = cstate.gamma[0];
	cstate.gamma[0]= mpu.fusedEuler[VEC3_Z];
	cstate.current_gamma = cstate.gamma[0];
	cstate.d_gamma = (cstate.gamma[0]-cstate.gamma[1])/DT;
	 
	
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
		
		// check for a tipover before anything else
		if(fabs(cstate.current_theta)>config.tip_angle){
			disarm_controller();
			printf("tip detected \n");
			break;
		}
		
		// evaluate inner loop controller D1z
		cstate.eTheta[2] = cstate.eTheta[1]; 
		cstate.eTheta[1] = cstate.eTheta[0];
		cstate.eTheta[0] = setpoint.theta - cstate.current_theta;
		cstate.u[2] = cstate.u[1];
		cstate.u[1] = cstate.u[0];
		cstate.u[0] = \
				config.K_D1 * (config.numD1_0 * cstate.eTheta[0]	\
								+	config.numD1_1 * cstate.eTheta[1] 	\
								+	config.numD1_2 * cstate.eTheta[2])	\
								-  (config.denD1_1 * cstate.u[1] 		\
								+	config.denD1_2 * cstate.u[2]); 		
		
		// check saturation of inner loop knowing that right after
		// this control will be scaled by battery voltage
		if(saturate_number(&cstate.u[0], config.v_nominal/cstate.vBatt)){
			D1_saturation_counter ++;
			if(D1_saturation_counter > SAMPLE_RATE_HZ/4){
				printf("inner loop controller saturated\n");
				disarm_controller();
				D1_saturation_counter = 0;
				break;
			}
		}
		else{
			D1_saturation_counter = 0;
		}
		cstate.current_u = cstate.u[0];
		
		// scale output to compensate for battery charge level
		compensated_D1_output = cstate.u[0] \
					* (config.v_nominal / cstate.vBatt);
		
		//steering controller
		// move the controller set points based on user input
		setpoint.gamma += setpoint.gamma_dot * DT;
		cstate.egamma[1] = cstate.egamma[0];
		cstate.egamma[0] = setpoint.gamma - cstate.current_gamma;
		//cstate.duty_split = config.KP_steer*(cstate.egamma[0]	\
				+config.KD_steer*(cstate.egamma[0]-cstate.egamma[1]));
		cstate.duty_split = user_interface.turn_stick;
		
		// if the steering input would saturate a motor, reduce
		// the steering input to prevent compromising balance input
		if(fabs(compensated_D1_output)+fabs(cstate.duty_split) > 1){
			if(cstate.duty_split > 0){
				cstate.duty_split = 1-fabs(compensated_D1_output);
			}
			else {
				cstate.duty_split = -(1-fabs(compensated_D1_output));
			}
		}	
		
		// add D1 balance controller and steering control
		dutyL  = compensated_D1_output + cstate.duty_split;
		dutyR = compensated_D1_output - cstate.duty_split;	
		
		// send to motors
		// one motor is flipped on chassis so reverse duty to L
		set_motor(config.motor_channel_L,dutyL); 				////////////switched signs on dutyL/R
		set_motor(config.motor_channel_R,-dutyR); 
		cstate.time_us = microsSinceEpoch();
		
		// pass new information to the log with add_to_buffer
		// this only puts information in memory, doesn't
		// write to disk immediately
		if(config.enable_logging){
			new_log_entry.time_us	= cstate.time_us-core_start_time_us;
			new_log_entry.theta		= cstate.current_theta;
			new_log_entry.theta_ref	= setpoint.theta; 
			new_log_entry.u 		= cstate.current_u;
			add_to_buffer(new_log_entry);
		}
		
		// end of normal balancing routine
		// last_state will be updated beginning of next interrupt
		break;
		
		default:
			break; // nothing to do if UNINITIALIZED
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
	// store theta and vbatt
	float theta_tmp = cstate.current_theta;
	float vBatt_tmp = cstate.vBatt;
	// wipe state
	memset(&cstate, 0, sizeof(core_state_t));
	// restore theta and vbatt
	cstate.current_theta = theta_tmp;
	cstate.vBatt = vBatt_tmp;
	// store previous controller mode
	core_mode_t last_mode = setpoint.core_mode;
	// wipe setpoint
	memset(&setpoint, 0, sizeof(core_setpoint_t));
	setpoint.core_mode = last_mode;
	
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
*	saturate_num(float val, float limit)
*	bounds val to +- limit
*	return one if saturation occurred, otherwise zero
************************************************************************/
int saturate_number(float* val, float limit){
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
*	battery_checker()
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
void* printf_loop(void* ptr){
	state_t last_state, new_state; // keep track of last state 
	while(1){
		new_state = get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf(" theta t_ref d_split  drive_stick   u    \n");
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
			printf(" %0.9f ", user_interface.drive_stick);
			//printf(" %0.2f ", cstate.current_gamma);
			printf(" %0.2f ", cstate.current_u);
			
			if(user_interface.mode == DSM2)
				printf(" DSM2 ");
			
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
		disarm_controller();
		setRED(HIGH);
		setGRN(LOW);
		break;
	case PAUSED:
		set_state(RUNNING);
		disarm_controller();
		setGRN(HIGH);
		setRED(LOW);
		break;
	default:
		break;
	}
	
	// now wait to see if the user want to shut down the program
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
*	toggle between position and angle modes if MiP is paused
***********************************************************************/
/* int on_mode_release(){
	// store whether or not the controller was armed
	int was_armed = setpoint.arm_state;
	
	// disarm the controller if necessary
	if(was_armed == ARMED){
		disarm_controller();
	}
	// toggle between position and angle modes
	if(setpoint.core_mode == POSITION){
		setpoint.core_mode = ANGLE;
		printf("using core_mode = ANGLE\n");
	}
	else {
		setpoint.core_mode = POSITION;
		printf("using core_mode = POSITION\n");
	}
	// arm the controller if it was armed before swapping modes
	if(was_armed == ARMED){
		zero_out_controller();
		arm_controller();
	}
	
	blink_green();
	return 0;
} */

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
*	dsm2_listener()
*	listen for RC control for driving around
***********************************************************************/
void* dsm2_listener(void* ptr){
	const int timeout_frames = 10; // after 10 missed checks, consider broken
	const int check_us = 5000; // dsm2 packets come in at 11ms, check faster
	
	int missed_frames;
	float turn, drive;
	
	while(get_state()!=EXITING){
		if(is_new_dsm2_data()){	
			
			// Read normalized (+-1) inputs from RC radio right stick
			// positive means turn right or go forward
			turn = config.dsm2_turn_polarity * \
					get_dsm2_ch_normalized(config.dsm2_turn_ch);
			drive = config.dsm2_drive_polarity * \
					get_dsm2_ch_normalized(config.dsm2_drive_ch);
			
			if(fabs(turn)>1.1 || fabs(drive)>1.1){
				// bad packet, ignore
			}
			else{
				missed_frames = 0;
				// dsm has highest interface priority so take over
				user_interface.mode = DSM2;
				user_interface.drive_stick = drive;
				user_interface.turn_stick  = turn;
			}
			
		}
		// if no new data and currently operating in DSM2 mode, 
		// count a missed frame
		else if(user_interface.mode == DSM2){
			missed_frames ++;
			// if enough frames were missed and in DSM2 mode, 
			// this thread relinquishes control 
			if(missed_frames >= timeout_frames){
				user_interface.mode = NONE;
			}
		}
		// wait for the next frame
		usleep(check_us); 
	}
	return 0;
}

