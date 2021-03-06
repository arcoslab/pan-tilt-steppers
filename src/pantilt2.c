
/*
 * Copyright (C) 2013 ARCOS-Lab Universidad de Costa Rica
 * Author: Federico Ruiz Ugalde <memeruiz@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*LibOpenCm3 libraries*/
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>

/*LibOpenCm3 libraries*/
#include <libopencm3-plus/newlib/syscall.h>
#include <libopencm3-plus/cdcacm_one_serial/cdcacm.h>
#include <libopencm3-plus/utils/misc.h>
#include <libopencm3-plus/stm32f4discovery/leds.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "pantilt2.h"

#define ABS(x) ((x > 0)? x : x*-1)

#define DEBUG 1

#define MSG_SIZE 50

/* control definitions */
#define TIM_TARGET 16000
#define TIM_TARGET_JOINT1 16
#define STEP_LIMIT 8
#define STEP_DOWNS 10
#define STEP_VEL_LIMIT (STEP_LIMIT * STEP_DOWNS)
#define DELTAVEL_LIM 500 /* hertz freq */
#define DELTAVEL_STEP 50 /* hertz freq */

#define NUM_JOINTS 2
#define JOINT1 0
#define JOINT2 1


//global variables

int serial_ready = 0;

/* Pinout definitions */
const uint32_t step_port[] = {GPIOD, GPIOD};//substitutes STEP_PORT
const uint16_t step_pin[] = {GPIO10, GPIO11};//substitutes STEP_PIN

const uint32_t dir_port[] = {GPIOD, GPIOD};//substitutes DIR_PORT
const uint16_t dir_pin[] = {GPIO7, GPIO8};//substitutes DIR_PIN

const uint32_t en_port[] = {GPIOD, GPIOD};//substitutes EN_PORT
const uint16_t en_pin[] = {GPIO6, GPIO2};//substitutes EN_PIN

const uint32_t cal_port[] = {GPIOD, GPIOD};//substitutes CAL_PORT 
const uint16_t cal_pin[] = {GPIO3, GPIO4};//substitutes CAL_PIN 

const int max_limit_pos[] = { 20000, 10000 }; //substitutes MAX_LIMIT_POS
const int min_limit_pos[] = { 10, 10};        //substitutes MIN_LIMIT_POS

/* command variables */
char ctrl_type = 't';   //defines type of control
int ctrl_val1 = 0;
int ctrl_val2 = 0;
int enbl = 0;           //Enable power to motor command
int denbl = 0;          //disable power to motor command
int dir = 0;            //establish direction
int calibrate = 0;      //execute calibration function command

/* control variables */

/* flags */
int calibrated[NUM_JOINTS];         //indicates if motor has been calibrated with init position
int vnull[NUM_JOINTS] = { 1, 1 };              //indicates velocity is zero after decreasing for a direction change
int change_dir[NUM_JOINTS];         //indicates that the direction has changed and is not yet applied to motor
int calibration_limit[NUM_JOINTS];   //indicates maximum position limit reached
int step_j[NUM_JOINTS];            //indicates step status is still high

/* vars */
int curr_pos_j1[NUM_JOINTS];        //indicates current position of motor
int targ_pos_j1[NUM_JOINTS];        //indicates currently commanded position
int deltav_j1[NUM_JOINTS];          //amount of difference between current and desired speed
int velocity[NUM_JOINTS];       //current speed
int vel_target_j1[NUM_JOINTS];      //indicates currently commanded position
int dir_j1[NUM_JOINTS];             //indicates current direction sent to motor
int targ_dir_j1[NUM_JOINTS];        //indicates desired direction 
int tim_target_j1[NUM_JOINTS] = { 16, 16 };     //amount of interruptions needed to send step
int remaining_steps[NUM_JOINTS];    //times DELTAVEL_STEP fits into current speed
int steps_to_limit[NUM_JOINTS];     //steps remaining to reach limit pos

/* temp vars */
int vel_temp_j1[NUM_JOINTS];        //storage for desired speed while speed in opposite direction decreases
int dir_tmp_j1[NUM_JOINTS];         //storage for desired direction while speed decreases

/* counters */
int tim_count_j1[NUM_JOINTS];       //curr amount of interruptions to next step
int step_j1_count[NUM_JOINTS];      //curr amount of interruptions to lower step 
int step_j1_vel_count[NUM_JOINTS];  //curr amount of steps to change velocity ***********cambiar


int prev_pos_j1[NUM_JOINTS];        //

/* Init configuration functions */
void leds_init(void) {
    rcc_periph_clock_enable(RCC_GPIOD);  
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

void gpio_init(void) {
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2 | GPIO6 | GPIO7 | GPIO8 | GPIO10 | GPIO11);
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO3 | GPIO4);
    gpio_set_output_options(GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO2 | GPIO6 | GPIO7 | GPIO8 | GPIO10 | GPIO11);
}

void tim_init(void) {
    /* Enable TIM1 clock. and Port E clock (for outputs) */
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_GPIOE);

    //Set TIM1 channel (and complementary) output to alternate function push-pull'.
    //f4 TIM1=> GIO9: CH1, GPIO11: CH2, GPIO13: CH3
    //f4 TIM1=> GIO8: CH1N, GPIO10: CH2N, GPIO12: CH3N
    gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9 | GPIO11 | GPIO13);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO9 | GPIO11 | GPIO13);
    gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO8 | GPIO10 | GPIO12);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO8 | GPIO10 | GPIO12);

    /* Reset TIM1 peripheral. */
    timer_reset(TIM1);

    /* Timer global mode:
    * - No divider
    * - Alignment edge
    * - Direction up
    */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, //For dead time and filter sampling, not important for now.
		 TIM_CR1_CMS_CENTER_3,	//TIM_CR1_CMS_EDGE
		 //TIM_CR1_CMS_CENTER_1
		 //TIM_CR1_CMS_CENTER_2
		 //TIM_CR1_CMS_CENTER_3 la frequencia del pwm se divide a la mitad.
		 TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM1, PRESCALE); //1 = disabled (max speed)
    timer_set_repetition_counter(TIM1, 0); //disabled
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);

    /* Period (32kHz). */
    timer_set_period(TIM1, PWM_PERIOD_ARR); //ARR (value compared against main counter to reload counter aka: period of counter)

    /* Configure break and deadtime. */
    timer_set_enabled_off_state_in_idle_mode(TIM1);
    timer_set_enabled_off_state_in_run_mode(TIM1);
    timer_disable_break(TIM1);
    timer_set_break_polarity_high(TIM1);
    timer_disable_break_automatic_output(TIM1);
    timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);

    /* Disable outputs. */
    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_disable_oc_output(TIM1, TIM_OC1N);
    timer_disable_oc_output(TIM1, TIM_OC2);
    timer_disable_oc_output(TIM1, TIM_OC2N);
    timer_disable_oc_output(TIM1, TIM_OC3);
    timer_disable_oc_output(TIM1, TIM_OC3N);

    /* -- OC1 and OC1N configuration -- */
    /* Configure global mode of line 1. */
    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    /* Configure OC1. */
    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC1); //When idle (braked) put 0 on output
    /* Configure OC1N. */
    timer_set_oc_polarity_high(TIM1, TIM_OC1N);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC1N);
    /* Set the capture compare value for OC1. */
    timer_set_oc_value(TIM1, TIM_OC1, INIT_DUTY*PWM_PERIOD_ARR);//initial_duty_cycle*pwm_period_ARR);

    /* -- OC2 and OC2N configuration -- */
    /* Configure global mode of line 2. */
    timer_enable_oc_preload(TIM1, TIM_OC2);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    /* Configure OC2. */
    timer_set_oc_polarity_high(TIM1, TIM_OC2);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC2);
    /* Configure OC2N. */
    timer_set_oc_polarity_high(TIM1, TIM_OC2N);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC2N);
    /* Set the capture compare value for OC2. */
    timer_set_oc_value(TIM1, TIM_OC2, INIT_DUTY*PWM_PERIOD_ARR);//initial_duty_cycle*pwm_period_ARR);

    /* -- OC3 and OC3N configuration -- */
    /* Configure global mode of line 3. */
    timer_enable_oc_preload(TIM1, TIM_OC3);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
    /* Configure OC3. */
    timer_set_oc_polarity_high(TIM1, TIM_OC3);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC3);
    /* Configure OC3N. */
    timer_set_oc_polarity_high(TIM1, TIM_OC3N);
    timer_set_oc_idle_state_unset(TIM1, TIM_OC3N);
    /* Set the capture compare value for OC3. */
    timer_set_oc_value(TIM1, TIM_OC3, INIT_DUTY*PWM_PERIOD_ARR);//initial_duty_cycle*pwm_period_ARR);//100);

    /* ARR reload enable. */
    timer_enable_preload(TIM1);

    /*
    * Enable preload of complementary channel configurations and
    * update on COM event.
    */
    timer_disable_preload_complementry_enable_bits(TIM1);

    /* Enable outputs in the break subsystem. */
    timer_enable_break_main_output(TIM1);

    /* Generate update event to reload all registers before starting*/
    timer_generate_event(TIM1, TIM_EGR_UG);

    /* Counter enable. */
    timer_enable_counter(TIM1);

    //enable capture compare interrupt
    timer_enable_update_event(TIM1);

    timer_enable_irq(TIM1, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);
}

void serial_conf(void) {
    setvbuf(stdin,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)
    setvbuf(stdout,NULL,_IONBF,0); // Sets stdin in unbuffered mode (normal for usart com)

    while (poll(stdin) > 0) {
        getc(stdin);
    }

    serial_ready = 1;
}

void system_init(void) {
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    leds_init();
    gpio_init();
    cdcacm_init(); //default 921600bps    
    tim_init();
    serial_conf();
}

/* End configuration functions */

/* Init control functions */

/*calibration 

Rotates the motor towards a direction (which, once calibrated, would be decreasing position). It will rotate with a fixed velocity until it receives a 'high' in its input pin which indicates that the movement has reached its limit. When that happens, the position is marked as zero and the motor stops rotating */

void calibration(int joint_id) {
    velocity[joint_id] = DELTAVEL_LIM;
    tim_target_j1[joint_id] = TIM_TARGET / velocity[joint_id];//indicates how many interuption cycles have to be completed to send a step signal
    dir_j1[joint_id] = 1;
    gpio_clear(dir_port[joint_id], dir_pin[joint_id]);//sends a 'high' to the dir GPIO
    if (!calibrated[joint_id]) {
        if (tim_count_j1[joint_id] == tim_target_j1[joint_id]) {
            step_j[joint_id] = 1;
            gpio_clear(step_port[joint_id], step_pin[joint_id]); // sends a 'high' to the step GPIO
            tim_count_j1[joint_id] = 0;
        }
        else{
            if (tim_count_j1[joint_id] > STEP_LIMIT){ 
                step_j[joint_id] = 0;
                gpio_set(step_port[joint_id], step_pin[joint_id]); // sends a 'low' to the step GPIO
            }
            tim_count_j1[joint_id] += 1;
        }
    }
    calibration_limit[joint_id] = gpio_get(cal_port[joint_id], cal_pin[joint_id]);//reads the state in the calibration limit GPIO
    if (calibration_limit[joint_id]){
        velocity[joint_id] = 0;
        curr_pos_j1[joint_id] = 0;
        calibrated[joint_id] = 1;
	    step_j1_count[joint_id] = 1;
        tim_count_j1[joint_id] = 0;
        step_j[joint_id] = 0;
        /*variables and flags to set the initial position to 10*/
        //ctrl_type[joint_id] = 'p';
        //targ_pos_j1[joint_id] = 10;
        //vel_target_j1[joint_id] = DELTAVEL_LIM;
    }
    calibrate = 0;
    int joint;
    for (joint = 0; joint < NUM_JOINTS; joint++) {
        if (!calibrated[joint]){
            calibrate = 1;
        }
    }
}

/*
lower_step

Lowers the step signal, when operating after the calibration has been completed.

*/

void lower_step(int joint_id){
    if (step_j[joint_id] == 1 && step_j1_count[joint_id] == STEP_LIMIT){
        gpio_clear(step_port[joint_id], step_pin[joint_id]);
        #ifdef DEBUG
        gpio_clear(GPIOD, GPIO14);
        #endif
        step_j[joint_id] = 0;
        step_j1_count[joint_id] = 0;
    }
}

/*
set_step

Determines how many interruption cycles have to be completed to send a step signal. When said quantity of steps have been fullfilled sets the step pin in 'high' 
*/

void set_step(int joint_id) {    
    tim_target_j1[joint_id] = (int) (velocity[joint_id] > 0)? TIM_TARGET / velocity[joint_id] : 0; //indicates how many interuption cycles have to be completed to send a step signal
    if (tim_target_j1[joint_id] != 0){
        calibration_limit[joint_id] = gpio_get(cal_port[joint_id], cal_pin[joint_id]);        
        if (calibration_limit[joint_id]) {
            tim_count_j1[joint_id] = 0;
        }
        if (tim_count_j1[joint_id] >= tim_target_j1[joint_id]) {
            step_j[joint_id] = 1;
			step_j1_count[joint_id] = 1;
            gpio_set(step_port[joint_id], step_pin[joint_id]); //sends a 'high' to the step GPIO
            #ifdef DEBUG
            gpio_set(GPIOD, GPIO14);
            #endif
            tim_count_j1[joint_id] = 0;        

            if (dir_j1[joint_id] == 0){
                curr_pos_j1[joint_id] += 1;
            }
            else{
                curr_pos_j1[joint_id] -= 1;
            }
        }
		else {
			step_j1_count[joint_id]++;
		}
        tim_count_j1[joint_id] += 1;
    }
}

/*
establish_direction

Outputs a 'high' or a 'low' in the direction control pin depending on the control value.
*/

void establish_direction(int joint_id){
    if (dir_j1[joint_id] == 1){
        gpio_clear(dir_port[joint_id], dir_pin[joint_id]);
    }
    else{
        gpio_set(dir_port[joint_id], dir_pin[joint_id]);
    }
}

/*
change_velocity

Determines what the difference is between the actual rotation velocity and the target velocity. Then, if there's a difference it will change the value of the velocity to either the target velocity, or a speed closer to the target. It also determines if the desacceleration after a change of movement direction has been completed.
*/

void change_velocity(int joint_id){
    deltav_j1[joint_id] = vel_target_j1[joint_id] - velocity[joint_id];
    if (ABS(deltav_j1[joint_id]) > DELTAVEL_STEP){
        if (step_j1_count[joint_id] == 1 && step_j1_vel_count[joint_id] >= tim_target_j1[joint_id] * STEP_DOWNS){ //a counter ensures that the velocity changes are executed every determined number of steps
            velocity[joint_id] += (deltav_j1[joint_id] > 0)? DELTAVEL_STEP : (DELTAVEL_STEP * -1);
            step_j1_vel_count[joint_id] = 0;
			if (ctrl_type == 'p'){
				check_velocity_pos(joint_id);
    		}
			if (ctrl_type == 'v'){
				check_velocity(joint_id);
    		}
        }
    }
    else{
        if (step_j1_count[joint_id] == 1 && step_j1_vel_count[joint_id] >= tim_target_j1[joint_id] * STEP_DOWNS){ //a counter ensures that the velocity changes are executed every determined number of steps
            velocity[joint_id] = vel_target_j1[joint_id];
            step_j1_vel_count[joint_id] = 0;
            vnull[joint_id] = (vel_target_j1[joint_id] == 0)? 1 : 0;
			if (ctrl_type == 'p'){
				check_velocity_pos(joint_id);
    		}
			if (ctrl_type == 'v'){
				check_velocity(joint_id);
    		}
        }
    }
    if (vnull[joint_id] && change_dir[joint_id]){ //if there is a change in direction taking place, and the velocity is null, the deacceleration has finished
        dir_j1[joint_id] = dir_tmp_j1[joint_id];
        targ_dir_j1[joint_id] = dir_tmp_j1[joint_id];
        change_dir[joint_id] = 0;
        step_j1_vel_count[joint_id] = 0;
        vel_target_j1[joint_id] = vel_temp_j1[joint_id];
        vnull[joint_id] = 0;
    }
    step_j1_vel_count[joint_id] += 1;
}

/*
check velocity

Determines if, given the current velocity, and while in velocity control mode, the motor needs to start deaccelerating to stop at the absolute maximum or minimum limits.
*/

void check_velocity(int joint_id){
    if (calibrated[joint_id]){
        remaining_steps[joint_id] = (int) ceil((float) velocity[joint_id] / (float) DELTAVEL_STEP); // determines hoy many velocity changes are needed to stop moving
        steps_to_limit[joint_id] = (int) (dir_j1[joint_id] == 0)? floor((max_limit_pos[joint_id] - curr_pos_j1[joint_id]) / STEP_DOWNS) : ceil((curr_pos_j1[joint_id] - min_limit_pos[joint_id]) / STEP_DOWNS); //determines how many multiples of the quantity of steps between every velocity change, are there between the current position and the maximum or minimum limit.
        vel_target_j1[joint_id] = (remaining_steps[joint_id] >= (steps_to_limit[joint_id]))? vel_target_j1[joint_id] - DELTAVEL_STEP : vel_target_j1[joint_id];	
        #ifdef DEBUG
        if (vel_target_j1[joint_id] == 0){
            gpio_set(GPIOD, GPIO12);
        }
        else{
            gpio_clear(GPIOD, GPIO12);
        }
        #endif
    }
}

/*
check_velocity_pos

Determines if, given the current velocity, and while in position control mode, the motor needs to start deaccelerating to stop at the target position.
*/

void check_velocity_pos(int joint_id){
    if (calibrated[joint_id]){
        remaining_steps[joint_id] = (int) ceil((float) velocity[joint_id] / (float) DELTAVEL_STEP); // determines hoy many velocity changes are needed to stop moving
        if (change_dir[joint_id] == 0){
            steps_to_limit[joint_id] = (int) (dir_j1[joint_id] == 0)? floor((targ_pos_j1[joint_id] - curr_pos_j1[joint_id]) / STEP_DOWNS) : ceil((curr_pos_j1[joint_id] - targ_pos_j1[joint_id]) / STEP_DOWNS);//determines how many multiples of the quantity of steps between every velocity change, are there between the current position and the target position, if no change in rotation sense is detected.
        }
        else {
            steps_to_limit[joint_id] = (int) (dir_j1[joint_id] == 1)? floor((targ_pos_j1[joint_id] - curr_pos_j1[joint_id]) / STEP_DOWNS) : ceil((curr_pos_j1[joint_id] - targ_pos_j1[joint_id]) / STEP_DOWNS);//determines how many multiples of the quantity of steps between every velocity change, are there between the current position and the target position, if a change in rotation sense is ongoing.
        }
            vel_target_j1[joint_id] = ((remaining_steps[joint_id]) >= (ABS(steps_to_limit[joint_id])+1))? velocity[joint_id] - DELTAVEL_STEP : vel_target_j1[joint_id];
        #ifdef DEBUG
        if (vel_target_j1[joint_id] == 0){
            gpio_set(GPIOD, GPIO12);
        }
        else{
            gpio_clear(GPIOD, GPIO12);
        }
        #endif
    }
}

/*
position_control

Sets the required variables for position control mode operation. Checks if a change in rotation direction happens so that the motor deaccelerates in the current direction.
*/

void position_control(int joint_id){
    if (dir_j1[joint_id] != targ_dir_j1[joint_id]){
        dir_tmp_j1[joint_id] = targ_dir_j1[joint_id];
        targ_dir_j1[joint_id] = dir_j1[joint_id];
        change_dir[joint_id] = 1;
        vel_temp_j1[joint_id] = vel_target_j1[joint_id];
        vel_target_j1[joint_id] = 0;
    }
    else{
        if (curr_pos_j1[joint_id] > targ_pos_j1[joint_id] && change_dir[joint_id] != 1){
            targ_dir_j1[joint_id] = 1;
        }
        else if (curr_pos_j1[joint_id] < targ_pos_j1[joint_id] && change_dir[joint_id] != 1){
            targ_dir_j1[joint_id] = 0;
        }
        else if(curr_pos_j1[joint_id] == targ_pos_j1[joint_id]){
            velocity[joint_id] = 0;
            vel_target_j1[joint_id] = 0;
            tim_count_j1[joint_id] = 1;
        }
    }
}

/*
velocity_control

Sets the required variables for velocity control mode operation. Checks if a change in rotation direction happens so that the motor deaccelerates in the current direction. Stops the rotation entirely if a maximum or minimum limit is reached.
*/

void velocity_control(int joint_id){
    if (dir_j1[joint_id] != targ_dir_j1[joint_id]){
        dir_tmp_j1[joint_id] = targ_dir_j1[joint_id];
        targ_dir_j1[joint_id] = dir_j1[joint_id];
        change_dir[joint_id] = 1;
        vel_temp_j1[joint_id] = vel_target_j1[joint_id];
        vel_target_j1[joint_id] = 0;
    }
    else{
	    if ((velocity[joint_id] != 0) && (curr_pos_j1[joint_id] == max_limit_pos[joint_id]) && (dir_j1[joint_id] == 0)){
	        velocity[joint_id] = 0;
		}
		if ((velocity[joint_id] != 0) && (curr_pos_j1[joint_id] == max_limit_pos[joint_id]) && (dir_j1[joint_id] == 1)){
			velocity[joint_id] = 0;
		}
    }
}

/* End control functions */

/* Init timer functions */

/*
tim1_up_tim10_isr

Executes the time based interruption.
*/

void tim1_up_tim10_isr(void) {
    timer_clear_flag(TIM1,  TIM_SR_UIF); // Clear the update interrupt flag

    int joint_id;  

    if (enbl == 1){
        for(joint_id = 0; joint_id < NUM_JOINTS; joint_id++){
            gpio_set(en_port[joint_id], en_pin[joint_id]);
        }
        enbl = 0;
    }
    
    if (denbl == 1){
         for(joint_id = 0; joint_id < NUM_JOINTS; joint_id++){
             gpio_clear(en_port[joint_id], en_pin[joint_id]);
         }
        denbl = 0;
    }

    for (joint_id = 0; joint_id < NUM_JOINTS; joint_id++) {


        if (dir == 1){
            dir_j1[joint_id] = (dir_j1[joint_id] == 1)? 0 : 1;
            dir = 0;
        }

        if (calibrate){
            calibration(joint_id);
        }

        if (ctrl_type == 'p'){
            position_control(joint_id);
        }

        if (ctrl_type == 'v'){
            velocity_control(joint_id);
        }

        if (calibrated[joint_id]){
            establish_direction(joint_id);
            lower_step(joint_id);
            set_step(joint_id);
            change_velocity(joint_id);
        }
    }
}

/* End timer functions */

/* Init util functions */

/*
main

Polls the PC via serial communication to set the control variables needed to determine the operation of the control.
*/

void read_serial(char* buffer){
    int i = 0;
    char c = ' ';
    memset(buffer,0,MSG_SIZE);
    c = getc(stdin);
    while (c != '\r'){
        buffer[i] = c;
        i++;
        c = getc(stdin);
        putc(c, stdout);
    }
    buffer[i]='\0';
}

/* End util functions */

/* Main function */

int main(void){
    char cmd_s[MSG_SIZE]=" ";
    system_init();
    char ctrl_sel = 't';
    int val1 = 0;
    int val2 = 0;
    int val3 = 0;
    int val4 = 0;

    //init values
    enbl = 0;
    denbl = 0;
    dir = 0;
    
    int joint_id;

    for(joint_id = 0; joint_id < NUM_JOINTS; joint_id ++) {   
        vnull[joint_id] = 1;
        change_dir[joint_id] = 0;
        dir_j1[joint_id] = 0;
        dir_tmp_j1[joint_id] = 0;
        targ_dir_j1[joint_id] = 0;
        gpio_clear(en_port[joint_id], en_pin[joint_id]);
        gpio_clear(step_port[joint_id], step_pin[joint_id]);
    }

    while(true) {
        if ((poll(stdin) > 0)) {
            read_serial(cmd_s);
            sscanf(cmd_s, "%c %d %d %d %d\n", &ctrl_sel, &val1, &val2, &val3, &val4);

            // enable pantilt
            if (ctrl_sel == 'e') {
	            enbl = 1;
                calibrate = 1;
            }
	        // disable pantilt
            if (ctrl_sel == 'o') {
	            denbl = 1;
                calibrate = 0;
                calibrated[JOINT1] = 0;
                calibrated[JOINT2] = 0;
            }
	        // change dir
	        if (ctrl_sel == 'd'){
	            dir = 1;
	        }
            if (ctrl_sel == 'c'){
	            calibrate = 1;
	        }

            ctrl_type = 't';
			
			//sets variables to execute position control
            if (ctrl_sel == 'p') {
                ctrl_type = ctrl_sel;
                targ_pos_j1[JOINT1] = val1;
                targ_pos_j1[JOINT2] = val2;
                vel_target_j1[JOINT1] = DELTAVEL_LIM;
                vel_target_j1[JOINT2] = DELTAVEL_LIM;
            }        

			//sets variables to execute velocity control
            if (ctrl_sel == 'v') {
                ctrl_type = ctrl_sel;
                vel_target_j1[JOINT1] = val1;
                vel_target_j1[JOINT2] = val3;
                targ_dir_j1[JOINT1] = val2;
                targ_dir_j1[JOINT2] = val4; 
            }
        }
    	printf("step: %d vel_j1: %d remaining_steps: %d steps_to_limit %d curr_pos: %d change_dir: %d vel_target: %d \n", step_j[JOINT1], velocity[JOINT1], remaining_steps[JOINT1], steps_to_limit[JOINT1], curr_pos_j1[JOINT1], change_dir[JOINT1], vel_target_j1[JOINT1]);
    }
    return(0);
}


