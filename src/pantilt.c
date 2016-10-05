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

#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/stm32/f3/gpio.h>
#include <libopencm3/stm32/f3/timer.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3-plus/newlib/syscall.h>
#include "pantilt.h"
#include <libopencm3-plus/cdcacm_one_serial/cdcacm.h>
#include <stdio.h>
#include <libopencm3-plus/utils/misc.h>
#include <libopencm3-plus/stm32f3discovery/leds.h>
#include <limits.h>
#include <stdbool.h>

//pwm-related timer configuration
#define SYSFREQ     48000000 //168MHz
#define PWMFREQ        32000  //32000
#define PWMFREQ_F       ((float )(PWMFREQ)) //32000.0f
#define PRESCALE        1   //freq_CK_CNT=freq_CK_PSC/(PSC[15:0]+1)
#define PWM_PERIOD_ARR  SYSFREQ/( PWMFREQ*(PRESCALE+1) )
#define INIT_DUTY 0.0f //init duty


void leds_init(void) {
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8| GPIO9| GPIO10| GPIO11| GPIO12| GPIO13| GPIO14| GPIO15);
}

void tim1_init(void)
{
	/* Enable TIM1 clock. and Port E clock (for outputs) */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);//F4 clock for TIM 1 is APB2. TIM1 port is E

	/*Enable AF clock*/
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPEEN);


	//Set TIM1 channel (and complementary) output to alternate function push-pull.
	//f4 TIM1=> GIO9: CH1, GPIO11: CH2
	//f4 TIM1=> GIO8: CH1N, GPIO10: CH2N
	gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO9 | GPIO11);
	gpio_set_af(GPIOE, GPIO_AF1, GPIO9 | GPIO11);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO8 | GPIO10 );
	gpio_set_af(GPIOE, GPIO_AF1, GPIO8 | GPIO10);

	/* Enable TIM1 commutation interrupt. */
	//nvic_enable_irq(NVIC_TIM1_TRG_COM_TIM11_IRQ);	//f4

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

/* Disable repetition counter value for TIM1. */
	timer_set_repetition_counter(TIM1, 0); 

/* Enable preload on TIM1. */
	timer_enable_preload(TIM1);

/* Sets TIM1 Continuous mode. */
	timer_continuous_mode(TIM1);

	/* Period (32kHz). */
	timer_set_period(TIM1, PWM_PERIOD_ARR); //ARR (value compared against main counter to reload counter aka: period of counter)

	/* Configure break and deadtime. */
	//timer_set_deadtime(TIM1, deadtime_percentage*pwm_period_ARR);
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
	//timer_disable_oc_output(TIM1, TIM_OC3);
	//timer_disable_oc_output(TIM1, TIM_OC3N);

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

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_oc_output(TIM1, TIM_OC1N);
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC2N);
	
	/* ---- */

	/* ARR reload enable. */
	timer_enable_preload(TIM1);

	/*
	 * Enable preload of complementary channel configurations and
	 * update on COM event.
	 */
	//timer_enable_preload_complementry_enable_bits(TIM1);
	timer_disable_preload_complementry_enable_bits(TIM1);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM1);

	/* Generate update event to reload all registers before starting*/
	timer_generate_event(TIM1, TIM_EGR_UG);

	/* Counter enable. */
	timer_enable_counter(TIM1);

	/* Enable commutation interrupt. */
	//timer_enable_irq(TIM1, TIM_DIER_COMIE);

	/*********/
	/*Capture compare interrupt*/

	//enable capture compare interrupt
	timer_enable_update_event(TIM1);

	/* Enable commutation interrupt. */
	//timer_enable_irq(TIM1, TIM_DIER_CC1IE);	//Capture/compare 1 interrupt enable
	/* Enable commutation interrupt. */
	//timer_enable_irq(TIM1, TIM_DIER_CC1IE);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM1_UP_TIM16_IRQ);
}


void tim1_up_tim10_isr(void) {
  // Clear the update interrupt flag
  timer_clear_flag(TIM1,  TIM_SR_UIF);
  
}

void system_init(void) {
  rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_48MHZ]);
  leds_init();
  cdcacm_init();
  tim1_init();
}

void clean_stdin(void){
  while ( poll(stdin)>0){
    //printf("Cleaning stdin\n");
    getc(stdin);
  }
}

void stdin_init(void){
  setvbuf(stdin,NULL,_IONBF,0);  // Sets stdin  in unbuffered mode (normal for usart com)
  setvbuf(stdout,NULL,_IONBF,0); // Sets stdout in unbuffered mode (normal for usart com)
  clean_stdin();
} 

int main(void)
{
  system_init();
  stdin_init();

  char cmd_s[50]="";
  char cmd[10]="";
  char confirm[10]="";
  int i, j;
  int c=0;
  int n_char=0;

  while (1){
    printf("Test\n");
    if ((poll(stdin) > 0)) {
      i=0;
      if (poll(stdin) > 0) {
    	c=0;
    	while (c!='\r') {
    	  c=getc(stdin);
    	  i++;
    	  putc(c, stdout);
    	}
      }
    }
  }
}
