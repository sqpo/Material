/* =========================================================================
 * Project:       ADC_Interrupt
 * File:          main.c
 * Description:   ADC转换结束中断
 *   1. 1.设置ADC时钟频率为1MHz，采样脉冲宽度为8个ADC时钟，
 *	    ADC conversion timeADC转换时间 = (8+12+2)*1us = 22us , ADC conversion rateADC转换速率 = 1/22us = 45.5KHz
 *   2. 将AIN1(PA1)设置为ADC模拟输入
 *   3. 将ADC转换结果位11~位4存储到RAM“R_AIN1_DATA_HB”，位3~位0存储到RAM“R_AIN1_DATA_LB[3:0]”(x=0~4)               
 * Author:        JasonLee
 * Version:       V1.0		                      
 * Date:          2019.10.01
 =========================================================================*/
#include <ny8.h>
#include "ny8_constant.h"

unsigned char R_AIN1_DATA_HB;	
unsigned char R_AIN1_DATA_LB;
unsigned char text;
unsigned char test;
#define UPDATE_REG(x)	__asm__("MOVR _" #x ",F")

void F_wait_eoc(void);
void delay(int);

//! interrupt service routine
void isr(void) __interrupt(0)
{
	ADRbits.ADIF = 0;						// Clear adc interrupt flag bit
	R_AIN1_DATA_HB = ADD;					// RAM "R_AIN1_DATA_HB" Store AIN1's ADC data bit 11~4
	R_AIN1_DATA_LB = 0x0F & ADR;			// RAM "R_AIN1_DATA_LB" bit3~0 Store AIN1's ADC data bit 3~0
	ADMDbits.START =1;						// Start a ADC conversion session		
}

void main(void)
{
    R_AIN1_DATA_HB=R_AIN1_DATA_LB=0x00;
  //----- Initial GPIO-----
    IOSTA = C_PA_Input;						// Set PortA as input port
    PORTA = 0xFF;							// PortA Data Register = 0xFF
    INTE  = 0x00;							// INTE = 0x00

 //----- Initial ADC-----	  
	ADMD  = C_ADC_En | C_ADC_CH_Dis | C_ADC_PA1 ;	// Enable ADC power, Disable global ADC input channel, Select PA1 pad as ADC input (SFR "ADMD")
 	ANAEN	= C_CMPEN ;						// Enable Analog Bias
 //----- ADC high reference voltage source select-----
	ADVREFH = C_Vrefh_VDD;					// ADC reference high voltage is supplied by internal VDD (Note: ADC clock freq. must be equal or less 2MHz @ VDD=5.0V) 
 	//ADVREFH = C_Vrefh_4V;					// ADC reference high voltage is supplied by internal 4V  (Note: ADC clock freq. must be equal or less 1MHz)
 	//ADVREFH = C_Vrefh_3V;					// ADC reference high voltage is supplied by internal 3V  (Note: ADC clock freq. must be equal or less 500KHz)
 	//ADVREFH = C_Vrefh_2V;					// ADC reference high voltage is supplied by internal 2V  (Note: ADC clock freq. must be equal or less 250KHz)
 
//----- ADC clock frequency select----------------------------	 
	ADR	 = C_ADC_CLK_Div1;					// ADC clock=Fcpu/1, Clear ADIF, disable ADC interrupt	
	//ADR	 = C_ADC_CLK_Div2;				// ADC clock=Fcpu/2, Clear ADIF, disable ADC interrupt	
	//ADR	 = C_ADC_CLK_Div8;				// ADC clock=Fcpu/8, Clear ADIF, disable ADC interrupt	
	//ADR	 = C_ADC_CLK_Div16;				// ADC clock=Fcpu/16, Clear ADIF, disable ADC interrupt	
 
//----- ADC Sampling pulse width select-------------	 
 	//ADCR  = C_Sample_1CLK | C_12BIT;		// Sample pulse width=1 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 500KHz)
 	//ADCR  = C_Sample_2CLK | C_12BIT;		// Sample pulse width=2 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1MHz)
 	//ADCR  = C_Sample_4CLK | C_12BIT;		// Sample pulse width=4 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 1.25MHz)
 	ADCR  = C_Sample_8CLK | C_12BIT; 		// Sample pulse width=8 adc clock, ADC select 12-bit conversion ( Note: ADC clock freq. must be equal or less 2MHz)	

//--------------------------------------------------	
	PACON = C_PA1_AIN1;						// Set AIN1(PA1) as pure ADC input for reducing power consumption (SFR "PACON")
	ADMDbits.GCHS = 1;						// Enable global ADC channel	(SFR "ADMD")
	ADRbits.ADIF = 0;						// Clear adc interrupt flag bit
	ADRbits.ADIE = 1;						// Enable adc interrupt bit
	delay(50);								// Delay 1.12ms(Instruction clock=4MHz/4T) for waiting ADC stable
	ADMDbits.START =1;						// Start a ADC conversion session	
	ENI();
		 
    while(1)
    {
        CLRWDT();							// Clear WatchDog
    }
}

void delay(int count)
{
	int i;
	for(i=1;i<=count;i++)
	;
}