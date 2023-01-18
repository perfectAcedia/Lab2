#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>

//Var 14
//B13, B14 - PWM 0.2C
//B15 - variable PWM 
//B9 button - decreate pulse

#define PERIOD (uint16_t) (SystemCoreClock / 10000)  //Amount of timer ticks for one period
#define FREQUENT 1  //Amount of periods in one second

int main(void)
{
	int i = 0;
	uint16_t TIM_Pulse;
	
	GPIO_InitTypeDef port;
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef timerPWM;

	//Enable clock at GPIOA, GRIOB, and TIM1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	//B9 input mode 
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_IPU;
	port.GPIO_Pin =  GPIO_Pin_9;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &port);
	
	
	//B13-B15 push/pull mode
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	port.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &port);

	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler = SystemCoreClock / PERIOD / FREQUENT;
	timer.TIM_Period = PERIOD;
	TIM_TimeBaseInit(TIM1, &timer);

	TIM_Pulse = (uint16_t) (PERIOD*0.80);  //80% to get 20% on inverted pin
	TIM_OCStructInit(&timerPWM);
	timerPWM.TIM_OCMode = TIM_OCMode_PWM1;	
	timerPWM.TIM_OutputNState = TIM_OutputNState_Enable; //CH1N komplimentar output
	timerPWM.TIM_OutputState = TIM_OutputState_Enable; //CH1 output
	timerPWM.TIM_Pulse = TIM_Pulse;
	timerPWM.TIM_OCPolarity = TIM_OCPolarity_High; //CH1 komplimentar output
	timerPWM.TIM_OCNPolarity = TIM_OCNPolarity_High; //CH1N komplimentar output

	TIM_OC1Init(TIM1, &timerPWM);
	TIM_OC2Init(TIM1, &timerPWM);
	TIM_OC3Init(TIM1, &timerPWM);

  TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); //turn on MainOutputs. Required for TIM1, not required for others 
	
	//  1 - increase pulse
	// -1 - descrease pulse
	int direction = 1;
	
	while (1) {
		
		//Button B9 pressed - decrase/increase pulse 
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0) {
				//if max/min value reached change direction			
				if (TIM_Pulse > PERIOD || TIM_Pulse <0)
				   direction *= -1;
				TIM_Pulse += direction;
				TIM1->CCR3 = TIM_Pulse;
		}
		// delay to prevent contact bounce
		for(i=0;i<0x1000;i++);
	}
		
}