#include "main.h"

int main()
{
	// Initialize the board
	Board_Init();

	// Time Slice Definition
	static sysTick_t taskDelayer1 = 0;
	static sysTick_t taskDelayer2 = 0; 
    static sysTick_t taskDelayer3 = 0;

	while(1)
	{
		// Event Driven Section
		printf("Hello World!\r\n");

		// Time Slice Polling Section
		if(sysTick_DelayMs(&taskDelayer1, 500)){
			_INFO("0.5s");
		}
		if(sysTick_DelayMs(&taskDelayer2, 1000)){
			_WARN("1s");
		}
        if(sysTick_DelayMs(&taskDelayer3, 2000)){
			_ERROR("2s");
		}
	}
}

void Board_Init(void)
{
	// Initialize the hardware abstraction layer
	UART_Init();
	Timer_Init();
	GPIO_I2C_Init();
	sysTick_Init();
	// Initialize the driver layer
	
	// Initialize the service layer

	// Initialize the application layer

}
