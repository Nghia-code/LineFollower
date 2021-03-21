#include "Sensor.h"
#include "stm32f1xx_hal.h"

int sensor[5];
int error;

uint8_t read_sensor(void)
{

	  sensor[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	  sensor[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	  sensor[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
	  sensor[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	  sensor[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

	  //if only middle sensor detects black line
	  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		  error = 0;
	  //going forward with full speed [1 1 0 1 1] - 0


	  //if only left sensor detects black line
	  else if (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
		  error = -1;
	  //going right with full speed [1 0 1 1 1] - (-1)


	  //if only left most sensor detects black line
	  else if (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
		  error = -4;
	  //going right with full speed [0 1 1 1 1] - (-5)


	  //if only right sensor detects black line
	  else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] ==1)
		  error = 1;
	  //going left with full speed [1 1 1 0 1] ---- (1)


	  //if only right most sensor detects black line
	  else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0)
	   	  error = 4;
	  //going left with full speed [1 1 1 1 0] ---- (5)


	  //if middle and right sensor detects black line
	  else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)
	 	  error = 2;
	  //going left with full speed [1 1 0 0 1] ---- (2)


	  //if middle and left sensor detects black line
	  else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		  error = -2;
	  //going right with full speed [1 0 0 1 1] ---- (-2)


	  //if middle, left and left most sensor detects black line
	  else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)
		 error = -3;
	  //going right with full speed [0 0 0 1 1] ---- (-3)


	  //if middle, right and right most sensor detects black line
	  else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)
		  error = 3;
	  //going left with full speed [1 1 0 0 0] ---- (3)

	  else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
	  		  error = 3;
	  // [0 0 1 1 1} --- (4)

	  else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0)
	  		  error = 3;
	  // [1 1 1 0 0] --- (-4)
	  else
		  error = -6 ;

	  return error;
}
