
#include "init.h"


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize peripherals */
	MX_GPIO_Init();

    /* CALL REMAINING INIT FUNCTIONS HERE, they are defined in init.c and should 
     * be declared in init.h, which is included at the beginning of this file.
     * this is a template file and does not get modified in any way, you are
     * responsible for calling those functions. */

    while (1)
    {


    }
}
