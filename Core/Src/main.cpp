
#include "init.h"

#include "app_fatfs.h"
#include <stdio.h>
#include "fatfs_sd.h"
#include "app_fatfs.h"

uint16_t address = 0;
FATFS fs;
FIL fil;
FRESULT fresult;
UINT br, bw;  // File read/write count
/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
uint8_t Rec_Data[14];
char buffer[128] = {};

struct {
    int16_t x;
    int16_t y;
    int16_t z;
} acc;

int16_t temp;
double temp_d;

struct {
    int16_t x;
    int16_t y;
    int16_t z;
} gyro;


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
    if (MX_FATFS_Init() != APP_OK) {
        Error_Handler();
    }
    MX_I2C1_Init();

    /* CALL REMAINING INIT FUNCTIONS HERE, they are defined in init.c and should 
     * be declared in init.h, which is included at the beginning of this file.
     * this is a template file and does not get modified in any way, you are
     * responsible for calling those functions. */

    //FATFS

    fresult = f_mount(&fs, "/", 1);
    if (fresult != FR_OK) Error_Handler();

    /*************** Card capacity details ********************/

    /* Check free space */
    f_getfree("", &fre_clust, &pfs);

    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    f_open(&fil, "file2.txt", FA_OPEN_ALWAYS | FA_WRITE);

    //scan i2c
    while (address == 0) {
        for (uint16_t i = 1; i < 128; i++) {
            if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 10) == HAL_OK)
                address = i << 1;
        }
    }

    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, address, 0x75,1,&data, 1, 10);//who-I-am check
    if(data != 0x68) Error_Handler();

    HAL_Delay(50);

    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, address, 0x6b, 1,&data, 1, 10);//pow conf
    HAL_I2C_Mem_Write(&hi2c1, address, 0x1b, 1,&data, 1, 10);//gyr conf
    HAL_I2C_Mem_Write(&hi2c1, address, 0x1c, 1,&data, 1, 10);//acc conf
    data = 7;
    HAL_I2C_Mem_Write(&hi2c1, address, 0x19, 1,&data, 1, 10);//data rate 1kHz

    data = 1<<4;
    HAL_I2C_Mem_Write(&hi2c1, address, 0x37, 1,&data, 1, 10);//irr clear when read
    data = 1<<0;
    HAL_I2C_Mem_Write(&hi2c1, address, 0x38, 1,&data, 1, 10);//irr enable
    data = 0x7;
    HAL_I2C_Mem_Write(&hi2c1, address, 0x68, 1,&data, 1, 10);//reset
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t i = 1024*free_space/65;
    while(i) {
        --i;
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
          HAL_I2C_Mem_Read(&hi2c1, address, 0x3b, 1, Rec_Data, 14, 2);
            acc.x = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
            acc.y = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
            acc.z = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

            temp = (int16_t)(Rec_Data[6]<<8 | Rec_Data[7]);
            temp_d = temp/340 + 36.53;

            gyro.x = (int16_t)(Rec_Data[8]<<8 | Rec_Data[9]);
            gyro.y = (int16_t)(Rec_Data[10]<<8 | Rec_Data[11]);
            gyro.z = (int16_t)(Rec_Data[12]<<8 | Rec_Data[13]);

            sprintf(buffer,"%08x,%08x,%08x;%03f;%08x,%08x,%08x\n",acc.x,acc.y,acc.z,temp_d,gyro.x,gyro.y,gyro.z);
            f_puts(buffer, &fil);
    }
    f_close(&fil);
    while(1)
        __NOP();
}
