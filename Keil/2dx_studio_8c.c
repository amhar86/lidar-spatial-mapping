
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;  //0b00000000    								      // Make PM0 and PM1 inputs 
  GPIO_PORTH_DEN_R = 0b00001111;  //0b00000011
	return;
}
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	
	uint32_t delay = 1; //us
	int count_45_CW = 0;
	int count_on=0;
	int LED_flag = 0;
	int on = 0b0;
	int clockdir = 0b0; //0 is clockwise, 1 is counter clockwise
	
	// hello world!
	/////UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	/////UART_printf(printf_buffer);
	



/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	/////UART_printf(printf_buffer);

	// 1 Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	/////UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
	status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
	sprintf(printf_buffer, "(Model_ID)=0x%x\r\n", byteData);
	/////UART_printf(printf_buffer); 
  status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC)
	sprintf(printf_buffer, "(Model_Type)=0x%x\r\n", byteData);
	/////UART_printf(printf_buffer); 
  status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type
	sprintf(printf_buffer, "(Model_ID, Model_Type)=0x%x\r\n", wordData);
	/////UART_printf(printf_buffer); 
	
	// Get the Distance Measures 50 times
	for(int i = 0; i < 24; i++) {
		
      //5 wait until the ToF sensor's data is ready
	  on ^= 0b1;
		
		
		while (on){
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait10ms(delay);
					count_45_CW++;
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait10ms(delay);
					count_45_CW++;
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait10ms(delay);
					count_45_CW++;
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait10ms(delay);
					count_45_CW++;
				
					if ((count_45_CW % 256) == 0) on^= 0b1;
		}
		
	  
		//7 read the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
    
		FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART
		//sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
		//sprintf(printf_buffer,"%u, %u\r\n", RangeStatus, Distance);
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
		}
  
  
	VL53L1X_StopRanging(dev);
  while(1){}


}
