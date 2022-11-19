/*

The address of the PCF8574 is 0 1 0 0 A2 A1 A0 R/W. To change the address we are provided with A0, A1 and A2 pins.
By default these three pins are high so the address by default is 01001110 which is 0x4E.  
To change the address of this device, you have to connect any/all of these three pins to ground, which is provided just above them. 

*/

/*
    The above function will send the command to the device to which our LCD is connected.
    As we are using 4 bit LCD mode, we have to send command in two parts.
    First we send the upper nibble, and than the lower one. Both parts are sent along enable pin 1 and than with enable pin 0.
    When the data_t is OR(|) with 0x0C, which implies that P2 (En) pin is high and P0 (RS), P1(R/W) are low for the command and write operation.
    In the second case data is sent along with 0x08 only. This is to ensure that the back light is on and En, RS, R/W are all low. 
*/
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#include "lcd-1.h"
#include "stm32f4xx_hal.h"
//#include "main.c"
extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart2;


void lcd_send_cmd (char cmd)
{

  char data_u, data_l;
    uint8_t data_t[4]; // 4 times 8bit integer
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0    0x0C = 00001100
	data_t[1] = data_u|0x08;  //en=0, rs=0    0x08 = 00001000
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0  this is to ensure the backlight is on
	//for ( int var = 0; var < 4; ++var) {
	//	HAL_UART_Transmit(&huart2, data_t[var], sizeof(data_t), 10000);
//	}

	//if(HAL_I2C_IsDeviceReady(&hi2c1, SLAVE_ADDRESS_LCD, 3, 100)!=HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *)data_t,sizeof(data_t),HAL_MAX_DELAY);
		//HAL_Delay(100);
}


/*
    The above function sends the data to the device to which our LCD is connected.
    Similarly like command, We have to send data in two parts, first upper and than lower half.
    Both parts are sent along enable pin 1 and than with enable pin 0. Data is OR(|) with 0x0D which implies that P2 (En) and P0 (RS) pin are high , P1(R/W) is low and back light is on, for the data and write operation.
    In second case data is OR(|) with 0x09 to make only RS pin high and for the back light, and all others low.   
*/


void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	//if(HAL_I2C_IsDeviceReady(&hi2c1, SLAVE_ADDRESS_LCD, 3, 100)!=HAL_OK)
		HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t*) data_t,sizeof(data_t),HAL_MAX_DELAY);
		//HAL_Delay(100);
}


//As according to the datasheet of the LCD 16×2, in order to initialize the LCD, we have to use some sequence of commands. The code is commented properly, so that you can understand it better

void lcd_init (void)
{
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	// 4 bit initialisation
	//HAL_Delay(100);
	delay_ms(100);  // wait for >40ms
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x30); //0011 0000
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	delay_ms(5);  // wait for >4.1ms
	//HAL_Delay(5);
	lcd_send_cmd(0x30);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	delay_ms(1);  // wait for >100us
	//HAL_Delay(1);
	lcd_send_cmd(0x30);
	delay_ms(10);
	//HAL_Delay(10);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x20);  // 4bit mode // 0010 0000
	delay_ms(10);
	//HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	//HAL_Delay(1);
	delay_ms(1);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	//HAL_Delay(1);
	delay_ms(1);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x01);  // clear display

	//HAL_Delay(4);
	delay_ms(4);
	//HAL_Delay(1);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	//HAL_Delay(1);
	delay_ms(1);
	//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
	lcd_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

	//HAL_Delay(1);
	delay_ms(1);
}

// function used to send string to the lcd
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);

}


void CreateCustomCharacter (unsigned char *Pattern, const char Location)
	{
int i=0;
lcd_send_cmd (0x40+(Location*8));     //Send the Address of CGRAM
for (i=0; i<8; i++)
lcd_send_data (Pattern [ i ] );         //Pass the bytes of pattern on LCD
}

void lcd_set_cursor(unsigned char row, unsigned char column){
unsigned char dram = 0x80;
	unsigned char pos = column+(row*50);
	lcd_send_cmd(dram|pos);

}

void LCD_Set_Cursor(unsigned char r, unsigned char c)
{
    unsigned char Temp,Low4,High4;
    if(r == 1)
    {
      Temp  = 0x80 + c - 1; //0x80 is used to move the cursor
      High4 = Temp >> 4;
      Low4  = Temp & 0x0F;
      lcd_send_cmd(High4);
      lcd_send_cmd(Low4);
    }
    if(r == 2)
    {
      Temp  = 0xC0 + c - 1;
      High4 = Temp >> 4;
      Low4  = Temp & 0x0F;
      lcd_send_cmd(High4);
      lcd_send_cmd(Low4);
    }
    //HAL_Delay(2);
    delay_ms(2);
}

void lcd_clear(void){

	lcd_send_cmd(LCD_CLEARDISPLAY);
	//HAL_Delay(10);
	delay_ms(10);
}


