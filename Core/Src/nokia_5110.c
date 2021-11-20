/*
 * nokia_5110.c
 *
 *  Created on: Oct 1, 2021
 *      Author: a6gue
 */

#include "main.h"
#include "nokia_5110.h"

void LCD_Write(xmit_type type, SPI_TypeDef *instance, uint8_t data)
{
    if(type == COMMAND)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    	SPI_WriteByte(instance, &data);
    }
    else if(type == DATA)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        SPI_WriteByte(instance, &data);
    }
}

void Nokia5110_SetCursor(SPI_TypeDef *instance, unsigned char newX, volatile unsigned char newY){
  if((newX > 80) || (newY > 5)){        // Error handling
    return;
  }
  LCD_Write(COMMAND, instance, 0x80|(newX));     // setting bit 7 updates X-position
  LCD_Write(COMMAND, instance, 0x40|(newY));     // setting bit 6 updates Y-position
}

void Clear_Display(SPI_TypeDef *instance)
{
  Nokia5110_SetCursor(instance,0,0);
  int idx1, idx2;
  for(idx2 = 0; idx2 < 6; ++idx2)
  {
    for(idx1 = 0; idx1 < 84; ++idx1)
    {
      LCD_Write(DATA, instance, 0x00);
    }
  }
  Nokia5110_SetCursor(instance,0,0);
}

void Nokia_Reset()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	for(int i = 0; i < 100; ++i);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void SPI_WriteByte(SPI_TypeDef *instance, uint8_t *data)
{
	uint8_t volatile dummy;
    while((instance->SR & 0x02) == 0);
    *((__IO uint8_t *)&instance->DR) = *data;
    while ((instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
    dummy = *((__IO uint8_t *)&instance->DR);
    while((instance->SR & SPI_SR_BSY));
}

void SPI_ReadByte(SPI_TypeDef *instance, uint8_t *val)
{
	while ((instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
	*val = *((__IO uint8_t *)&instance->DR);
	//*val = *((__IO uint8_t *)&instance->DR);
	while ((instance->SR & SPI_SR_BSY));
}

void Nokia_Config(SPI_TypeDef *instance)
{
  Nokia_Reset();
  LCD_Write(COMMAND, instance, 0x21);
  LCD_Write(COMMAND, instance, 0xC0);                // Contrast (Vop0 to Vop6) pp.14
  LCD_Write(COMMAND, instance, 0x04);                // Temperature Coeffecient pp.14
  LCD_Write(COMMAND, instance, 0x14);                // LCD bias mode
  LCD_Write(COMMAND, instance,
	    0x20 | (0<<PD | 0 << V | 0 << H)); // PD = 0, V = 0, H = 0
                                           // Regular Instruction set
  LCD_Write(COMMAND, instance, 0x0C);                // Display Control normal

  // Set At Origin.
  LCD_Write(COMMAND, instance, 0x80);
  LCD_Write(COMMAND, instance, 0x40);
}

//********Nokia5110_OutString*****************
// Print a string of characters to the Nokia 5110 48x84 LCD.
// The string will automatically wrap, so padding spaces may
// be needed to make the output look optimal.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutString(SPI_TypeDef *instance, char *ptr){
  while(*ptr){
    Nokia5110_OutChar(instance, (unsigned char)*ptr);
    ptr = ptr + 1;
  }
}

void Nokia5110_OutChar(SPI_TypeDef *instance, unsigned char data){
  int i;
  LCD_Write(DATA, instance, 0x00);                 // blank vertical line padding
  for(i=0; i<5; i=i+1){
    LCD_Write(DATA, instance, ASCII[data - 0x20][i]);
  }
  LCD_Write(DATA, instance, 0x00);                 // blank vertical line padding
}
