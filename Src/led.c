/*
 * led.c
 *
 *  Created on: Jan 18, 2024
 *      Author: kacpa
 */

#include "led.h"


bool ledmatrix[32][8];
uint32_t ledbuf[40+256*24] = {0};


// Initializing LED matrix by setting all values to false and clearing color
void LedInit(uint8_t rowsnb, uint8_t colnb)
{
	  for (int i = 0; i < 32; i++)
	  {
	  	  for (int j = 0; j < 8; j++)
	  	  {
	  		  ledmatrix[i][j] = false;
	  	  }
	  }

	  for (int i = 0; i < 256 * 24; i++)
	  {
		  ledbuf[40 + i] = 32;
	  }

	  clearmatrix();
	  clearbuf();
}


// Prepare pack of bytes to given colors
void setcolor(int position, uint8_t color_green, uint8_t color_red, uint8_t color_blue)
  {
	  //GREEN
	  for (int i = 0; i < 8; i++)
	  {
		  if(color_green & 0x80)
			  ledbuf[40 + i + position * 24] = 64;
		  else
			  ledbuf[40 + i + position * 24] = 32;
		  color_green <<= 1;
	  }
	  //RED
	  for (int i = 0; i < 8; i++)
	  {
		  if(color_red & 0x80)
			  ledbuf[40 + i + position * 24 + 8] = 64;
		  else
			  ledbuf[40 + i + position * 24 + 8] = 32;
		  color_red <<= 1;
	  }
	  //BLUE
	  for (int i = 0; i < 8; i++)
	  {
		  if(color_blue & 0x80)
			  ledbuf[40 + i + position * 24 + 16] = 64;
		  else
			  ledbuf[40 + i + position * 24 + 16] = 32;
		  color_blue <<= 1;
	  }

  }


// Updating buffor
void updatebuf()
  {
	  for (int i = 0; i < 32; i++)
	  {
		  if(i%2 == 0)
			  for (int j = 0; j < 8; j++)
			  {
				  if(ledmatrix[i][j] == true)
				  {
					  if(j < 4)
						  setcolor(j + 8 * i, 0b11110000, 0, 0);
					  else if(j >= 4 && j < 7)
						  setcolor(j + 8 * i, 0b11110000, 0b11110000, 0);
					  else
						  setcolor(j + 8 * i, 0, 0b11110000, 0);
				  }
			  }
		  else
		  {
			  int jj = 0;
			  for(int j = 7; j >= 0; j--)
			  {
				  if(ledmatrix[i][j] == true)
				  {
					  if(j < 4)
						  setcolor(jj + 8 * i, 0b11110000, 0, 0);
					  else if(j >= 4 && j < 7)
						  setcolor(jj + 8 * i, 0b11110000, 0b11110000, 0);
					  else
						  setcolor(jj + 8 * i, 0, 0b11110000, 0);
				  }
				  jj++;
			  }
		  }
	  }
  }

// Clearing matrix
void clearmatrix()
  {
	  for (int i = 0; i < 32; i++)
	  {
		  for (int j = 0; j < 8; j++)
			  ledmatrix[i][j] = false;
	  }
	  updatebuf();
  }

// Clearing buffor
void clearbuf()
  {
	  for (int i = 0; i < 256 * 24; i++)
	  	{
		  ledbuf[40 + i] = 32;
		}
  }

// Main function, choose what stripe number and set given amplitude up to eight
void setstripe(int stripe_number, int amp)
  {
	  switch(stripe_number)
	  {
	  case 1:
		  for (int i = 0; i < amp; i++)
		  {
			  ledmatrix[0][i] = true;
			  ledmatrix[1][i] = true;
		  }
		  break;

	  case 2:
		  for (int i = 0; i < amp; i++)
		  {
			  ledmatrix[3][i] = true;
			  ledmatrix[4][i] = true;
		  }
		  break;

	  case 3:
		  for (int i = 0; i < amp; i++)
		  {
			  ledmatrix[6][i] = true;
			  ledmatrix[7][i] = true;
		  }
		  break;

	  case 4:
		  for (int i = 0; i < amp; i++)
		  {
			  ledmatrix[9][i] = true;
			  ledmatrix[10][i] = true;
		  }
		  break;

	  case 5:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[12][i] = true;
	  			  ledmatrix[13][i] = true;
	  		  }
	  		  break;
	  case 6:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[15][i] = true;
	  			  ledmatrix[16][i] = true;
	  		  }
	  		  break;
	  case 7:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[18][i] = true;
	  			  ledmatrix[19][i] = true;
	  		  }
	  		  break;
	  case 8:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[21][i] = true;
	  			  ledmatrix[22][i] = true;
	  		  }
	  		  break;

	  case 9:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[24][i] = true;
	  			  ledmatrix[25][i] = true;
	  		  }
	  		  break;

	  case 10:
	  		  for (int i = 0; i < amp; i++)
	  		  {
	  			  ledmatrix[27][i] = true;
	  			  ledmatrix[28][i] = true;
	  		  }
	  		  break;

	  case 11:
			  for (int i = 0; i < amp; i++)
			  {
				  ledmatrix[30][i] = true;
				  ledmatrix[31][i] = true;
			  }
			  break;


	  }
	  updatebuf();
  }
