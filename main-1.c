#include <asf.h>
#include "init.h"

#define DEBOUNCE_N 200

#define MIN_DUTY 0x3
#define MAX_DUTY 0xFB
#define MIN_RPM 0
#define MAX_RPM 4900

#define MODE_SPD 0
#define MODE_POS 1

#define K_P_SPD 0.05
#define K_I_SPD 0.1
#define K_D_SPD 0.0

#define K_P_POS 2.0
#define K_I_POS 1.0
#define K_D_POS 0.05

unsigned char buffer[4];

float sumErrorRPM, sumErrorPOS, oldErrorPOS;

int absolute(int);
void Add_Digit(unsigned char);
int Get_Digit(unsigned char);
void Num_To_Buff(int);
int Buff_To_Num(void);
int maps(int, int, int, int, int);

int setRPM, setPOS, measuredRPM[2], measuredPOS[2], count, mode;

int main(void)
{
	//int referenceValue, measurementValue, inputValue;
	Config_Ports();
	Simple_Clk_Init();
	GPIO_Init();
	TC2_Init();
	TC3_Init();
	TC4_Init();
	EIC_Init();
	Int_Init();
	
	while (1)
	{};
		
	return 0;
}

void TC2_Handler(void)
{
	static int system_state = 1, goal, key = 16;
	static unsigned char grid[17] = {	'1','2','3','A',
										'4','5','6','B',
										'7','8','9','C',
										'*','0','#','D', 0x0};
	int i, cnt, row, col, temp = 16, number;
	
	for (row = 0; row < 4; row++)
	{
		porta->OUTCLR.reg = 0x80 >> row;
		portb->OUTCLR.reg = Get_Digit(buffer[row]);
		
		for (col = 0; col < 4; col++)
		{
			cnt = 0;
			for (i = 0; i < DEBOUNCE_N; i++)
				if (porta->IN.reg & 0x80000 >> col)		cnt++;
			if (cnt >= DEBOUNCE_N)		key = temp = 4*row + col;
		}
	
		porta->OUTSET.reg = 0x80 >> row;
		portb->OUTSET.reg = 0x7F;
	}


	if (temp == 16)
	{
		switch (grid[key])
		{
			case '0'...'9':
							Add_Digit(grid[key]);
							break;
			case 'A':
							goal = 1;
							break;
			case 'B':	
							goal = 3;
							number = Buff_To_Num();
							setRPM = (number > MAX_RPM)? MAX_RPM: number;
							break;
			case 'C':
							goal = 5;
							number = Buff_To_Num();
							setPOS = (number % 360);
							sumErrorPOS = 0;
							oldErrorPOS = 0;
							break;
			case 'D':		//reset reference degree
							goal = 5;
							count = 0;
							setPOS = 0;
							sumErrorPOS = 0;
							oldErrorPOS = 0;
							break;
		}
		
		key = 16;
	}
	
	switch (system_state)
	{
		case 1:	//Idle
				tc4->CC[0].reg = MIN_DUTY;
				tc4->CC[1].reg = MIN_DUTY;
								
				if (goal == 3)
				{
					system_state = 2;
					tc3->INTENSET.reg = 1u;
				}
				else if (goal == 5)
				{
					system_state = 5;
					tc3->INTENSET.reg = 1u;
				}
					
				break;
				
		case 2:	//Acceleration
				mode = MODE_SPD;		
				if (goal == 1 || goal == 5)
					system_state = 4;
				else if ((setRPM - measuredRPM[0] < 5) && goal == 3)
					system_state = 3;
				break;
				
		case 3:	//Spd_ctrl
				if (goal == 1 || goal == 5)
					system_state = 4;
				break;
				
		case 4:	//Deceleration
				setRPM = 0;
				if (goal == 3)
					system_state = 2;
				else if (measuredRPM[0] == 0 && goal == 1)
					{
						system_state = 1;
						tc3->INTENCLR.reg = 1u;		
						Num_To_Buff(0);				
					}
				else if (measuredRPM[0] == 0 && goal == 5)
					{
						system_state = 5;
						count = 0;
					}
				break;
				
		case 5:	//Pos_ctrl
				mode = MODE_POS;
				if (goal == 3)
				{
					system_state = 2;
					count = 0;
				}
				else if (goal == 1)
					{
						system_state = 1;
						tc3->INTENCLR.reg = 1u;
						Num_To_Buff(0);
					}
				break;
	}
	tc2->INTFLAG.bit.OVF = 1;
}

//PID control
void TC3_Handler(void)
{
	static float displayRPM[2];
	int command, error;
	
	switch (mode)
	{
		case MODE_SPD:
					//1Hz low pass filter with 200 Hz sampling frequency
					//y = b0*y1 + a0*u1
					//a0 = 2*pi*1 Hz/200 Hz = 0.0314; b0 = 1 - a0
					displayRPM[0] = 0.9686*displayRPM[1] + 0.0314*measuredRPM[1];
		
					//convert to buffer
					Num_To_Buff((int) displayRPM[0]);
		
					//Delay values
					displayRPM[1] = displayRPM[0];
					measuredRPM[1] = measuredRPM[0];
					
					error = setRPM - measuredRPM[0];
						
					//////PPPPPPPPPPPPPPPPPPPP///IIIIIIIIIIIIIIIII
					command = K_P_SPD * error + K_I_SPD * sumErrorRPM;
						
					sumErrorRPM = sumErrorRPM + error / 200.;
					
					tc4->CC[1].reg = (command > MAX_DUTY)? MAX_DUTY: command;
					
					//RPM = nF * 60 / N
					//Using only phase A detecting both edges, so N = 780
					//200Hz * 60 / 800 = 15.
					//F = 200 Hz
					measuredRPM[0] = count * 15.;

					//Reset count for encoder
					count = 0;
					
					break;
		
		case MODE_POS:
					measuredPOS[0] = (count % 770) * 360. / 770.; //converts to degree
		
					//Num_To_Buff(measuredPOS[0]);
					
					error = setPOS - measuredPOS[0];
								
					//////PPPPPPPPPPPPPPPPPPPPPP///IIIIIIIIIIIIIIIIIIIIIIIIIIIII///DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
					command = K_P_POS * error + K_I_POS * sumErrorPOS + K_D_POS * (error - oldErrorPOS) * 200.;
										
					if (absolute(command) > MAX_DUTY)
						sumErrorPOS = sumErrorPOS;
					else
						sumErrorPOS = sumErrorPOS + error * 0.005;
					
					Num_To_Buff(absolute(measuredPOS[0]));
					
					oldErrorPOS = error;
										
					if (command <= 0)
					{
						tc4->CC[0].reg = (absolute(command) > MAX_DUTY)? 0xFB: absolute(command); //(absolute(command) > 0xFB)? 0xFB: absolute(command);
						tc4->CC[1].reg = 0;
					}
					else if (command > 0)
					{
						tc4->CC[1].reg = (absolute(command) > MAX_DUTY)? 0xFB: absolute(command);
						tc4->CC[0].reg = 0;
					}
				
					/*
					command = pid_Controller(setPOS, measuredPOS[0], &pidPOS);

					if (count == 0)
					{
						tc4->CC[0].reg = MIN_DUTY;
						tc4->CC[1].reg = MIN_DUTY;
					}
					else
					if (count > 0)
					{
						tc4->CC[0].reg = maps(command, 780, 0, MAX_DUTY, MIN_DUTY);
						tc4->CC[1].reg = MIN_DUTY;
					}
					else
					{
						tc4->CC[1].reg = maps(command, 780, 0, MAX_DUTY, MIN_DUTY);
						tc4->CC[0].reg = MIN_DUTY;
					}*/
					break;
	}
	
	tc3->INTFLAG.bit.OVF = 1u;
}

void EIC_Handler(void)
{
	int checkA, checkB;
	
	checkA = porta->IN.reg & 1u << 28;
	checkB = portb->IN.reg & 1u << 14;
	
	//based on phase A
	if (checkA)
	{
		if (checkB)
			count++;
		else
			count--;
	}
	else
	{
		if (checkB)
			count--;
		else
			count++;
	}
	
	eic->INTFLAG.reg = 1u << 8;	// Clear EIC interrupt
}

void Add_Digit(unsigned char key)
{
	for (int i = 0; i < 3; i++)
		buffer[i] = buffer[i+1];
	buffer[3] = key;
}

int Get_Digit(unsigned char key)
{
	switch (key)
	{
		case '0': return 0b0111111;
		case '1': return 0b0000110;
		case '2': return 0b1011011;
		case '3': return 0b1001111;
		case '4': return 0b1100110;
		case '5': return 0b1101101;
		case '6': return 0b1111101;
		case '7': return 0b0000111;
		case '8': return 0b1111111;
		case '9': return 0b1100111;
		default : return 0b0000000;
	}
}

void Num_To_Buff(int num)
{
	for (int i = 0; i < 3; i++)
	{
		buffer[3-i] = (num) % 10 + 48;
		(num) /= 10;
	}
	buffer[0] = (num) % 10 + 48;
}

int Buff_To_Num(void)
{
	int num = 0;
	
	num += (buffer[0])? (buffer[0] - 48) * 1000 : 0;
	num += (buffer[1])? (buffer[1] - 48) * 100 : 0;
	num += (buffer[2])? (buffer[2] - 48) * 10 : 0;
	num += (buffer[3])? (buffer[3] - 48) : 0;

	return num;
}

int maps(int orig, int oldMax, int oldMin, int newMax, int newMin)
{
	return ((orig - oldMin)*(newMax - newMin)/(oldMax - oldMin) + newMin);
}

int absolute(int number)
{
	return (number >=0)? number : -number;
}