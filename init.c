#include <asf.h>
#include "init.h"

PortGroup *porta;
PortGroup *portb;
TcCount8 *tc2;
TcCount8 *tc3;
TcCount8 *tc4;
Eic *eic;

void Config_Ports(void)
{
	porta = (PortGroup *) PORT;
	portb = (PortGroup *) PORT + 1;
	tc2 = &(TC2->COUNT8);
	tc3 = &(TC3->COUNT8);
	tc4 = &(TC4->COUNT8);
	eic = (Eic *) EIC;
}

void Simple_Clk_Init(void)
{
	/* Various bits in the INTFLAG register can be set to one at startup.
	   This will ensure that these bits are cleared */
	
	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
			SYSCTRL_INTFLAG_DFLLRDY;
			
	system_flash_set_waitstates(0);  		//Clock_flash wait state =0

	SYSCTRL_OSC8M_Type temp = SYSCTRL->OSC8M;      	/* for OSC8M initialization  */

	temp.bit.PRESC    = 0;    			// no divide, i.e., set clock=8Mhz  (see page 170)
	temp.bit.ONDEMAND = 1;    			// On-demand is true
	temp.bit.RUNSTDBY = 0;    			// Standby is false
	
	SYSCTRL->OSC8M = temp;

	SYSCTRL->OSC8M.reg |= 0x1u << 1;  		// SYSCTRL_OSC8M_ENABLE bit = bit-1 (page 170)
	
	PM->CPUSEL.reg = (uint32_t)0;    		// CPU and BUS clocks Divide by 1  (see page 110)
	PM->APBASEL.reg = (uint32_t)0;     		// APBA clock 0= Divide by 1  (see page 110)
	PM->APBBSEL.reg = (uint32_t)0;     		// APBB clock 0= Divide by 1  (see page 110)
	PM->APBCSEL.reg = (uint32_t)0;     		// APBB clock 0= Divide by 1  (see page 110)

	PM->APBAMASK.reg |= 01u<<3;   			// Enable Generic clock controller clock (page 127)

	/* Software reset Generic clock to ensure it is re-initialized correctly */

	GCLK->CTRL.reg = 0x1u << 0;   			// Reset gen. clock (see page 94)
	while (GCLK->CTRL.reg & 0x1u ) {  /* Wait for reset to complete */ }
	
	// Initialization and enable generic clock #0

	*((uint8_t*)&GCLK->GENDIV.reg) = 0;  		// Select GCLK0 (page 104, Table 14-10)

	GCLK->GENDIV.reg  = 0x0100;   		 	// Divide by 1 for GCLK #0 (page 104)

	GCLK->GENCTRL.reg = 0x030600;  		 	// GCLK#0 enable, Source=6(OSC8M), IDC=1 (page 101)
}

void GPIO_Init(void)
{
	//enabling DC motor
	porta->PINCFG[22].bit.PMUXEN = 0x1;		//Enable multiplexing
	porta->PMUX[11].bit.PMUXE = 0x5;		//TC4 multiplexes by peripheral F
	
	porta->PINCFG[23].bit.PMUXEN = 0x1;		//Enable multiplexing
	porta->PMUX[11].bit.PMUXO = 0x5;		//TC4 multiplexes by peripheral F

	//enabling optical encoder
	porta->PINCFG[28].bit.PMUXEN = 1;		//enable PA28 multiplexing
	porta->PMUX[14].bit.PMUXE = 0x0;			//choose PA28 as EIC output, Peripheral A, EXTINT[8]

	portb->PINCFG[14].bit.PMUXEN = 1;		//enable PB14 multiplexing
	portb->PMUX[7].bit.PMUXE = 0x0;			//choose PB14 as EIC output, Peripheral A, EXTINT[14]
		
	//enabling TC2
	porta->PINCFG[12].bit.PMUXEN = 1;		//enable PA12 multiplexing
	porta->PMUX[6].bit.PMUXE = 0x4;			//choose PA12 as timer, Peripheral E

	//enabling SSD
	//pin B0 - B6 is a - g, pin B9 is the near by red LED
	portb->DIRSET.reg = 0x7F;

	//four digits of the SSD are pin A4 - A7
	porta->DIRSET.reg = 0xF0;
	
	porta->DIRCLR.reg = 0xF0000;
	porta->PINCFG[16].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	porta->PINCFG[17].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	porta->PINCFG[18].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	porta->PINCFG[19].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	
}

void TC2_Init(void)
{
	//TC2 clock initiation
	PM->APBCMASK.reg |= 1u << 10;  	// PM_APBCMASK___for TC2____ is in the _10 bit__ position
	
	uint32_t temp = 0x14;   		// ID for ___TC2 clock_____ is ____0x14______  (see table 14-2)
	temp |= 0<<8;         			//  Selection Generic clock generator 0
	GCLK->CLKCTRL.reg = temp;   		//  Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;    	// enable it.
	
	//TC2 configuration
	tc2->CTRLA.reg	=	(0x1 << 2)	//Mode, 8-bit
					|	(0x0 << 5)	//Wave gen, normal frequency mode
					|	(0x7 << 8)	//Prescaler, DIV1024
					|	(0x1 << 12);//Prescaler sync on prescaler clock
	
	//Fixing period
	tc2->PER.reg = 130;			//count = 130 to get 60 Hz
	
	//Enable overflow interrupt or match interrupt
	tc2->INTENSET.reg = 1u;
	
	//Enable TC
	tc2->CTRLA.reg |= 1u << 1;
}

void TC3_Init(void)
{
	//TC3 clock initiation
	PM->APBCMASK.reg |= 1u << 11;  	// PM_APBCMASK___for TC2____ is in the _10 bit__ position
	
	uint32_t temp = 0x14;   		// ID for ___TC2 clock_____ is ____0x14______  (see table 14-2)
	temp |= 0<<8;         			//  Selection Generic clock generator 0
	GCLK->CLKCTRL.reg = temp;   		//  Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;    	// enable it.
	
	//TC3 configuration
	tc3->CTRLA.reg	=	(0x1 << 2)	//Mode, 8-bit
					|	(0x0 << 5)	//Wave gen, normal frequency mode
					|	(0x6 << 8)	//Prescaler, DIV256
					|	(0x1 << 12);//Prescaler sync on prescaler clock
	
	//Fixing period
	tc3->PER.reg = 0x9C;			//count = 156 to get 200 Hz
	
	//Enable overflow interrupt or match interrupt
	//tc3->INTENSET.reg = 1u;
	
	//Enable TC
	tc3->CTRLA.reg |= 1u << 1;	
}

void TC4_Init(void)
{
	//TC4 clock initiation
	PM->APBCMASK.reg |= 1u << 12;  	// PM_APBCMASK___for TC4____ is in the _12 bit__ position
	
	uint32_t temp = 0x15;   		// ID for ___TC4 clock_____ is ____0x15______  (see table 14-2)
	temp |= 0<<8;         			//  Selection Generic clock generator 0
	GCLK->CLKCTRL.reg=temp;   		//  Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;    	// enable it.
	
	//TC4 configuration
	tc4->CTRLA.reg =	(0x1 << 2)	//Mode, 8-bit
					|	(0x2 << 5)	//Wave gen, normal PWM
					|	(0x0 << 8)	//Prescaler GCLK_TC
					|	(0x1 << 12);//Prescaler sync on prescaler clock
	
	//Fixing period
	tc4->PER.reg = 0xFF;
	
	//Enable TC
	tc4->CTRLA.reg |= 1u << 1;
}

void Int_Init(void)
{
	NVIC->ISER[0] |= 1u << 4 | 1u << 15 | 1u << 16;    //enable TC2, TC3 and EIC lines
	
	//Set priority
	NVIC->IP[1] = 0x00000000;			//Highest priority for EIC
	NVIC->IP[3] = 0x80000000;			//Lowest priority for TC2, system tick
	NVIC->IP[4] = 0x00000040;			//Medium priority for TC3, PID control
}

void EIC_Init(void)
{
	//EIC clock initiation
	PM->APBAMASK.reg |= 1u << 6;	// PM_APBAMASK___EIC____ is in the __6__ position
	
	uint32_t temp = 0x3;  				// ID for GCLK_EIC is EIC_GCLK_ID  (see table 15-5)
	temp |= 0<<8;         				// Selection Generic clock generator 0
	GCLK->CLKCTRL.reg = temp;  			// Setup in the CLKCTRL register
	GCLK->CLKCTRL.reg |= 0x1u << 14;    // enable it.
	
	//EIC configuration
	eic->CTRL.reg = 0;						// DISABLES EIC
	eic->CONFIG[1].bit.SENSE0 = 0x3; 		// both edges detection for phase A

	eic->INTENSET.reg = 1u << 8;			// You must set EXTINT[8] as a target for interrupt
	
	while(eic->STATUS.reg & EIC_STATUS_SYNCBUSY) {} 	// wait for the the EIC to finish sync
	
	eic->CTRL.reg |= 1u << 1;  					// enable EIC
}
