#ifndef MY_INIT_H
#define MY_INIT_H

extern PortGroup *porta;
extern PortGroup *portb;
extern TcCount8 *tc2;
extern TcCount8 *tc3;
extern TcCount8 *tc4;
extern Eic *eic;

void Config_Ports(void);
void Simple_Clk_Init(void);
void GPIO_Init(void);
void TC2_Init(void);
void TC3_Init(void);
void TC4_Init(void);
void EIC_Init(void);
void Int_Init(void);

#endif