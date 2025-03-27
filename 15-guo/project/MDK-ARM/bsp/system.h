#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include "math.h"

#pragma diag_suppress 1295
#define lcd_printf(LINE, STR, ...) do{\
	char buff[21] = {0};\
	snprintf(buff, 20, STR, ##__VA_ARGS__);\
	LCD_DisplayStringLine(LINE, (uint8_t*)buff);\
}while(0);
#define BIT(x) (0x01 << (x))
#define LED(x) (0x01 << ((x)-1))

typedef struct __task{
	void (*function)(void);
	int interval;
	int last;
}task_t;

typedef struct node{
	int dest_val[2];
	struct node* next; 
}node_t;

typedef enum {
	Idle,
	Busy,
	Wait
}state_t;

typedef enum {
	DATA,
	PARA,
	RECD
}window_t;


void system_init();
void scheduler();

#endif
