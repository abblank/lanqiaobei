#include "system.h"

static uint8_t task_num, key_now, key_down, key_up, key_old, isRec,
	uart_buff[51]={0}, led_val, led_trig, circum/*工作场景*/, B/*10-100*/=10;
static uint32_t adc_val, freq, pwmA_freq, pwmB_freq, TT/*总共-运行-时长*/, t_key;
static float adc_voltage, R/*1.0-2.0*/=1.0, velocity, pwmA_ratio, pwmB_ratio, TS/*累计行程*/;
const float pi = 3.14;
	#define run_check_interval 30
	static double speed, cur_x, cur_y;

static node_t node_small[15]={0};	//小括号解析，第一个为头节点，dest【0】表示数据个数
static node_t node_big[2]={0};	//大括号解析
static state_t state;
static window_t window;
static node_t* to_node;

void led_ctrl(uint8_t led_val){
	GPIOC->ODR |= 0xFF00;
	GPIOC->BRR = led_val << 8;
	GPIOD->BSRR = BIT(2);
	GPIOD->BRR = BIT(2);
}

void pwm_ctrl(TIM_HandleTypeDef *htimx, int freq, double ratio){
	int arr = 1000000.0f/freq;
	int pulse = arr*ratio;
	__HAL_TIM_SetAutoreload(htimx, arr);
	__HAL_TIM_SetCompare(htimx, TIM_CHANNEL_1, pulse);
}

void led_proc(){
	led_ctrl(led_val^=led_trig);
}

void key_proc(){
	static uint8_t isR=1;
	static uint8_t pressed_key;
	key_now = ~(((GPIOA->IDR&0x01)<<3)|(GPIOB->IDR&0x07)) & 0x0F;
	key_down = key_now & (key_now^key_old);
	key_up = ~key_now & (key_now^key_old);
	key_old = key_now;
	pressed_key |= key_down;
	pressed_key &= ~key_up;
	switch(key_down){
		case 0x01: 
			if(state == Idle){
				if(node_small->next != NULL){
					state = Busy;
					to_node = node_small->next;
				}
			}else if(state == Busy){
				state = Wait;
			}else state = Busy;
			switch(state){
				case Idle: led_trig &= ~0x01; led_val &= ~0x01; break;
				case Busy: led_trig &= ~0x01; led_val |= 0x01; break;
				case Wait: led_trig |= 0x01; break;
				default: break;
			}
			break;
		case 0x02:
			isR = 1;
			if(++window>2) window=DATA;
			break;
		case 0x04:
			if(window == PARA) isR ^= 1;
			break;
		case 0x08:
			if(window == PARA){
				if(isR == 1){
					if((R+=0.1f) > 1.95f){R=1.0f;}
				}else{
					if((B+=10) > 100){B=10;}
				}
			}
			break;
		default: break;
	}
	if(window == DATA && (pressed_key & 0x0C) == 0x0C){
		if(t_key == 0){
			t_key = uwTick;
		}
	}else if(t_key!=0){
		if((uwTick-t_key >= 2000)){
			TS = TT = 0;
		}else {
			t_key = 0;
		}
	}
}

void adc_proc(){
	HAL_ADC_Start(&hadc2);
	adc_val = HAL_ADC_GetValue(&hadc2);
	adc_voltage = adc_val*3.3f/4095.0f;
	if(adc_voltage < 0.5f){
		pwmA_ratio = 0.1f; pwmB_ratio = 0.05f;
	}else if(adc_voltage < 1.0f){
		pwmA_ratio = 0.3f; pwmB_ratio = 0.25f;
	}else if(adc_voltage < 1.5f){
		pwmA_ratio = 0.5f; pwmB_ratio = 0.45f;
	}else if(adc_voltage < 2.0f){
		pwmA_ratio = 0.7f; pwmB_ratio = 0.65f;
	}else if(adc_voltage < 2.5f){
		pwmA_ratio = 0.9f; pwmB_ratio = 0.85f;
	}else{
		pwmA_ratio = 0.95f; pwmB_ratio = 0.90f;
	}
	pwm_ctrl(&htim16, pwmA_freq, state==Busy? pwmA_ratio: 0);
	pwm_ctrl(&htim17, pwmB_freq, state==Busy? pwmB_ratio: 0);
}

void freq_proc(){
	static uint32_t freq_buff[4]={0};
	float freq_temp;
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, freq_buff, 4);
	freq_temp = 0;
	for(int i=0; i<4; ++i){
		freq_temp+=freq_buff[i];
	}
	freq_temp /= 4;
	freq_temp = 80000000.0f/freq_temp;
	freq = freq_temp;
	if(state == Busy){
		velocity = pi*R*freq/100.0f + B;
	}else{
		velocity = 0.0f;
	}
}

int analysis(node_t* node){
	node[0].next = NULL;	//node0为头节点，表示无数据
	int i = 1;	//从数字开始解析
	int tuple = 1; //有n-1对数据	(第一个为头节点，不记录数据)
	while(1){
		int x = 0;
		while(uart_buff[i] != ','){	//解析出一个数字
			if(uart_buff[i] >= '0' && uart_buff[i] <= '9'){
				x*=10;
				x+=uart_buff[i]-'0';
				++i;
			}else return 1; //error
		}
		if(uart_buff[i+1]<'0' || uart_buff[i+1]>'9') return 1; //error,必须是x，y对
		int y = 0;
		++i; //数字
		while(uart_buff[i] != ',' && uart_buff[i] != ')' && uart_buff[i] != '}'){	//解析出一个数字
			if(uart_buff[i] >= '0' && uart_buff[i] <= '9'){
				y*=10;
				y+=uart_buff[i]-'0';
				++i;
			}else return 1; //error
		}//正确解析完成
		node[tuple].dest_val[0]=x;
		node[tuple].dest_val[1]=y;		
		if(uart_buff[i] == ','){node[tuple].next = &(node[tuple+1]); ++tuple; i++;}
		else{node[tuple].next = NULL; node[0].next=&(node[1]); node[0].dest_val[0]=tuple; return 0;}	//全部解析完成
	}
}

int analysis_del(node_t* node){
	node[0].next = NULL;	//node0为头节点，表示无数据
	int i = 1;	//从数字开始解析
	int tuple = 1; //有n-1对数据	(第一个为头节点，不记录数据)
	int x = 0;
	while(uart_buff[i] != ','){	//解析出一个数字
		if(uart_buff[i] >= '0' && uart_buff[i] <= '9'){
			x*=10;
			x+=uart_buff[i]-'0';
			++i;
		}else return 1; //error
	}
	if(uart_buff[i+1]<'0' || uart_buff[i+1]>'9') return 1; //error,必须是x，y对
	int y = 0;
	++i; //数字
	while(uart_buff[i] != ',' && uart_buff[i] != ')' && uart_buff[i] != '}'){	//解析出一个数字
		if(uart_buff[i] >= '0' && uart_buff[i] <= '9'){
			y*=10;
			y+=uart_buff[i]-'0';
			++i;
		}else return 1; //error
	}//正确解析完成
	node[tuple].dest_val[0]=x;
	node[tuple].dest_val[1]=y;		
	if(uart_buff[i] == '}'){node[tuple].next = NULL; node[0].next=&(node[1]); node[0].dest_val[0]=tuple; return 0;} //全部解析完成
	else{return 1;}	
}

int try_del(){
	int x=node_big[1].dest_val[0];
	int y=node_big[1].dest_val[1];
	node_t* pnode;
	node_t* pnode0;
	if(node_small[0].next != NULL){
		pnode0 = &(node_small[0]);
		pnode = node_small[0].next;
	}else return 1;
	do{
		if(pnode->dest_val[0] == x && pnode->dest_val[1] == y){
			if(to_node == pnode){
				to_node = pnode->next;
			}
			pnode0->next = pnode->next;	//删除节点pnode
			node_small->dest_val[0]--;
			return 0; //succeed
		}
		pnode0 = pnode0->next;
	}while((pnode = pnode->next) != NULL);
	return 1;
}

void uart_proc(){
	if(isRec == 1){
		if(uart_buff[0] == '('){
			if(!analysis(node_small)){
				printf("Got it");
			}
		}else if(uart_buff[0] == '{'){
			if(!analysis_del(node_big)){

				if(try_del()){
					printf("Nonexistent");
				}else printf("Got it");
			}
		}else if(uart_buff[0] == '[' && uart_buff[2] == ']' && uart_buff[1] >= '0' && uart_buff[1] <= '9'){
			if(state == Busy){
				circum = uart_buff[1] - '0';
				printf("Got it");
				switch(circum){
					case 1: pwmA_freq = 1000; pwmB_freq = 1000; led_val &= 0x0F; led_val |= 0x10;
					break;
					case 2: pwmA_freq = 4000; pwmB_freq = 1000; led_val &= 0x0F; led_val |= 0x20;
					break;
					case 3: pwmA_freq = 1000; pwmB_freq = 4000; led_val &= 0x0F; led_val |= 0x40;
					break;
					case 4: pwmA_freq = 4000; pwmB_freq = 4000; led_val &= 0x0F; led_val |= 0x80;
					break;
					default: break;
				}
				pwm_ctrl(&htim16, pwmA_freq, state==Busy? pwmA_ratio: 0);
				pwm_ctrl(&htim17, pwmB_freq, state==Busy? pwmB_ratio: 0);
			}else printf("Device offline");
		}else if(uart_buff[0] == '?' && uart_buff[1] == '\0'){
			switch(state){
				case Idle: printf("Idle"); 
				break;
				case Busy: printf("Busy"); 
				break;
				case Wait: printf("Wait"); 
				break;
				default: break;
			}
		}else if(uart_buff[0] == '#' && uart_buff[1] == '\0'){
			printf("(%.0lf,%.0lfd)", cur_x, cur_y);
		}else printf("error");
		memset(uart_buff, 0, 50);
		isRec = 0;
	}
}

void lcd_proc(){
	switch(window){
		case DATA: 
			lcd_printf(Line1, "        DATA        ");
			switch(state){
				case Idle: lcd_printf(Line3, "     ST=%s           ", "Idle"); 
				break;
				case Busy: lcd_printf(Line3, "     ST=%s           ", "Busy"); 
				break;
				case Wait: lcd_printf(Line3, "     ST=%s           ", "Wait"); 
				break;
				default: break;
			}
			lcd_printf(Line4, "     CP=%.0lf,%.0lf           ", cur_x, cur_y); 
			if(state != Idle){
				lcd_printf(Line5, "     TP=%d,%d           ", to_node->dest_val[0], to_node->dest_val[1]); 
			}else lcd_printf(Line5, "     TP=NF           "); 
			lcd_printf(Line6, "     SE=%.1f           ", velocity); 
			if(state != Idle){
				lcd_printf(Line7, "     RN=%d           ", node_small->dest_val[0]); 
			}else lcd_printf(Line7, "     RN=NF           "); 
		break;
			
		case PARA:
			lcd_printf(Line1, "        PARA        ");
			lcd_printf(Line3, "     R=%.1f           ", R); 
			lcd_printf(Line4, "     B=%d           ", B); 
			lcd_printf(Line5, "                    ");
			lcd_printf(Line6, "                    "); 
			lcd_printf(Line7, "                    "); 		
		break;
		
		case RECD: 
			lcd_printf(Line1, "        RECD        ");
			lcd_printf(Line3, "     TS=%.1f           ", TS); 
			lcd_printf(Line4, "     TT=%d           ", TT/1000); 
		break;
		default: break;
	}
}

void TT_proc(){
	static uint32_t last;
	if(state == Busy){
		if(last!=0){
			TT += uwTick-last;
			last = uwTick;
		}
	}else{
		last = uwTick;
	}
}

double abs_double(double para){
	return para>=0? para: -para;
}

void run_proc(){
	double theta;
	static double x_last=9000000;
	if(state == Busy){
		speed = velocity*run_check_interval/1000.0f;
		theta = atan2(((double)to_node->dest_val[1]-cur_y),((double)to_node->dest_val[0]-cur_x));
		double cos_=speed*cos(theta), sin_=speed*sin(theta);
		cur_x += cos_;
		cur_y += sin_;
		TS += speed;
		if(!(abs_double((double)to_node->dest_val[0]-cur_x) < x_last)){
			cur_x = to_node->dest_val[0];
			cur_y = to_node->dest_val[1];
			to_node = to_node->next;
			node_small->next = to_node;
			node_small->dest_val[0] -= 1;
			if(node_small->next == NULL){state = Idle; led_val &= ~0x01;}
			x_last = abs_double((double)to_node->dest_val[0]-cur_x);
		}else{
			x_last = abs_double((double)to_node->dest_val[0]-cur_x);
		}
	}
}

task_t task[] = {
	{uart_proc, 50, 0},
	{adc_proc, 50, 0},
	{freq_proc, 50, 0},
	{key_proc, 10, 0},
	{led_proc, 200, 0},
	{lcd_proc, 100, 0},
	{TT_proc, 50, 0},
	{run_proc, run_check_interval, 0},
};

void system_init(){
	LCD_Init();
	LCD_SetTextColor(White);
	LCD_SetBackColor(Black);
	LCD_Clear(Black);
	task_num = sizeof(task)/sizeof(task_t);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_buff, 50);
	__HAL_DMA_DISABLE_IT((&huart1)->hdmarx, DMA_IT_HT);
	led_ctrl(0x00);
}

void scheduler(){
	for(int i=0; i<task_num; ++i){
		if(task[i].interval+task[i].last <= uwTick){
			task[i].last = uwTick;
			task[i].function();
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	isRec = 1;
	HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_buff, 50);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

int fputc(int ch, FILE* f){
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
	return ch;
}
