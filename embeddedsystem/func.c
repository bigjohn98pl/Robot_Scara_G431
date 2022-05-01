#include "func.h"

const uint16_t steps_per_obort =  6400;
double deg_per_step = 360.00/(double)steps_per_obort;
uint16_t rot_time = 60000000/steps_per_obort;

double new_X = 330,	new_Y = 0,	new_Z = 70;
uint16_t new_grab = OPEN;
double t_J1 = 0,	t_J2 = 0;
int steps1 = 0,	steps2 = 0 ;

u_int8_t sekwencja_save 	= false;
u_int8_t sekwencja_start 	= false;
u_int8_t z_limit_flag 		= false;
u_int8_t J2_limit_flag 		= false;
u_int8_t move_done 			= true;

static uint8_t UART_OUT[N_OUT];
static uint8_t UART_IN[N_IN];
uint16_t size =0;
char *pEnd=NULL;

/*
 * Funkcja wywołująca się przy zakończeniu odbierania ramki danych,
 * po zakończeniu funkcja analizuje buffor, wyciąga z niego dane, i wysyła dane z powrotem
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(UART_IN[0] == ' ' && UART_IN[1] == ' '){
		if(UART_IN[26]=='M'){
			set_settings(&Rob.M[0]);
			if(UART_IN[27]=='1'){
				set_settings(&Rob.M[0]);
			}
			if (UART_IN[27]=='2') {
				set_settings(&Rob.M[1]);
			}
			if (UART_IN[27]=='3') {
				set_settings(&Rob.M[2]);
			}
		}
		else{
			new_X = strtod((char*)UART_IN,&pEnd);
			new_Y = strtod(pEnd,&pEnd);
			new_Z = strtod(pEnd,&pEnd);
			new_grab = strtol(pEnd,NULL,0);
		}
	}
	if(UART_IN[0]=='+'){
		sekwencja_save = true;
	}
	if(UART_IN[0]=='-'){
		sekwencja_save = false;
	}
	if(UART_IN[1]=='+'){
		sekwencja_start = true;
	}
	if(UART_IN[1]=='-'){
		sekwencja_start = false;
	}
	if(UART_IN[2]=='-'){
		clear_sequence();
	}


	HAL_UART_Receive_DMA(&huart2,UART_IN, N_IN);
	if(move_done == true){

		size = sprintf((char*)UART_OUT,	"X %7.2f %7.2f %7.2f %4d %7.2f %7.2f \r\n",Rob.X,Rob.Y,Rob.Z,Rob.grab,Rob.J1,Rob.J2); //50
		HAL_UART_Transmit_DMA(&huart2, UART_OUT, N_OUT);
	}
	else if(move_done == false){
		size = sprintf((char*)UART_OUT, "M %10.2f %10.2f %10.2f %6s      \r\n",Rob.M[0].angle,Rob.M[1].angle,Rob.M[2].angle,Mess2);
		HAL_UART_Transmit_DMA(&huart2, UART_OUT, N_OUT); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	}

}

/*
 * Funkcja opóźnienia do generacji sygnału PWM
 */
void delay1(uint16_t time){
	/* time = 1 equals 10 ns
	 * 1000000ns = 1ms*/
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3) < time);
}
/*
 * Funkcja opóźnienia do generacji sygnału PWM
 */
void delay2(uint16_t time){
	/* time = 1 equals 10 ns
	 * 1000000ns = 1ms*/
	__HAL_TIM_SET_COUNTER(&htim7,0);
	while(__HAL_TIM_GET_COUNTER(&htim7) < time);
}
/*
 * Funkcja zamienia wartość PWM na okres opóźnienia w nano sekundach
 */
uint16_t RPM_2_Delay(uint16_t RPM){
	return rot_time/RPM;
}
/*
 * Funkcja pozwala wybrać opóźnienie na innym timerze
 */
void Delay_Set(uint16_t del, int timer){
	switch(timer){
	case 1:
		delay1(del);
		break;
	case 2:
		delay2(del);
		break;
	}
}

/*
 * Funkcja realizuje obrót silnikiem o jeden skok w zależności od adresu obraca silnikiem M1 lub M2
 */
int Rot_Deg_Fast_Acc2(silnik *S, double angle,int step, int steps){

	if(S == &Rob.M[0]){

		if(step < steps){

			if(angle<0.00){
				HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_RESET);
				S->angle = S->Last_angle + (double)step*deg_per_step*(-1);
			}
			else{
				HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_SET);
				S->angle = S->Last_angle + (double)step*deg_per_step;
			}

			if(step <= S->t1){
				t_J1 = t_J1 + 0.01;
				S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t_J1/S->tau)));
				//S->RPM = RPM_2_Delay(S->RPM_Max)/pow((1+exp(-k*t_J1)),a);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,1);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			else if((step > S->t1) && (step < S->t2)){
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,1);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			else if(step >= S->t1){
				t_J1 = t_J1 - 0.01;
				S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t_J1/S->tau)));
				//S->RPM = RPM_2_Delay(S->RPM_Max)/pow((1+exp(-k*t_J1)),a);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,1);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			return 1;
		}
		else{
			t_J1=0;
			S->RPM = 0;
			Delay_Set(S->RPM,1);
			return 0;
		}

	}
	else if(S == &Rob.M[1]){

		if(step <= steps){	//kryterium stopu obrotu

			if(angle<0.00){
				HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_RESET);
				S->angle = S->Last_angle + (double)step*deg_per_step*(-1);
			}
			else{
				HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_SET);
				S->angle = S->Last_angle + (double)step*deg_per_step;
			}

			if(step <= S->t1){

				t_J2 = t_J2 + 0.01;

				S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t_J2/S->tau)));
				//S->RPM = RPM_2_Delay(S->RPM_Max)/pow((1+exp(-k*t_J1)),a);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,2);	//Wylicza czas pomiedzy stanem wysokim i niskim i ustawia wartosc RPM
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			else if((step > S->t1) && (step< S->t2)){ //Predkosc stała
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,2);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			else if(step >= S->t2){ //zwalnianie
				t_J2 = t_J2 - 0.01;
				S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t_J2/S->tau)));
				//S->RPM = RPM_2_Delay(S->RPM_Max)/pow((1+exp(-k*t_J1)),a);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
				Delay_Set(S->RPM,2);
				HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
			}
			return 1;
		}
		else{
			t_J2=0;
			S->RPM = 0;
			Delay_Set(S->RPM,2);
			return 0;
		}
	}
	else{
		return -1;
	}
}
/*
 * Funkcja Obraca na przemian 2 silnikami tworząc wrażenie, że poruszają się w tym samym czasie
 */
void Move_J1_J2(robot *R, double *ang1, double *ang2){

	//Wyliczenie ilosc kroków  na bazie podanego kąta
	steps1 = round((fabs(*ang1)/deg_per_step));	steps1 = steps1 * R->M[0].gear;
	steps2 = round((fabs(*ang2)/deg_per_step));	steps2 = steps2 * R->M[0].gear;
	int steps_max = steps1;

	//Ustawianie maksymalnej ilosci krokow do przebycia
	if(steps1<steps2){
		steps_max = steps2;
	}

	// Wyliczanie ti i t2 dla silników M1 i M2
	R->M[0].t1 = steps1*(R->M[0].t1_proc);
	R->M[0].t2 = steps1*(R->M[0].t2_proc);
	R->M[1].t1 = steps2*(R->M[1].t1_proc);
	R->M[1].t2 = steps2*(R->M[1].t2_proc);

	for( int step=0; step <= steps_max ;step++ ){

		Rot_Deg_Fast_Acc2(&R->M[0], *ang1, step , steps1);
		Rot_Deg_Fast_Acc2(&R->M[1], *ang2, step , steps2);
		delay2(1000);
		if(!J2_limit_flag){
			if( HAL_GPIO_ReadPin(GPIOA, LIMIT_J2_Pin) == GPIO_PIN_SET ){
				break;
			}
		}
	}
	R->M[0].Last_angle = R->M[0].angle;
	R->M[1].Last_angle = R->M[1].angle;

}
/*
 * Funkcja realizuje obrót silnikiem o dany kąt
 */
void Rot_Deg_Fast_Acc(silnik *S, double angle){

	int steps = ceil((fabs(angle)/deg_per_step));
	steps = (int)steps*S->gear;
	S->t1 = steps*(S->t1_proc);
	S->t2 = steps*(S->t2_proc);
	double t = 0;
	for(int step=0; step <= steps ;step++)
	{

		if(angle<0.00){
			HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_RESET);
			S->angle = S->Last_angle + (double)step*deg_per_step*(-1);
		}
		else{
			HAL_GPIO_WritePin(S->GPIO_DIR, S->Dir_Pin,GPIO_PIN_SET);
			S->angle = S->Last_angle + (double)step*deg_per_step;
		}

		if(!z_limit_flag){
			if( HAL_GPIO_ReadPin(GPIOA, LIMIT_Z_Pin) == GPIO_PIN_SET ){
				break;
			}
		}
		else if(!J2_limit_flag){
			if( HAL_GPIO_ReadPin(GPIOA, LIMIT_J2_Pin) == GPIO_PIN_SET ){
				break;
			}
		}
		if(step <=S->t1){
			t = t + 0.001;
			S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t/(S->tau))));
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
			Delay_Set(S->RPM,1);
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
		}

		else if(step >=S->t2){
			t = t - 0.001;
			S->RPM = (RPM_2_Delay(S->RPM_Max)*(1-exp(-t/(S->tau))));
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
			Delay_Set(S->RPM,1);
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
		}
		else if((step > S->t1) && (step < S->t2)){
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_SET);
			Delay_Set(S->RPM,1);
			HAL_GPIO_WritePin(S->GPIO_STEP,S->STEP_Pin,GPIO_PIN_RESET);
		}
	}
	S->Last_angle = S->angle;
	S->RPM = 0;
	Delay_Set(S->RPM,1);
	t=0;
}
/*
 * Obliczanie kinematyki prostej
 */
void kinematic( double deg_J1, double deg_J2,double *X, double *Y){

	double J1=(deg_J1*PI)/180.00,J2=(deg_J2*PI)/180.00;;

	*X = round( L1 * cos(J1) + L2 * cos(J1 + J2) );
	*Y = round( L1 * sin(J1) + L2 * sin(J1 + J2) );

}
/*
 * Obliczanie kinematyki odwrotnej
 * + zabezpieczenie przed przekroczeniem obszaru roboczego ramienia
 */
void inverse_kinematic(double X,double Y, double *J1, double *J2){
	double J1_rad = PI,J2_rad = PI;
	double R = sqrtl(X*X+Y*Y);
	if(2*PI*R < OBSZAR_MIN || 2*PI*R > OBSZAR_MAX){
		*J1 = 69.00;
		*J2 = 69.00;
	}
	else{
		J1_rad = 2*atan((400*Y - pow((- X*X*X*X - 2*X*X*Y*Y + 113800*X*X - Y*Y*Y*Y + 113800*Y*Y - 533610000),(0.5)))/(X*X + 400*X + Y*Y + 23100));
		J2_rad = -2*atan(pow((-(X*X + Y*Y - 4900)*(X*X + Y*Y - 108900)),(0.5))/(X*X + Y*Y - 4900));
		*J1 = (J1_rad*180.0)/ PI;
		*J2 = (J2_rad*180.0)/ PI;
	}
}
/*
 * Zamiana współrzędnych kartezjańskich Z na odlegość kątową, jaką musi przebyć silnik,
 *  aby osiągnąć odpowiednią wysokość
 */
void Z_move(robot *R,double Move_Z){
	double skok = 2.00;

	R->Move_JZ = (360.00/skok) * Move_Z;
	//	R->M[2].angle = R->M[2].angle + R->Move_JZ;
	Rot_Deg_Fast_Acc(&R->M[2],-R->Move_JZ);
}
/*
 * Zapisywanie współrzędnych i stanu chwytaka do tablicy sekwencji
 */
int save_sequence(robot *R,bool sequence){
	static int count = 0;

	if((count < N_seq && sequence) ){
		R->sekwencja.X[count] 			= R->X;
		R->sekwencja.Y[count] 			= R->Y;
		R->sekwencja.Z[count] 			= R->Z;
		R->sekwencja.grab[count] 		= R->grab;
		count++;
	}
	return count;
}
/*
 * Realizacjia sekwencji, przechodzenie po elementach tablicy sekwencji
 */
void sequence(robot *R){
	int move_nr = save_sequence(R,false);
	//tempp = move_nr;
	for(int i = 0; i<move_nr ; i++){
		move_xyz(R,R->sekwencja.X[i] ,R->sekwencja.Y[i] ,R->sekwencja.Z[i],R->sekwencja.grab[i], false );
		HAL_UART_RxCpltCallback(&huart2);
	}
	delay_ms(500);
}
/*
 * Czyszczenie tablicy
 */
void clear_sequence(){
	for (int i = 0; i < N_seq; i++) {
		Rob.sekwencja.X[i] = 330;
		Rob.sekwencja.Y[i] = 0;
		Rob.sekwencja.Z[i] = 70;
		Rob.sekwencja.grab[i] = OPEN;
	}

}
/*
 * Funkcja sterująca otwieraniem i zamykaniem chwytakiem
 */
void grabbing(robot *R, int grb){
	if(grb == OPEN){
		Mess2 = "Open";
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,OPEN);
	}
	else if(grb == CLOSE){
		Mess2 = "Close";
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,CLOSE-100);
		delay_ms(300);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,CLOSE);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,grb);
	}
}
/*
 * Przechodzenie do wskazanego punktu (x,y,z,chwytak)
 * Możliwość zapisywania punktu do którego się przeniosło
 * sequence = true 	-> zapisuj
 * sequence = false -> nie zapisuj
 */
void move_xyz(robot *R,double x,double y,double z,int grb, bool sequence){

	if(move_done){
		R->Last_X = R->X;       R->Last_Y = R->Y;   R->Last_Z = R->Z;	R->Last_grab = R->grab;
		R->X = x;               R->Y = y;           R->Z = z;	R->grab = grb;
		inverse_kinematic( R->Last_X, R->Last_Y,&R->Last_J1,&R->Last_J2);
		inverse_kinematic(x,y,&R->J1,&R->J2);

		if(R->J1 != R->Last_J1 && R->J2 != R->Last_J2){
			move_done = false;
			save_sequence(R,sequence);

			R->Move_X =  R->Last_X - R->X;
			R->Move_Y =  R->Last_Y - R->Y;

			R->Move_J1 = R->Last_J1 - R->J1;
			R->Move_J2 = R->Last_J2 - R->J2;

			Move_J1_J2(R, &R->Move_J1, &R->Move_J2);

		}
		else if(R->J1 != R->Last_J1){
			move_done = false;
			save_sequence(R,sequence);

			R->Move_J1 = R->Last_J1 - R->J1;

			Rot_Deg_Fast_Acc(&R->M[0] , R->Move_J1);
		}
		else if(R->J2 != R->Last_J2){
			move_done = false;
			save_sequence(R,sequence);

			R->Move_J2 = R->Last_J2 - R->J2;

			Rot_Deg_Fast_Acc(&R->M[1] , R->Move_J2);
		}
		if(R->Z != R->Last_Z){
			move_done = false;
			save_sequence(R,sequence);

			R->Move_Z =  R->Last_Z - R->Z;

			Z_move(R,R->Move_Z);
		}
		if(R->grab != R->Last_grab){
			move_done = false;
			save_sequence(R,sequence);
			grabbing(R,R->grab);
		}
		move_done = true;
		delay_ms(500);
	}
}

/*
 * Konfiguracja ramienia względem realnego położenia, dojazd do krańcówki, kalibracja osi Z
 */
void setup(robot *R, int RPM_Max_Z,int RPM_Max_J2){
	Z_move(R,-1500);
	z_limit_flag = true;
	delay_ms(100);
	Z_move(R,2);
	z_limit_flag = false;
	R->M[2].RPM_Max = 10;
	Z_move(R, -1500);
	z_limit_flag = true;
	R->M[2].RPM_Max = RPM_Max_Z;
	new_Z = 70;
	R->M[2].angle = 0;
	delay_ms(500);

}
/*
 * Przypisywanie ustawień do silnika
 */
void set_settings(silnik *M){
	M->RPM_Max 	= strtol((char*)UART_IN,&pEnd,0);
	M->tau 	 	= strtod(pEnd,&pEnd);
	M->t1_proc 	= strtod(pEnd,&pEnd);
	M->t2_proc 	= strtod(pEnd,NULL);
}
