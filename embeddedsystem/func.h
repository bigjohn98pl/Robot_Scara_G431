/*
 * func.h
 *
 *  Created on: 10 lut 2022
 *      Author: Janusz
 */

#include "main.h"

#ifndef INC_FUNC_H_
#define INC_FUNC_H_

#define N_seq 20
#define PI 3.14159265359
#define L1 200.00
#define L2 130.00
#define OBSZAR_MIN 879.646
#define OBSZAR_MAX 2073.46
#define GEAR 1.00
#define N_IN 33 // ramka "  -999.99 -999.99 -999.99 9999 \n\r"
#define N_OUT 50
#define N_MESS 80
#define false 0
#define true 1
#define OPEN 2100
#define CLOSE 1500

const uint16_t steps_per_obort;
double deg_per_step;
uint16_t rot_time;

double new_X,	new_Y,	new_Z;
uint16_t new_grab;
double t_J1,t_J2;
int steps1,	steps2;

u_int8_t sekfencja_save;
u_int8_t sekfencja_start;
u_int8_t z_limit_flag;
u_int8_t J2_limit_flag;
u_int8_t move_done;

static uint8_t UART_OUT[N_OUT];
static uint8_t UART_IN[N_IN];
static char *Mess2;
uint16_t size;
char *pEnd;

typedef struct path{
	double X[N_seq],Y[N_seq],Z[N_seq];
	int grab[N_seq];
}path;

typedef struct silnik{
	GPIO_TypeDef* GPIO_DIR;
	GPIO_TypeDef* GPIO_STEP;
	uint16_t STEP_Pin;
	uint16_t Dir_Pin;
	double Last_angle;
	double angle;
	double t1_proc,t2_proc;
	double t1,t2;
	double tau;
	int gear;
	int RPM;
	int RPM_Max;
}silnik;

typedef struct robot{
	double X,Y,Z;
	double Last_X,Last_Y,Last_Z;
	double Move_X,Move_Y,Move_Z;
	double J1,J2;
	double Last_J1,Last_J2;
	double Move_J1,Move_J2,Move_JZ;
	int grab,Last_grab;
	silnik M[3];
	path sekwencja;
}robot;

/*
 * Główna zmienna
 */
robot Rob;

void Rot_Deg_Fast_Acc(silnik *S, double angle);
int Rot_Deg_Fast_Acc2(silnik *S, double angle,int step, int steps);
void Move_J1_J2(robot *R, double *ang1, double *ang2);
void Delay_Set(uint16_t del, int timer);
void kinematic(double J1,double J2,double *X,double *Y);
void inverse_kinematic(double X,double Y, double *J1, double *J2);
void Z_move(robot *R,double Z_UP_DOWN);
void move_xyz(robot *R,double x,double y,double z,int grb, bool sequence);
int save_sequence(robot *R,bool sequence);
void sequence(robot *R);
void clear_sequence();
void grabbing(robot *R, int grb);
void setup(robot *R, int RPM_Max_Z,int RPM_Max_J2);
void set_settings(silnik *M);
void delay1(uint16_t time);
void delay2(uint16_t time);

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

#endif /* INC_FUNC_H_ */
