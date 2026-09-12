#include "motor_control.h"

// Внешние хэндлы таймеров из main.c
extern TIM_HandleTypeDef htim1;

// Инициализация глобальных переменных
uint8_t speedL = 40; // скорости левой и правой стороны моторов
uint8_t speedR = 40;
uint32_t timerSoftStart = 0;

/**
 * @brief  Реализует плавное нарастание скорости моторов (Soft Start).
 * @note   Прибавляет по +1 к ШИМ каждые 70 мс. Вызывается внутри forward().
 */
void softstart(void) {
    if(HAL_GetTick() - timerSoftStart > 70) {
        ++speedL;
        ++speedR;
        timerSoftStart = HAL_GetTick();
    }
}

void forward(void) {
    if(speedL < MAX_SPEED && speedR < MAX_SPEED){
        softstart();
    } else if(speedL > MAX_SPEED || speedR > MAX_SPEED){
        speedL = MAX_SPEED;
        speedR = MAX_SPEED;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void left(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);   // IN1=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // IN2=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
void right(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   // IN1=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // IN2=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
void forwardRight(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR/(uint8_t)4);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   // IN1=1
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // IN2=0
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
void forwardLeft(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL/(uint8_t)4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   // IN1=1
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // IN2=0
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void reverse(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // IN1=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   // IN2=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}
void reverseLeft(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL/(uint8_t)4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // IN1=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   // IN2=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}
void reverseRight(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speedL);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedR/(uint8_t)4);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // IN1=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   // IN2=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void brake(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   // IN1=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   // IN2=1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void coast(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // IN1=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // IN2=0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
