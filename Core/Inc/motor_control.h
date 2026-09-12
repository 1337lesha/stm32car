#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h" // Обязательно подключаем, чтобы работали типы uint8_t и функции HAL

// Макросы
#define MAX_SPEED 80

// Экспортируем переменные, чтобы другие файлы могли их читать/менять
extern uint8_t speedL;
extern uint8_t speedR;

// Объявления функций (прототипы)
void softstart(void);
void forward(void);
void left(void);
void right(void);
void forwardRight(void);
void forwardLeft(void);
void reverse(void);
void reverseLeft(void);
void reverseRight(void);
void brake(void);
void coast(void);

#endif /* MOTOR_CONTROL_H */
