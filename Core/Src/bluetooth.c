#include "bluetooth.h"

// Подключаем модули, которыми будем управлять через Bluetooth
#include "motor_control.h"
#include "lidar_driver.h"

// Если ты уже вынес автопилот в отдельный файл, раскомментируй строку ниже:
// #include "autopilot.h"
// А пока просто объявим функцию здесь, чтобы компилятор не ругался:
extern void autopilot(void);

// ==============================================================================
// ИНИЦИАЛИЗАЦИЯ ПЕРЕМЕННЫХ МОДУЛЯ СВЯЗИ
// ==============================================================================
uint8_t str = 0;                // Последняя полученная команда
uint32_t now_getBTcommand = 0;  // Таймер активности
uint8_t uart_tx_busy = 0;       // Флаг для защиты от наложения отправок по UART2

// ==============================================================================
// ФУНКЦИИ СВЯЗИ И УПРАВЛЕНИЯ
// ==============================================================================

/**
 * @brief  Watchdog таймер для Bluetooth. Останавливает робота при потере связи.
 * @note   Если команды с пульта не приходили более 300 мс, переводит робота в режим 'S' (coast/стоп).
 */
void BTcommandIsActive(void) {
    if(HAL_GetTick() - now_getBTcommand < 300) return;
    str = 'S'; // Сбрасываем команду на "Стоп"
    coast();   // Плавно останавливаем моторы
}

/**
 * @brief  Исполняет текущую команду, сохраненную в переменной str.
 * @note   Вызывается в главном цикле (while 1).
 */
void execute_bt_command(void) {
    switch (str) {
        case 'T':
            autopilot();
            break;
        case 'F':
            forward();
            break;
        case 'B':
            reverse();
            break;
        case 'S':
            coast();
            break;
        case 'L':
            left();
            break;
        case 'R':
            right();
            break;
        case 'G':
            forwardLeft();
            break;
        case 'I':
            forwardRight();
            break;
        case 'H':
            reverseLeft();
            break;
        case 'J':
            reverseRight();
            break;

        // Управление скоростью
        case '0': speedL = 100; speedR = 100; break;
        case '1': speedL = 140; speedR = 140; break;
        case '2': speedL = 153; speedR = 153; break;
        case '3': speedL = 165; speedR = 165; break;
        case '4': speedL = 178; speedR = 178; break;
        case '5': speedL = 191; speedR = 191; break;
        case '6': speedL = 204; speedR = 204; break;
        case '7': speedL = 216; speedR = 216; break;
        case '8': speedL = 229; speedR = 229; break;
        case '9': speedL = 242; speedR = 242; break;
        case 'q': speedL = 250; speedR = 250; break;

        // Включение/выключение лидара
        case 'E':
            toggleLidar();
            break;
    }
}
