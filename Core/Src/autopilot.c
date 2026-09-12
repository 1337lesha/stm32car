#include "autopilot.h"
#include "motor_control.h" // Для speedL, speedR, forward(), left(), right()
#include "lidar_driver.h"  // Для массива lidar_data и функции sector_count_near()

// ==============================================================================
// ОСНОВНАЯ ЛОГИКА АВТОПИЛОТА
// ==============================================================================

/**
 * @brief  Автономное управление роботом с использованием данных лидара.
 * @note   Реализует превентивное торможение и конечный автомат (таймер) для надежного поворота.
 */
void autopilot(void) {
    // 1. Проверяем дальнюю зону
    uint16_t front1 = sector_count_near(lidar_data, 0, 40, DIST_WARN) +
                      sector_count_near(lidar_data, 320, 359, DIST_WARN);

    // Переменная для удержания поворота (минимальное время маневра)
    static uint32_t turn_timer = 0;

    // Едем быстро, только если впереди чисто (дальше 500мм) и мы не находимся в процессе маневра
    if(front1 < 2 && HAL_GetTick() > turn_timer){
        forward();
        return;
    }
    else if (HAL_GetTick() > turn_timer) {
        // Ограничиваем скорость перед препятствием (от 300 до 500 мм)
        speedL = 55;
        speedR = 55;
    }

    // 2. Проверяем ближнюю (критическую) зону
    uint16_t front = sector_count_near(lidar_data, 0, 40, DIST_MIN) +
                     sector_count_near(lidar_data, 320, 359, DIST_MIN);
    uint16_t left_ = sector_count_near(lidar_data, 260, 319, DIST_MIN);
    uint16_t right_ = sector_count_near(lidar_data, 41, 100, DIST_MIN);

    // Впереди (300мм) чисто, едем вперед (на сниженной скорости 55, пока не выйдем за 500мм)
    if(front < 2 && HAL_GetTick() > turn_timer){
        forward();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Гасим светодиод (нет препятствия)
    }
    else {
        // 3. Выполняем маневр уклонения
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Зажигаем светодиод (препятствие)

        // Запоминаем, что начали крутиться, чтобы не оборвать поворот
        // 10 мс - это очень мало, рекомендую поднять хотя бы до 100-150 мс для уверенного маневра!
        if (HAL_GetTick() > turn_timer) {
            turn_timer = HAL_GetTick() + 150;
        }

        if(left_ > right_){
            speedL = 120;
            right();
        }
        else if(left_ < right_){
            speedR = 120;
            left();
        }
        else {
            // Если стена плоская (left_ == right_) или заехали в угол
            // Принудительно уходим вправо, чтобы не застрять
            speedL = 120;
            right();
        }
    }
}

/*void autopilot(){
	uint16_t front1 = sector_count_near(lidar_data, 0, 40, 500) + sector_count_near(lidar_data, 320, 359, 500);
	if(front1 < 2){
		forward();
		return;
	 }
	else {
		speedL = 55;
		speedR = 55;
	}

	uint16_t front = sector_count_near(lidar_data, 0, 40, DIST_MIN) + sector_count_near(lidar_data, 320, 359, DIST_MIN);
	uint16_t left_ = sector_count_near(lidar_data, 260, 319, DIST_MIN);
	uint16_t right_ = sector_count_near(lidar_data, 41, 100, DIST_MIN);
	if(front < 2){
		forward();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else if(left_ > right_){
		speedL = 120;
		right();
	}
	else if(left_ < right_){
		speedR = 120;
		left();
	}
	if(front > 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}

}*/



