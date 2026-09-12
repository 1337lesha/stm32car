#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include "main.h" // Нужен для типов uint8_t, uint16_t и функций HAL

// ==============================================================================
// МАКРОСЫ ПАКЕТА DELTA-2G
// ==============================================================================
#define FRAME_HEADER          0XAA  // определяем заголовок пакета
#define DATA_HEADER           0xAD  // определяем значение байта начала данных
#define SIZE_HI_BYTE          1     // определяем место в пакете старшего байта размера данных без checksum
#define SIZE_LO_BYTE          2     // определяем место в пакете младшего байта размера данных без checksum
#define DATA_START_BYTE       5     // определяем место в пакете байта начала полезных данных
#define DISTANCE_DATA_SYZE    7     // определяем место в пакете размера данных расстояния
#define START_ANGLE_HI_BYTE   11    // определяем место старшего байта стартового угла пакета
#define START_ANGLE_LO_BYTE   12    // определяем место младшего байта стартового угла пакета
#define FIRST_DIST_HI_BYTE    14    // определяем место старшего байта первой пробы расстояния в пакете
#define FIRST_DIST_LO_BYTE    15    // определяем место младшего байта первой пробы расстояния в пакете


#define offset_angle 15 // смещение угла лидара

// ==============================================================================
// ЭКСПОРТ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ
// ==============================================================================
extern uint16_t lidar_data[360]; // чистые данные в мм
extern uint8_t data_ready;       // флаг готовности данных
extern uint8_t lidar_on;         // статус мотора лидара
extern uint32_t now_toggleLidar; // таймер для защиты от дребезга кнопки

// ==============================================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==============================================================================
uint16_t checksum_cmp(uint8_t *arr, uint16_t raw_data_length);
void parsing(uint8_t *arr, uint16_t size);

void lidar_print_step_it(void);
void lidar_print_frame_it(void);

uint16_t sector_count_near(uint16_t d[360], uint16_t start, uint16_t end, uint16_t min_dist);
uint16_t sector_average(uint16_t d[360], uint16_t start, uint16_t end);

void toggleLidar(void);

#endif /* LIDAR_DRIVER_H */
