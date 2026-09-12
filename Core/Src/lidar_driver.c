#include "lidar_driver.h"
#include <stdio.h>  // Нужен для функции snprintf

// ==============================================================================
// ИНИЦИАЛИЗАЦИЯ ПЕРЕМЕННЫХ МОДУЛЯ ЛИДАРА
// ==============================================================================
uint16_t lidar_data[360] = {0}; // массив на 360 градусов (дистанция в мм)
uint8_t data_ready = 0;         // флаг: 1 - данные обновлены, 0 - ждем
uint8_t lidar_on = 0;           // 0 - выключен, 1 - включен
uint32_t now_toggleLidar = 0;   // таймер

// Локальные буферы (убраны из main.c для чистоты)
char uart_frame_buf[2200];      // Буфер для целикового вывода (Python)
char uart_tx_buf[64];           // Буфер для пошагового вывода (отладка)

// ==============================================================================
// ВНЕШНИЕ ЗАВИСИМОСТИ ИЗ main.c
// ==============================================================================
extern UART_HandleTypeDef huart2; // Хэндл UART для отправки на Bluetooth/ПК
extern uint8_t uart_tx_busy;      // Флаг занятости UART шины
extern TIM_HandleTypeDef htim2;   // Хэндл таймера для управления ШИМ мотора лидара

// ==============================================================================
// ФУНКЦИИ ПАРСИНГА И МАТЕМАТИКИ
// ==============================================================================

/**
 * @brief  Вычисляет контрольную сумму пакета данных от лидара.
 * @param  arr: Указатель на массив сырых данных пакета.
 * @param  raw_data_length: Длина полезных данных в пакете.
 * @retval Вычисленная контрольная сумма (uint16_t).
 */
uint16_t checksum_cmp(uint8_t *arr, uint16_t raw_data_length) {
    uint16_t _checksum = 0;
    while (raw_data_length--) {
        // складываем все байты пакета
        _checksum += *arr++;
    }
    return _checksum;
}

/**
 * @brief  Парсит сырые байты пакета лидара Delta-2G и заполняет массив lidar_data[360].
 * @note   Функция проверяет заголовки, контрольную сумму и высчитывает угол для каждого измерения.
 * @param  arr: Указатель на массив принятых сырых данных (через DMA).
 * @param  size: Размер принятых данных.
 * @retval None
 */
void parsing(uint8_t *arr, uint16_t size)
{
        data_ready = 0;
        // Проверка первого байта заголовка
        if (arr[0] != FRAME_HEADER)
            return;

        // Вычисление заявленной длины пакета
        uint16_t raw_data_length = ((uint16_t)arr[SIZE_HI_BYTE] << 8) | arr[SIZE_LO_BYTE];

        // Проверка, что мы приняли пакет целиком (защита от выхода за пределы массива)
        if (raw_data_length + 1 >= size)
            return;

        // Извлечение контрольной суммы из пакета
        uint16_t checksum = ((uint16_t)arr[raw_data_length] << 8) | arr[raw_data_length + 1];

        // Сверка контрольной суммы
        if (checksum != checksum_cmp(&arr[0], raw_data_length))
            return;

        // Проверка байта начала данных
        if (arr[5] != 0xAD)
            return;

        // Вычисление стартового угла пакета
        float start_angle = (((uint16_t)arr[START_ANGLE_HI_BYTE] << 8) |
                              arr[START_ANGLE_LO_BYTE]) * 0.01f;

        uint16_t payload_len = (((uint16_t)arr[6] << 8) | arr[7]);
        uint8_t read_count = (payload_len - 5) / 3; // Количество измерений в пакете

        float sector = 24.0f; // Один пакет обычно покрывает сектор в 24 градуса
        float step = sector / read_count; // Шаг угла между соседними измерениями

        // Распределение измерений по массиву 360 градусов
        for (uint8_t n = 0; n < read_count; n++)
        {
            float angle = start_angle + step * n + offset_angle;

            // Нормализация угла в пределах 0-359 градусов
            while (angle >= 360.0f) angle -= 360.0f;
            while (angle < 0.0f) angle += 360.0f;

            int i = (int)(angle + 0.5f); // Округление до целого градуса
            if (i >= 360) i = 0;

            // Извлечение сырого расстояния (в четвертях миллиметра)
            uint16_t dist_raw = ((uint16_t)arr[FIRST_DIST_HI_BYTE + n * 3] << 8) |
                                 arr[FIRST_DIST_LO_BYTE + n * 3];

            // Конвертация в миллиметры и сохранение в глобальный массив
            lidar_data[i] = (uint16_t)(dist_raw * 0.25f + 0.5f);
        }
        data_ready = 1; // Устанавливаем флаг успешного обновления данных
}

/**
 * @brief  Отправляет данные лидара по UART по одной точке (устаревшая/отладочная функция).
 * @note   Использует неблокирующую отправку. Отправляет "angle=X dist=Y\r\n".
 */
void lidar_print_step_it(void)
{
    static uint16_t idx = 0;
    static uint8_t printing = 0;

    if (!printing)
    {
        if (!data_ready)
            return;

        data_ready = 0;
        idx = 0;
        printing = 1;
    }

    if (uart_tx_busy)
        return;

    while (idx < 360)
    {
        uint16_t dist = lidar_data[idx];
        uint16_t angle = idx;
        idx++;

        if (dist == 0)
            continue;

        int len = snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                           "angle=%u dist=%u\r\n", angle, dist);

        if (len > 0)
        {
            uart_tx_busy = 1;
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_tx_buf, len);
            return;
        }
    }

    printing = 0;
}

/**
 * @brief  Отправляет полный массив (360 значений) данных лидара по UART одной строкой.
 * @note   Формат: "d0,d1,d2,...,d359\r\n". Используется для визуализации в Python.
 */
void lidar_print_frame_it(void)
{
    // Если данные не обновлены или UART занят предыдущей отправкой - выходим
    if (!data_ready || uart_tx_busy)
        return;

    int pos = 0;

    // Сборка массива в единую текстовую строку формата CSV
    for (uint16_t i = 0; i < 360; i++)
    {
        if (i < 359)
            pos += snprintf(&uart_frame_buf[pos], sizeof(uart_frame_buf) - pos, "%u,", lidar_data[i]);
        else
            pos += snprintf(&uart_frame_buf[pos], sizeof(uart_frame_buf) - pos, "%u\r\n", lidar_data[i]);

        // Защита от переполнения буфера
        if (pos >= (int)(sizeof(uart_frame_buf) - 16))
            break;
    }

    uart_tx_busy = 1; // Блокируем новые отправки до завершения текущей
    data_ready = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_frame_buf, pos); // Отправка по прерыванию
}

/**
 * @brief  Подсчитывает количество точек препятствия в заданном секторе.
 * @param  d: Массив данных лидара (360 градусов).
 * @param  start: Начальный угол сектора.
 * @param  end: Конечный угол сектора.
 * @param  min_dist: Дистанция порога (считаем точки ближе этого значения).
 * @retval Количество точек (uint16_t), удовлетворяющих условию. Игнорирует нули.
 */
uint16_t sector_count_near(uint16_t d[360], uint16_t start, uint16_t end, uint16_t min_dist)
{
    uint16_t count = 0;

    for (uint16_t i = start; i <= end; i++)
    {
        if (d[i] != 0 && d[i] < min_dist)
        {
            count++;
        }
    }

    return count;
}

/**
 * @brief  Вычисляет среднее расстояние до препятствий в заданном секторе.
 * @param  d: Массив данных лидара.
 * @param  start: Начальный угол сектора.
 * @param  end: Конечный угол сектора.
 * @retval Средняя дистанция в мм. Если валидных точек нет, возвращает 0.
 */
uint16_t sector_average(uint16_t d[360], uint16_t start, uint16_t end)
{
    uint32_t sum = 0;
    uint16_t count = 0;

    for (uint16_t i = start; i <= end; i++)
    {
        if (d[i] != 0)
        {
            sum += d[i];
            count++;
        }
    }

    if (count == 0)
        return 0;

    return (uint16_t)(sum / count);
}

/**
 * @brief  Включает или выключает вращение мотора лидара (генерацию ШИМ).
 * @note   Имеет защиту от команд: меняет состояние не чаще чем раз в 1000 мс.
 */
void toggleLidar(){
      if( HAL_GetTick()-now_toggleLidar<1000) return;

      if(lidar_on==0){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,100);
        lidar_on = 1;
      }
      else if(lidar_on){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        lidar_on = 0;
      }
      now_toggleLidar = HAL_GetTick();
}
