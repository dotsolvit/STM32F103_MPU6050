#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"

#define MPU6050_I2C_ADDR    (0x68 << 1)

// Перелік станів для неколокуючого опитування
typedef enum {
    MPU_STATE_UNINIT = 0,
    MPU_STATE_INIT_PWR,
    MPU_STATE_INIT_CONFIG,
    MPU_STATE_INIT_GYRO,
    MPU_STATE_INIT_ACCEL,
    MPU_STATE_READY,
    MPU_STATE_READING_DATA,
    MPU_STATE_ERROR
} MPU_State_t;

// Сирі дані датчика
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU_RawData_t;

// Основний дескриптор драйвера
typedef struct {
    I2C_HandleTypeDef *hi2c;
    TIM_HandleTypeDef *htim;       // Посилання на таймер TIM2
    MPU_State_t        state;
    MPU_RawData_t      raw_data;
    uint8_t            tx_buffer;
    uint8_t            rx_buffer[14]; // Чітко задаємо розмір масиву для DMA
    volatile uint8_t   i2c_busy;  // Прапорець завершення транзакції

    // Нові змінні для математичних розрахунків:
    int32_t            gyro_z_offset;  // Смещение нуля оси Z
    float              gyro_z_dps;     // Углова швидкість у градусах/сек (°/s)
    float              angle_z;        // Поточний розрахований кут у градусах

    uint32_t           last_timestamp; // Для збереження попереднього значення TIM2->CNT
} MPU6050_HandleTypeDef;

// API функції
HAL_StatusTypeDef MPU6050_Init_Async(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);
void MPU6050_Process_State(MPU6050_HandleTypeDef *dev);
void MPU6050_I2C_MemTxCpltCallback(MPU6050_HandleTypeDef *dev);
void MPU6050_I2C_MemRxCpltCallback(MPU6050_HandleTypeDef *dev);

// Прототипи нових функцій для роботи з кутом
void MPU6050_Calibrate_Gyro_Z(MPU6050_HandleTypeDef *dev, uint16_t samples);
void MPU6050_Update_Angle(MPU6050_HandleTypeDef *dev);

#endif /* MPU6050_H_ */
