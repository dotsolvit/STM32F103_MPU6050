#include "mpu6050.h"

// Регістри MPU6050
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B

HAL_StatusTypeDef MPU6050_Init_Async(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim) {
    dev->hi2c = hi2c;
    dev->htim = htim;
    dev->state = MPU_STATE_INIT_PWR;
    dev->i2c_busy = 0;
    dev->last_timestamp = 0;

    // Запускаємо таймер у базовому режимі лічильника
    HAL_TIM_Base_Start(dev->htim);
    return HAL_OK;
}

void MPU6050_Process_State(MPU6050_HandleTypeDef *dev) {
    if (dev->i2c_busy) return;

    switch (dev->state) {
        case MPU_STATE_INIT_PWR:
            dev->tx_buffer = 0x01; // Клок від Gyro X, вимкнути Sleep
            dev->i2c_busy = 1;
            if (HAL_I2C_Mem_Write_IT(dev->hi2c, MPU6050_I2C_ADDR, REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &dev->tx_buffer, 1) != HAL_OK) {
                dev->state = MPU_STATE_ERROR;
                dev->i2c_busy = 0;
            }
            break;

        case MPU_STATE_INIT_CONFIG:
            dev->tx_buffer = 0x03; // DLPF = 42Hz
            dev->i2c_busy = 1;
            HAL_I2C_Mem_Write_IT(dev->hi2c, MPU6050_I2C_ADDR, REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &dev->tx_buffer, 1);
            break;

        case MPU_STATE_INIT_GYRO:
            dev->tx_buffer = 0x08; // ±500 deg/s
            dev->i2c_busy = 1;
            HAL_I2C_Mem_Write_IT(dev->hi2c, MPU6050_I2C_ADDR, REG_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &dev->tx_buffer, 1);
            break;

        case MPU_STATE_INIT_ACCEL:
            dev->tx_buffer = 0x10; // ±8g
            dev->i2c_busy = 1;
            HAL_I2C_Mem_Write_IT(dev->hi2c, MPU6050_I2C_ADDR, REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &dev->tx_buffer, 1);
            break;

        case MPU_STATE_READY:
        	// Стан очікування команди на зчитування даних (наприклад, від таймера)
            break;

        case MPU_STATE_READING_DATA:
            dev->i2c_busy = 1;
            // ВИДАЛИТИ АБО ЗАКОМЕНТУВАТИ ЦЕЙ РЯДОК ТУТ:
            // Фіксуємо поточний час лічильника прямо перед запитом даних
            //dev->last_timestamp = __HAL_TIM_GET_COUNTER(dev->htim);

            // Важливо: для роботи DMA I2C у CubeIDE має бути увімкнений відповідний DMA канал
            if (HAL_I2C_Mem_Read_DMA(dev->hi2c, MPU6050_I2C_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dev->rx_buffer, 14) != HAL_OK) {
                dev->state = MPU_STATE_ERROR;
                dev->i2c_busy = 0;
            }
            break;

        case MPU_STATE_ERROR:
        	// Обробка помилки шини (можна спробувати скинути I2C)
            break;

        default:
            break;
    }
}

void MPU6050_I2C_MemTxCpltCallback(MPU6050_HandleTypeDef *dev) {
    dev->i2c_busy = 0;
    if (dev->state == MPU_STATE_INIT_PWR)         dev->state = MPU_STATE_INIT_CONFIG;
    else if (dev->state == MPU_STATE_INIT_CONFIG) dev->state = MPU_STATE_INIT_GYRO;
    else if (dev->state == MPU_STATE_INIT_GYRO)   dev->state = MPU_STATE_INIT_ACCEL;
    else if (dev->state == MPU_STATE_INIT_ACCEL)  dev->state = MPU_STATE_READY;
}

void MPU6050_I2C_MemRxCpltCallback(MPU6050_HandleTypeDef *dev) {
    dev->i2c_busy = 0;
    if (dev->state == MPU_STATE_READING_DATA) {
    	// Коректний зсув байтів (Big-Endian) для 14-байтного пакету
        dev->raw_data.accel_x = (int16_t)((dev->rx_buffer[0]  << 8) | dev->rx_buffer[1]);
        dev->raw_data.accel_y = (int16_t)((dev->rx_buffer[2]  << 8) | dev->rx_buffer[3]);
        dev->raw_data.accel_z = (int16_t)((dev->rx_buffer[4]  << 8) | dev->rx_buffer[5]);
        dev->raw_data.temp    = (int16_t)((dev->rx_buffer[6]  << 8) | dev->rx_buffer[7]);
        dev->raw_data.gyro_x  = (int16_t)((dev->rx_buffer[8]  << 8) | dev->rx_buffer[9]);
        dev->raw_data.gyro_y  = (int16_t)((dev->rx_buffer[10] << 8) | dev->rx_buffer[11]);
        dev->raw_data.gyro_z  = (int16_t)((dev->rx_buffer[12] << 8) | dev->rx_buffer[13]);

        dev->state = MPU_STATE_READY;
    }
}

void MPU6050_Calibrate_Gyro_Z(MPU6050_HandleTypeDef *dev, uint16_t samples) {
    int32_t sum = 0;
    uint16_t i = 0;

    while (i < samples) {
        if (dev->state == MPU_STATE_READY) {
            dev->state = MPU_STATE_READING_DATA;

            // Оскільки це ініціалізація, тут ми можемо зачекати завершення DMA транзакції
            while(dev->state != MPU_STATE_READY && dev->state != MPU_STATE_ERROR) {
                MPU6050_Process_State(dev);
            }
            sum += dev->raw_data.gyro_z;
            i++;
            HAL_Delay(5); // Інтервал між вимірами для кращої вибірки
        } else {
            MPU6050_Process_State(dev);
        }
    }
    dev->gyro_z_offset = sum / samples;
    dev->angle_z = 0.0f; // Скидаємо початковий кут в нуль
}

//Фінальна версія
void MPU6050_Update_Angle(MPU6050_HandleTypeDef *dev) {
    uint32_t current_time = __HAL_TIM_GET_COUNTER(dev->htim);
    uint32_t dt_ticks;

    // Якщо це самий перший крок, просто прирівнюємо мітки пам'яті, щоб dt не був велетенським
    if (dev->last_timestamp == 0) {
        dev->last_timestamp = current_time;
    }

    // Враховуємо апаратне переповнення 16-бітного лічильника таймера (ARR = 65535)
    if (current_time >= dev->last_timestamp) {
        dt_ticks = current_time - dev->last_timestamp;
    } else {
        dt_ticks = (65535 - dev->last_timestamp) + current_time + 1;
    }

    // Переводимо мікросекунди в секунди для отримання точного dt
    float dt = (float)dt_ticks / 1000000.0f;

    // Переводимо сирі дані в градуси за секунду та множимо на -1.0f, щоб поворот за годинниковою стрілкою став додатнім (+)
    dev->gyro_z_dps = -1.0f * ((float)(dev->raw_data.gyro_z - dev->gyro_z_offset) / 65.5f);

    // Інтегруємо швидкість для отримання кута
    dev->angle_z += dev->gyro_z_dps * dt;

    // Нормалізація кута у робототехнічний формат від 0.0 до 360.0 градусів
    // Використовуємо while замість if на випадок, якщо dt буде великим (захист архітектури)
    while (dev->angle_z >= 360.0f) {
        dev->angle_z -= 360.0f;
    }
    while (dev->angle_z < 0.0f) {
        dev->angle_z += 360.0f;
    }

    // Запам'ятовуємо поточний час для наступного кроку
    dev->last_timestamp = current_time;
}

