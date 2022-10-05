//
// Created by vitaliy on 29.03.2022.
//

#ifndef C60APP_PREFERENCE_H
#define C60APP_PREFERENCE_H

#include "../Inc/main.h"
#include "../../Drivers/CMSIS/Include/cmsis_compiler.h"
#include "../RosLibs//std_msgs/Float32.h"


extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;


/** Переключение на задание скорости двигателя, вместо задания мощности */
#define RADIAN_SECOND

/** Контроль скорости вращения энкодера */
/** Коэффициент, на который умножается квадратный корень дельты уставки и показаний */
#define DELTA_SQRT_MULTIPLIER  1

/** ====================  Настройки логгера  =================== */
/** размер буфера лога для отправки в UART_TX */
#define LOG_TX_SIZE  1024  //
/** размер буфера лога создаваемого в прерывании */
#define LOG_IT_SIZE  512

/** Интервал в миллисекундах между отправками ЛОГА в UART через DMA */
#define LOG_UART_TX_DMA_TIMEOUT 10

/** uart привязанный к логгеру */
#define UART_LOGGER huart2
/** DMA привязанный к UART TX логгера */
#define DMA_TX_LOGGER hdma_usart2_tx
/** DMA привязанный к UART RX логгера */
#define DMA_RX_LOGGER hdma_usart2_rx


/** ========= Максимальное количество заданных интервалов ========= */
#define INTERVALS_QUA 10

/** ========= Тип сообщения ROS для управления мощностью двигателей ===== */
#define PWR_MOTOR_MSG std_msgs::Float32


/** ===== Управление мощностью двигателей ====== */

/** Значение PWM для минимальной мощности двигателя */
#define MINIMUM_LIMIT_PWM  100 // 688 для BLDC

/** Значение PWM для Максимальной мощности двигателя */
#define MAXIMUM_LIMIT_PWM  999

/** количество двигателей */
#define MOTORS_QUA 4
/** Значение в регистре ШИМ при максимальной мощности двигателя */
#define MAX_PWM_PULSE_COUNT 1000
/** Максимально возможное ускорение(увеличение мощности) двигателя <единиц мощности в миллисекунду> */
#define MAX_ACCELERATION 10
/** Длительность в миллисекундах одного шага при изменении мощности */
#define STEP_POWER_CONTROL 5
/** Шаг изменения мощности для выбранных настроек */
#define INCREMENT_PWR_VALUE  150

/** =========== Энкодеры двигателей ============ */
/** Интервал между считываниями состояния энкодеров в миллисекундах */
#define ENCODER_CONTROL_STEP 20

const float  PI_F=3.14159265358979f; // PI в формате FLOAT
/** Коэффициент пропорциональности количество радиан на один щелчёк энкодера */
#define SPEED_RADIAN_RATIO (PI_F*8/1800)
/** Максимальное значение скорости в радианах в секунду для DC двигателя */
#define MAX_RADIAN_SPEED  PI_F*8

/** PWM таймеры, каналы, энкодеры двигателей */
/** motor 0 */
#define ENCODER_TIM_M0 htim1
#define PWM_TIMER_M0 htim11
#define PWM_TIMER_M0_CHANNEL TIM_CHANNEL_1
#define PWM_REG_M0 CCR1
#define M0_DC_PWM PWM_TIMER_M0.Instance->PWM_REG_M0

/** motor1 */
#define ENCODER_TIM_M1 htim2
#define PWM_TIMER_M1 htim10
#define PWM_TIMER_M1_CHANNEL TIM_CHANNEL_1
#define PWM_REG_M1 CCR1
#define M1_DC_PWM PWM_TIMER_M1.Instance->PWM_REG_M1

/** motor2 */
#define ENCODER_TIM_M2 htim3
#define PWM_TIMER_M2 htim5
#define PWM_TIMER_M2_CHANNEL TIM_CHANNEL_1
#define PWM_REG_M2 CCR1
#define M2_DC_PWM PWM_TIMER_M2.Instance->PWM_REG_M2

/** motor3 */
#define ENCODER_TIM_M3 htim4
#define PWM_TIMER_M3 htim5
#define PWM_TIMER_M3_CHANNEL TIM_CHANNEL_2
#define PWM_REG_M3 CCR2
#define M3_DC_PWM PWM_TIMER_M3.Instance->PWM_REG_M3





/** Пины двигателей */
/** PWM - таймеры и каналы */
//#define M0_BLDC_PWM  htim3.Instance->CCR1
//#define M1_BLDC_PWM  htim3.Instance->CCR2
//#define M2_BLDC_PWM  htim3.Instance->CCR3
//#define M3_BLDC_PWM  htim3.Instance->CCR4
/** Датчики Холла */
//#define M0_HALL_SENSOR  htim4.Instance->CCR1

/** Тип сообщения о скорости двигателей */
#define SPEED_MSG std_msgs::Float32

#endif //C60APP_PREFERENCE_H
