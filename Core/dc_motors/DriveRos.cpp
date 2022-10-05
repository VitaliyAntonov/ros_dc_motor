//
// Created by vitaliy on 22.04.2022.
//

#include "DriveRos.h"
#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"
#include "../logger/Logger.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim11;

DriveRos dMotor; // Экземпляр класса управления двигателями

extern Logger mLog; // экземпляр логгера

//#define DEBUG_MOTORS_DIRECTION // отладка управления двигателями

/** Конструктор класса контроля всех двигателей устройства */
DriveRos::DriveRos() {

  /** Создаём объекты ячеек управления двигателями, фиксируем указатели  */

  /** мотор 0 */
  arrPwrm[NUM_MOTOR_0] = new PwrMotor0() ; // Экземпляр ячеек управления мощностью
  arrPwrm[NUM_MOTOR_0]->powerProcess = new MotorDifferenceProcess() ; // Экземпляр процесса для motor0

  /** мотор 1 */
  arrPwrm[NUM_MOTOR_1] = new PwrMotor1() ;
  arrPwrm[NUM_MOTOR_1]->powerProcess = new MotorDifferenceProcess() ;

  /** мотор 2 */
  arrPwrm[NUM_MOTOR_2] = new PwrMotor2() ;
  arrPwrm[NUM_MOTOR_2]->powerProcess = new MotorDifferenceProcess() ;

  /** мотор 3 */
  arrPwrm[NUM_MOTOR_3] = new PwrMotor3() ;
  arrPwrm[NUM_MOTOR_3]->powerProcess = new MotorDifferenceProcess() ;

  /** данные энкодеров двигателей */
  encoderDrives = new Encoder();

}


/** Инициализация двигателей */
void DriveRos::initMotor(){

  HAL_TIM_Base_Start(&ENCODER_TIM_M0); // Энкодер двигателя M0
  HAL_TIM_Base_Start(&ENCODER_TIM_M1); // Энкодер двигателя M1
  HAL_TIM_Base_Start(&ENCODER_TIM_M2); // Энкодер двигателя M2
  HAL_TIM_Base_Start(&ENCODER_TIM_M3); // Энкодер двигателя M3

  /** Запуск таймера двигателя M0 как PWM  */
  HAL_TIM_Base_Start(&PWM_TIMER_M0);
  HAL_TIM_PWM_Start(&PWM_TIMER_M0, PWM_TIMER_M0_CHANNEL);
  /** Запуск таймера двигателя M1 как PWM  */
  HAL_TIM_Base_Start(&PWM_TIMER_M1);
  HAL_TIM_PWM_Start(&PWM_TIMER_M1, PWM_TIMER_M1_CHANNEL);
  /** Запуск таймера двигателя M2 как PWM  */
  HAL_TIM_Base_Start(&PWM_TIMER_M2);
  HAL_TIM_PWM_Start(&PWM_TIMER_M2, PWM_TIMER_M2_CHANNEL);
  /** Запуск таймера двигателя M3 как PWM  */
  HAL_TIM_Base_Start(&PWM_TIMER_M3);
  HAL_TIM_PWM_Start(&PWM_TIMER_M3, PWM_TIMER_M3_CHANNEL);

  /** Motor0 Задаём направление вращения двигателя - СТОП */
  HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(BIN1_MOTOR_1_GPIO_Port, BIN1_MOTOR_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_1_GPIO_Port, BIN2_MOTOR_1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(AIN1_MOTOR_2_GPIO_Port, AIN1_MOTOR_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_2_GPIO_Port, AIN2_MOTOR_2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(BIN1_MOTOR_3_GPIO_Port, BIN1_MOTOR_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_3_GPIO_Port, BIN2_MOTOR_3_Pin, GPIO_PIN_RESET);

}


/** Приведение типа сообщения PWR_MOTOR_MSG к типу int */
int DriveRos::msgRosToInt(PWR_MOTOR_MSG msg){
  /** умножаем пришедший float на 1000 и преобразуем в int */
  int pwm = static_cast <int> (msg.data * MAX_PWM_PULSE_COUNT);
  if(pwm > MAX_PWM_PULSE_COUNT) pwm = MAX_PWM_PULSE_COUNT;
  if(pwm < -MAX_PWM_PULSE_COUNT) pwm = -MAX_PWM_PULSE_COUNT;
  return pwm;
}



/** Motor0 */
void PwrMotor0::motorStop() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 0 Stop\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_RESET);
  direction = MOTOR_STOP;
}
void PwrMotor0::motorForward() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 0 Forward\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_SET);
  direction = MOTOR_FORWARD;
}
void PwrMotor0::motorBack() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 0 Back\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_RESET);
  direction = MOTOR_BACK;
}
void PwrMotor0::setPwr(int pwr) {
  if(pwr == 0) motorStop();
  else if(pwr > 0) motorForward();
  else motorBack();
  uint16_t mRegLoadPwm = static_cast <uint16_t> (abs(pwr)); /** Значение для загрузки в регистр ШИМ */
  M0_DC_PWM = mRegLoadPwm; /** Загружаем регистр ШИМ */
  mPwrState = pwr;
}

/** Motor1 */
void PwrMotor1::motorStop() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 1 Stop\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_1_GPIO_Port, BIN1_MOTOR_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_1_GPIO_Port, BIN2_MOTOR_1_Pin, GPIO_PIN_RESET);
  direction = MOTOR_STOP;
}
void PwrMotor1::motorForward() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 1 Forward\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_1_GPIO_Port, BIN1_MOTOR_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_1_GPIO_Port, BIN2_MOTOR_1_Pin, GPIO_PIN_SET);
  direction = MOTOR_FORWARD;
}
void PwrMotor1::motorBack() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 1 Back\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_1_GPIO_Port, BIN1_MOTOR_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_MOTOR_1_GPIO_Port, BIN2_MOTOR_1_Pin, GPIO_PIN_RESET);
  direction = MOTOR_BACK;
}
void PwrMotor1::setPwr(int pwr) {
  if(pwr == 0) motorStop();
  else if(pwr > 0) motorForward();
  else motorBack();
  uint16_t mRegLoadPwm = static_cast <uint16_t> (abs(pwr)); /** Значение для загрузки в регистр ШИМ */
  M1_DC_PWM = mRegLoadPwm; /** Загружаем регистр ШИМ */
  mPwrState = pwr;
}

/** Motor2 */
void PwrMotor2::motorStop() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 2 Stop\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_2_GPIO_Port, AIN1_MOTOR_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_2_GPIO_Port, AIN2_MOTOR_2_Pin, GPIO_PIN_RESET);
  direction = MOTOR_STOP;
}
void PwrMotor2::motorForward() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 2 Forward\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_2_GPIO_Port, AIN1_MOTOR_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_MOTOR_2_GPIO_Port, AIN2_MOTOR_2_Pin, GPIO_PIN_SET);
  direction = MOTOR_FORWARD;
}
void PwrMotor2::motorBack() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 2 Back\r\n)
#endif
  HAL_GPIO_WritePin(AIN1_MOTOR_2_GPIO_Port, AIN1_MOTOR_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_MOTOR_2_GPIO_Port, AIN2_MOTOR_2_Pin, GPIO_PIN_RESET);
  direction = MOTOR_BACK;
}
void PwrMotor2::setPwr(int pwr) {
  if(pwr == 0) motorStop();
  else if(pwr > 0) motorForward();
  else motorBack();
  uint16_t mRegLoadPwm = static_cast <uint16_t> (abs(pwr)); /** Значение для загрузки в регистр ШИМ */
  M2_DC_PWM = mRegLoadPwm; /** Загружаем регистр ШИМ */
  mPwrState = pwr;
}

/** Motor3 */
void PwrMotor3::motorStop() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 3 Stop\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_3_GPIO_Port, BIN1_MOTOR_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_3_GPIO_Port, BIN2_MOTOR_3_Pin, GPIO_PIN_RESET);
  direction = MOTOR_STOP;
}
void PwrMotor3::motorForward() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 3 Forward\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_3_GPIO_Port, BIN1_MOTOR_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_MOTOR_3_GPIO_Port, BIN2_MOTOR_3_Pin, GPIO_PIN_SET);
  direction = MOTOR_FORWARD;
}
void PwrMotor3::motorBack() {
#ifdef DEBUG_MOTORS_DIRECTION
  log_string(Motor 3 Back\r\n)
#endif
  HAL_GPIO_WritePin(BIN1_MOTOR_3_GPIO_Port, BIN1_MOTOR_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_MOTOR_3_GPIO_Port, BIN2_MOTOR_3_Pin, GPIO_PIN_RESET);
  direction = MOTOR_BACK;
}
void PwrMotor3::setPwr(int pwr) {
  if(pwr == 0) motorStop();
  else if(pwr > 0) motorForward();
  else motorBack();
  uint16_t mRegLoadPwm = static_cast <uint16_t> (abs(pwr)); /** Значение для загрузки в регистр ШИМ */
  M3_DC_PWM = mRegLoadPwm; /** Загружаем регистр ШИМ */
  mPwrState = pwr;
}

