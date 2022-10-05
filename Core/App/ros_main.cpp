#include "ros_main.h"
#include "../Inc/main.h"

#include "../RosLibs/ros.h"
#include "../RosLibs/std_msgs/String.h"
#include "../RosLibs/std_msgs/Float32.h"
//#include "../RosLibs/sensor_msgs/Imu.h"
#include "../RosLibs/std_msgs/Int32.h"

#include "../Process/Interval.h"

#include "../logger/Logger.hpp"

#include "../Inc/preference.h"
#include "../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

#include "../dc_motors/DriveRos.h"


void receiverCallback(const std_msgs::String & msg);
void recMotor0(const PWR_MOTOR_MSG & msg);
void recMotor1(const PWR_MOTOR_MSG & msg);
void recMotor2(const PWR_MOTOR_MSG & msg);
void recMotor3(const PWR_MOTOR_MSG & msg);

/** Объект управления двигателями */
extern DriveRos dMotor;

/** массив данных об интервалах */
extern Interval arrIP;


extern Logger mLog;


extern TIM_HandleTypeDef htim1;

/** узел ROS */
ros::NodeHandle nh;


/** Публикатор */
std_msgs::String str_msg;
ros::Publisher chatter("kek", &str_msg);
char hello[] = "Hello world!";
char logMsgBuffer[50];

// Подписчики
/** Из обобщённого шаблона подписчика формируем объект подписчика для типа String */
ros::Subscriber <std_msgs::String> rec("lol", receiverCallback, 1);

/** motor0/pwr подписчик - мощность мотора 0 */
ros::Subscriber <PWR_MOTOR_MSG> motor0_pwr("dc0/pwr", recMotor0, 1);

/** -------------- Энкодер двигателя  motor0. Публикатор ------------------- */
std_msgs::Int32 encoderM0_data; // щелчки на интервал
#ifdef RADIAN_SECOND // При уставке скорости в радианах в секунду
PWR_MOTOR_MSG encM0_radData;    // радианы в секунду
ros::Publisher encDataRadM0("encDataRadM0",&encM0_radData);
#else // уставка по мощности в единицах ШИМ 0...1000
ros::Publisher encoderM0("encoderM0", &encoderM0_data);
#endif
/** ------------------------------------------------------------------------ */

/** motor1/pwr подписчик - мощность мотора 1 */
ros::Subscriber <PWR_MOTOR_MSG> motor1_pwr("dc1/pwr", recMotor1, 1);

/** --------------- Энкодер двигателя  motor1. Публикатор ------------------ */
std_msgs::Int32 encoderM1_data;  // щелчки на интервал
#ifdef RADIAN_SECOND // При уставке скорости в радианах в секунду
PWR_MOTOR_MSG encM1_radData;    // радианы в секунду
ros::Publisher encDataRadM1("encDataRadM1",&encM1_radData);
#else // уставка по мощности в единицах ШИМ 0...1000
ros::Publisher encoderM1("encoderM1", &encoderM1_data);
#endif
/** ------------------------------------------------------------------------ */

/** motor2/pwr подписчик - мощность мотора 2 */
ros::Subscriber <PWR_MOTOR_MSG> motor2_pwr("dc2/pwr", recMotor2, 1);

/** --------------- Энкодер двигателя  motor2. Публикатор ------------------ */
std_msgs::Int32 encoderM2_data;  // щелчки на интервал
#ifdef RADIAN_SECOND // При уставке скорости в радианах в секунду
PWR_MOTOR_MSG encM2_radData;    // радианы в секунду
ros::Publisher encDataRadM2("encDataRadM2",&encM2_radData);
#else // уставка по мощности в единицах ШИМ 0...1000
ros::Publisher encoderM2("encoderM2", &encoderM2_data);
#endif
/** ------------------------------------------------------------------------ */

/** motor2/pwr подписчик - мощность мотора 3 */
ros::Subscriber <PWR_MOTOR_MSG> motor3_pwr("dc3/pwr", recMotor3, 1);

/** --------------- Энкодер двигателя  motor3. Публикатор ------------------ */
std_msgs::Int32 encoderM3_data;
#ifdef RADIAN_SECOND // При уставке скорости в радианах в секунду
PWR_MOTOR_MSG encM3_radData;    // радианы в секунду
ros::Publisher encDataRadM3("encDataRadM3",&encM3_radData);
#else // уставка по мощности в единицах ШИМ 0...1000
ros::Publisher encoderM3("encoderM3", &encoderM3_data);
#endif
/** ------------------------------------------------------------------------ */

/** Функция вызываемая периодически для отправки сообщения с данными ЭНКОДЕРА motor0
 * с периодом ENCODER_CONTROL_STEP */
void encoderM0_public(){
  dMotor.encoderDrives->encData[NUM_MOTOR_0]->getCount() ;  // читаем и вычисляем значение энкодера
  encoderM0_data.data = dMotor.encoderDrives->encData[NUM_MOTOR_0]->measure ;

  /** Показания энкодера в радианах в секунду
   * измерение в щелчках за интервал умножаем на количество радиан в щелчке
   * и умножаем на количество интервалов в секунде */
  encM0_radData.data = encoderM0_data.data * SPEED_RADIAN_RATIO * (1000 / ENCODER_CONTROL_STEP);

#ifdef RADIAN_SECOND
  encDataRadM0.publish(&encM0_radData); // публикуем данные энкодера в радианах в секунду
#else // уставка по мощности в единицах ШИМ 0...1000
  encoderM0.publish(&encoderM0_data) ; // публикуем данные энкодера в щелчках за измерительный интервал
#endif

}

/** Функция вызываемая периодически для отправки сообщения с данными ЭНКОДЕРА motor1
 * с периодом ENCODER_CONTROL_STEP */
void encoderM1_public(){
  dMotor.encoderDrives->encData[NUM_MOTOR_1]->getCount() ;  // читаем и вычисляем значение энкодера
  encoderM1_data.data = dMotor.encoderDrives->encData[NUM_MOTOR_1]->measure ;

  /** Показания энкодера в радианах в секунду */
  encM1_radData.data = encoderM1_data.data * SPEED_RADIAN_RATIO * (1000 / ENCODER_CONTROL_STEP);

#ifdef RADIAN_SECOND
  encDataRadM1.publish(&encM1_radData); // публикуем данные энкодера в радианах в секунду
#else // уставка по мощности в единицах ШИМ 0...1000
  encoderM1.publish(&encoderM1_data) ;
#endif
}

/** Функция вызываемая периодически для отправки сообщения с данными ЭНКОДЕРА motor2
 * с периодом ENCODER_CONTROL_STEP */
void encoderM2_public(){
  dMotor.encoderDrives->encData[NUM_MOTOR_2]->getCount() ;  // читаем и вычисляем значение энкодера
  encoderM2_data.data = dMotor.encoderDrives->encData[NUM_MOTOR_2]->measure ;

  /** Показания энкодера в радианах в секунду */
  encM2_radData.data = encoderM2_data.data * SPEED_RADIAN_RATIO * (1000 / ENCODER_CONTROL_STEP);

#ifdef RADIAN_SECOND
  encDataRadM2.publish(&encM2_radData); // публикуем данные энкодера в радианах в секунду
#else // уставка по мощности в единицах ШИМ 0...1000
  encoderM2.publish(&encoderM2_data) ;
#endif
}

/** Функция вызываемая периодически для отправки сообщения с данными ЭНКОДЕРА motor3
 * с периодом ENCODER_CONTROL_STEP */
void encoderM3_public(){
  dMotor.encoderDrives->encData[NUM_MOTOR_3]->getCount() ;  // читаем и вычисляем значение энкодера
  encoderM3_data.data = dMotor.encoderDrives->encData[NUM_MOTOR_3]->measure ;

  /** Показания энкодера в радианах в секунду */
  encM3_radData.data = encoderM3_data.data * SPEED_RADIAN_RATIO * (1000 / ENCODER_CONTROL_STEP);

#ifdef RADIAN_SECOND
  encDataRadM3.publish(&encM3_radData); // публикуем данные энкодера в радианах в секунду
#else // уставка по мощности в единицах ШИМ 0...1000
  encoderM3.publish(&encoderM3_data) ;
#endif
}

/** Обратный вызов при приёме сообщения */
void receiverCallback(const std_msgs::String & msg){
/** Здесь код выполняемый при получении сообщения  */
  chatter.publish(&msg);
}



///** Приведение типа сообщения PWR_MOTOR_MSG к типу int */
int msgRosToInt(PWR_MOTOR_MSG msg){
  /** умножаем пришедший float на 1000 и преобразуем в int */
  int pwm = static_cast <int> (msg.data * MAX_PWM_PULSE_COUNT);
  if(pwm > MAX_PWM_PULSE_COUNT) pwm = MAX_PWM_PULSE_COUNT;
  if(pwm < -MAX_PWM_PULSE_COUNT) pwm = -MAX_PWM_PULSE_COUNT;
  return pwm;
}

/** Тестовая функция */
void setDcPwr(int pwr) {

  /** Остановка двигателя */
  if(pwr == 0){
    htim1.Instance->CCR1 = 0;

    HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_RESET);
  }
  /** Двигатель вперёд */
  else if(pwr > 0) {

    HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_SET);
  }
  /** Двигатель назад */
  else{

    HAL_GPIO_WritePin(AIN1_MOTOR_0_GPIO_Port, AIN1_MOTOR_0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_MOTOR_0_GPIO_Port, AIN2_MOTOR_0_Pin, GPIO_PIN_RESET);
  }

  uint32_t mRegLoadPwm = static_cast <uint16_t> (abs(pwr)); /** Значение для загрузки в регистр ШИМ */

  /** Размещение задания мощности в диапазон MINIMUM_LIMIT_PWM ... MAXIMUM_LIMIT_PWM */
  uint32_t loadPwr = (mRegLoadPwm * (MAXIMUM_LIMIT_PWM - MINIMUM_LIMIT_PWM) /1000) + MINIMUM_LIMIT_PWM;


  M0_DC_PWM = static_cast <uint16_t> (loadPwr); /** Загружаем регистр ШИМ */
//  mPwrState = pwr;
}

/**
 * При уставке скорости в радианах в секунду
 * Функция выставляет скорость как уставку энкодера для заданного двигателя
 * и стартует процесс регулировки скорости двигателя к уставке
 * @param numMotor         - номер двигателя
 * @param encoderSetting   - уставка энкодера
 */
void setEncoderSpeed(int numMotor, float encoderSetting){
  /** Приводим значение к типу Int и загружаем уставку энкодера */
  dMotor.arrPwrm[numMotor]->powerProcess->encoderSetting = static_cast <int> (encoderSetting);
  /** Активируем процесс контроля показаний энкодера и приведение его к уставке */
  dMotor.arrPwrm[numMotor]->powerProcess->ss_enc = ENC_SET_ACTIVE;

  /** Здесь код выполняемый при получении сообщения  */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/** Обратный вызов motor0_pwr */
void recMotor0(const PWR_MOTOR_MSG & msg){

  /** При уставке скорости в радианах в секунду */
#ifdef RADIAN_SECOND
  /** Скорость в радианах в секунду делим на коэффициент пропорциональности
   * получаем уставку энкодера в щелчках в секунду
   * затем деление на количество замеров энкодера в секунду - получаем количество
   * щелчков энкодера на измерительный интервал */
  float encoderSetting = (msg.data / SPEED_RADIAN_RATIO) / (1000 / ENCODER_CONTROL_STEP);

  setEncoderSpeed(NUM_MOTOR_0, encoderSetting);

#else // уставка по мощности в единицах ШИМ 0...1000

  /** преобразуем и загружаем пришедшее значение мощности, как новое состояние двигателя */
  dMotor.arrPwrm[NUM_MOTOR_0]->mPwrNewState = dMotor.msgRosToInt(msg);
  /** Стартуем процесс изменения мощности */
  dMotor.arrPwrm[NUM_MOTOR_0]->powerProcess->ss = MPDC_START;

//  setDcPwr(msgRosToInt(msg)); // Управление двигателем с помощью тестовой функции

  /** Отправка в ROS topic kek полученного сообщения */
  sprintf(logMsgBuffer, "%f", msg.data);
  str_msg.data = logMsgBuffer;
  chatter.publish(&str_msg);

  /** Здесь код выполняемый при получении сообщения  */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}

/** Обратный вызов motor1_pwr */
void recMotor1(const PWR_MOTOR_MSG & msg){

  /** При уставке скорости в радианах в секунду */
#ifdef RADIAN_SECOND

  float encoderSetting = (msg.data / SPEED_RADIAN_RATIO) / (1000 / ENCODER_CONTROL_STEP);

  setEncoderSpeed(NUM_MOTOR_1, encoderSetting);

#else // уставка по мощности в единицах ШИМ 0...1000
  /** преобразуем и загружаем пришедшее значение мощности, как новое состояние двигателя */
  dMotor.arrPwrm[NUM_MOTOR_1]->mPwrNewState = dMotor.msgRosToInt(msg);
  /** Стартуем процесс изменения мощности */
  dMotor.arrPwrm[NUM_MOTOR_1]->powerProcess->ss = MPDC_START;

  /** Отправка в ROS topic kek полученного сообщения */
  sprintf(logMsgBuffer, "%f", msg.data);
  str_msg.data = logMsgBuffer;
  chatter.publish(&str_msg);

  /** Здесь код выполняемый при получении сообщения  */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}

/** Обратный вызов motor2_pwr */
void recMotor2(const PWR_MOTOR_MSG & msg){

  /** При уставке скорости в радианах в секунду */
#ifdef RADIAN_SECOND

  float encoderSetting = (msg.data / SPEED_RADIAN_RATIO) / (1000 / ENCODER_CONTROL_STEP);

  setEncoderSpeed(NUM_MOTOR_2, encoderSetting);

#else // уставка по мощности в единицах ШИМ 0...1000
  /** преобразуем и загружаем пришедшее значение мощности, как новое состояние двигателя */
  dMotor.arrPwrm[NUM_MOTOR_2]->mPwrNewState = dMotor.msgRosToInt(msg);
  /** Стартуем процесс изменения мощности */
  dMotor.arrPwrm[NUM_MOTOR_2]->powerProcess->ss = MPDC_START;

  /** Отправка в ROS topic kek полученного сообщения */
  sprintf(logMsgBuffer, "%f", msg.data);
  str_msg.data = logMsgBuffer;
  chatter.publish(&str_msg);

  /** Здесь код выполняемый при получении сообщения  */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}

/** Обратный вызов motor3_pwr */
void recMotor3(const PWR_MOTOR_MSG & msg){

  /** При уставке скорости в радианах в секунду */
#ifdef RADIAN_SECOND

  float encoderSetting = (msg.data / SPEED_RADIAN_RATIO) / (1000 / ENCODER_CONTROL_STEP);

  setEncoderSpeed(NUM_MOTOR_3, encoderSetting);

#else // уставка по мощности в единицах ШИМ 0...1000
  /** преобразуем и загружаем пришедшее значение мощности, как новое состояние двигателя */
  dMotor.arrPwrm[NUM_MOTOR_3]->mPwrNewState = dMotor.msgRosToInt(msg);
  /** Стартуем процесс изменения мощности */
  dMotor.arrPwrm[NUM_MOTOR_3]->powerProcess->ss = MPDC_START;

  /** Отправка в ROS topic kek полученного сообщения */
  sprintf(logMsgBuffer, "%f", msg.data);
  str_msg.data = logMsgBuffer;
  chatter.publish(&str_msg);

  /** Здесь код выполняемый при получении сообщения  */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#endif
}

void ledToggle(){
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  str_msg.data = hello;
  chatter.publish(&str_msg);

  log_string(USB == Hello World - test message\r\n)

}

void txLoggerInterval(){
  mLog.transmitLogBuffer(); // отправка буфера лога в классе интервал
}

void linkSpinOnce(){
  nh.spinOnce();
}

/** Функция вызова процесса плавного изменения мощности */
void pwmMotorsProcess(){

  /** При уставке скорости в радианах в секунду */
#ifdef RADIAN_SECOND
  /** Обход процессов всех двигателей при задании скорости в радианах в секунду */
  for(int num = 0; num < MOTORS_QUA; num++){
    dMotor.arrPwrm[num]->powerProcess->encoderSettingControl(num) ;
  }

#endif
  /** Обход процессов всех двигателей при задании мощности */
  for(int num = 0; num < MOTORS_QUA; num++){
    dMotor.arrPwrm[num]->powerProcess->difProcess(num);
  }
//  dMotor.arrPwrm[0]->powerProcess->difProcess(0); // Тест для motor0
//  dMotor.arrPwrm[1]->powerProcess->difProcess(1); // Тест для motor1
//  dMotor.arrPwrm[2]->powerProcess->difProcess(2); // Тест для motor2
//  dMotor.arrPwrm[3]->powerProcess->difProcess(3); // Тест для motor3

}

/** Инициализация */
void setup(void){

/** Объекты класса интервал для функций основного цикла
 * Функция обработчика будет вызываться каждый раз через заданный интервал в миллисекундах
 * из loop() через arrIP.mainIntervalActivate()
 * посредством указателя на функцию */
  arrIP.createNewInterval(LOG_UART_TX_DMA_TIMEOUT, txLoggerInterval); // отправка лога
//  arrIP.createNewInterval(2000, ledToggle, 1);  // управление светодиодом
  arrIP.createNewInterval(998, linkSpinOnce, 2);  // основной отклик на ROS
  arrIP.createNewInterval(ENCODER_CONTROL_STEP, encoderM0_public, 3);  // публикатор энкодера M0
  arrIP.createNewInterval(ENCODER_CONTROL_STEP, encoderM1_public, 4);  // публикатор энкодера M1
  arrIP.createNewInterval(ENCODER_CONTROL_STEP, encoderM2_public, 5);  // публикатор энкодера M2
  arrIP.createNewInterval(ENCODER_CONTROL_STEP, encoderM3_public, 6);  // публикатор энкодера M3
  arrIP.createNewInterval(STEP_POWER_CONTROL, pwmMotorsProcess, 7); // процесс управления мощностью двигателей


  /** инициализация двигателей */
  dMotor.initMotor() ;

  /** инициализация энкодеров */
  dMotor.encoderDrives->init() ;

  /** Инициализаця элементов ROS */
  nh.initNode();                // инициализация узла ROS

  nh.advertise(chatter);     // инициализация тестового издателя "kek"
  nh.subscribe(rec);         // инициализация тестового подписчика "lol"

  nh.subscribe(motor0_pwr);  // задание мощности двигателя 0
  nh.subscribe(motor1_pwr);  // задание мощности двигателя 1
  nh.subscribe(motor2_pwr);  // задание мощности двигателя 2
  nh.subscribe(motor3_pwr);  // задание мощности двигателя 3

#ifdef RADIAN_SECOND // При уставке скорости в радианах в секунду
  nh.advertise(encDataRadM0);  // издатель энкодера motor0 в радианах в секунду
  nh.advertise(encDataRadM1);  // издатель энкодера motor1 в радианах в секунду
  nh.advertise(encDataRadM2);  // издатель энкодера motor2 в радианах в секунду
  nh.advertise(encDataRadM3);  // издатель энкодера motor2 в радианах в секунду
#else // уставка по мощности в единицах ШИМ 0...1000
  nh.advertise(encoderM0);   // издатель энкодера motor0
  nh.advertise(encoderM1);   // издатель энкодера motor1
  nh.advertise(encoderM2);   // издатель энкодера motor2
  nh.advertise(encoderM3);   // издатель энкодера motor3
#endif

}

