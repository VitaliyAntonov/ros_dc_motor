//
// Created by vitaliy on 29.04.2022.
/** Виталий Антонов  kaligraf@yandex.ru */
//

#include "Encoder.h"

extern DriveRos dMotor;

/** Пересчёт считанных значений счётчика энкодера */
void EncoderData::calcMeasure(int numMotor) {

    /** при направлении вперёд */
    if(dMotor.arrPwrm[numMotor]->direction == MOTOR_FORWARD){
      /** Переполнение */
      if(currCounter < prevCounter){
        measure = currCounter + 65535 - prevCounter ;
      }
      // нет переполнения
      else{
        measure = currCounter - prevCounter ;
      }
    }

    /** при направлении назад */
    if(dMotor.arrPwrm[numMotor]->direction == MOTOR_BACK){
      /** Переполнение - переход через 0 */
      if(currCounter > prevCounter){
        measure = 0 - prevCounter + (currCounter - 65536) ;
      }
        // нет переполнения
      else{
        measure = currCounter - prevCounter ;
      }
    }

    /** при СТОП */
    if(dMotor.arrPwrm[numMotor]->direction == MOTOR_STOP){
      measure = 0 ;
    }

    /** Фильтр от случайных значений при переходе через 0 */
    if(measure > 250) measure = 0 ;
    if(measure < -250) measure = 0 ;

    prevCounter = currCounter ;             // фиксация текущего замера
}

void EncMotor0::getCount(){
  currCounter = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_M0);
  calcMeasure(NUM_MOTOR_0);
}

void EncMotor1::getCount(){
  currCounter = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_M1);
  calcMeasure(NUM_MOTOR_1);
}

void EncMotor2::getCount(){
  currCounter = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_M2);
  calcMeasure(NUM_MOTOR_2);
}

void EncMotor3::getCount(){
  currCounter = __HAL_TIM_GET_COUNTER(&ENCODER_TIM_M3);
  calcMeasure(NUM_MOTOR_3);
}

/** конструктор */
Encoder::Encoder(){
  /** создаём объекты и фиксируем ссылки в массив */
  encData[NUM_MOTOR_0] = new EncMotor0;
  encData[NUM_MOTOR_1] = new EncMotor1;
  encData[NUM_MOTOR_2] = new EncMotor2;
  encData[NUM_MOTOR_3] = new EncMotor3;

  /** загружаем первоначальное состояние счётчиков */
  encData[NUM_MOTOR_0]->currCounter = ENCODER_TIM_M0.Instance->CNT;
  encData[NUM_MOTOR_1]->currCounter = ENCODER_TIM_M1.Instance->CNT;
  encData[NUM_MOTOR_2]->currCounter = ENCODER_TIM_M2.Instance->CNT;
  encData[NUM_MOTOR_3]->currCounter = ENCODER_TIM_M3.Instance->CNT;
}

/** инициализация энкодеров */
void Encoder::init(){
  /** Включаем таймеры энкодеров */
  HAL_TIM_Encoder_Start(&ENCODER_TIM_M0, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENCODER_TIM_M1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENCODER_TIM_M2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENCODER_TIM_M3, TIM_CHANNEL_ALL);
}

/** Опрос состояния энкодеров */
void Encoder::getEncodersCount() {
  for(int num = 0; num < MOTORS_QUA; num++){
    encData[num]->getCount();
  }
}





