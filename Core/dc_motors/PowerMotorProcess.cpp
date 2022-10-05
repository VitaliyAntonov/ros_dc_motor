//
// Created by vitaliy on 26.04.2022.
//


#include "PowerMotorProcess.h"
#include "DriveRos.h"

#include "../Process/Process.hpp"
#include <math.h>


extern DriveRos dMotor; // Экземпляр класса управления двигателями

/**
 * контроль процесса плавного изменения мощности двигателя
 * @param numMotor  - номер двигателя
 */
void MotorDifferenceProcess::difProcess(int numMotor) {

  /** Отрабатываем только при активированном процессе */
  if(ss > MPDC_FREE){ // проверка активен ли процесс
    if((flags & BLOCK_FLAG) == 0){ // проверка блокировки процесса

      /** ============== Старт входа в процесс ================ */
      if(ss == MPDC_START){
        /** Разность между новой и текущей мощностью */
        int difPower = dMotor.arrPwrm[numMotor]->mPwrNewState - dMotor.arrPwrm[numMotor]->mPwrState;
        /** Сравнение. Определение направления */
        if(difPower == 0){
          ss = MPDC_FREE; // при равенстве уставки и текущей мощности останавливаем процесс
        }
        else if(difPower > 0){ // уставка больше текущей
          ss = MPDC_FORWARD; // мощность вперёд
        }
        else{
          ss = MPDC_BACK; // мощность назад
        }
      }
      /** --------------------------------------------- */

      /** =================== мощность вперёд ================= */
      if(ss == MPDC_FORWARD){
        /** увеличенная мощность */
        int pwrValue = dMotor.arrPwrm[numMotor]->mPwrState + INCREMENT_PWR_VALUE;

        if(pwrValue >= dMotor.arrPwrm[numMotor]->mPwrNewState){ // проверка на достижение уставки мощности
          /** мощность достигла уставки */
          pwrValue = dMotor.arrPwrm[numMotor]->mPwrNewState;
          /** мощность в двигатель */
          dMotor.arrPwrm[numMotor]->setPwr(pwrValue);
          /** останавливаем процесс */
          ss = MPDC_FREE;
        }
        else{
          /** Уставка не достигнута - проверка на прохождение нуля */
          if(dMotor.arrPwrm[numMotor]->mPwrState < 0){
            if(pwrValue > 0){
              /** переход через 0  */
              pwrValue = 0;
            }
          }
          /** мощность в двигатель */
          dMotor.arrPwrm[numMotor]->setPwr(pwrValue);
        }
      }
      /** --------------------------------------------- */

      /** =========== мощность назад ================= */
      if(ss == MPDC_BACK){
        /** уменьшенная мощность */
        int pwrValue = dMotor.arrPwrm[numMotor]->mPwrState - INCREMENT_PWR_VALUE;

        if(pwrValue <= dMotor.arrPwrm[numMotor]->mPwrNewState){ // проверка на достижение уставки мощности
          /** мощность достигла уставки */
          pwrValue = dMotor.arrPwrm[numMotor]->mPwrNewState;
          /** мощность в двигатель */
          dMotor.arrPwrm[numMotor]->setPwr(pwrValue);
          /** останавливаем процесс */
          ss = MPDC_FREE;
        }
        else{
          /** Уставка не достигнута - проверка на прохождение нуля */
          if(dMotor.arrPwrm[numMotor]->mPwrState > 0){
            if(pwrValue < 0){
              /** переход через 0  */
              pwrValue = 0;
            }
          }
          /** мощность в двигатель */
          dMotor.arrPwrm[numMotor]->setPwr(pwrValue);
        }


      }
      /** --------------------------------------------- */

    }
  }


}


/**
 * Процесс контроля уставки энкодера
 * Функция вызывается с заданной периодичностью и изменяет мощность двигателя
 * в зависимости от отклонения показаний энкодера от уставки энкодера encoderSetting
 */
void MotorDifferenceProcess::encoderSettingControl(int numMotor) {

  /** Работает только при активированном процессе контроля скорости
   *  и остановленном процессе изменения мощности */
  if (ss_enc == ENC_SET_ACTIVE) {
    if (ss == MPDC_FREE) {

      // Текущее состояние энкодера
      int realEncoderData = dMotor.encoderDrives->encData[numMotor]->measure;
      // Уставка энкодера: encoderSetting

      /** Разницу между уставкой энкодера и реальными показаниями преобразуем
       * в разницу в мощности ( DELTA_SQRT_MULTIPLIER - коэффициент )   */
      int delta = encoderSetting - realEncoderData ;

      /** Новое значение мощности двигателя */
      int sign = 1 ;
      if(delta < 0) sign = -1 ; // сохраняем знак дельты

      double dDelta = static_cast <double> (abs(delta));  // в double для извлечения корня
      double sqrtDelta = sqrt(dDelta);                    // извлекаем корень
      int resultDelta = static_cast <int> (sqrtDelta);    // преобразуем обратно в int
      /** Умножаем на коэффициент, восстанавливаем знак и корректируем уставку мощности */
      dMotor.arrPwrm[numMotor]->mPwrNewState += (resultDelta * sign * DELTA_SQRT_MULTIPLIER);

      /** Активируем процесс изменения мощности */
      ss = MPDC_START;

    }
  }


}


