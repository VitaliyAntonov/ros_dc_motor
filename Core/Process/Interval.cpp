//
// Created by vitaliy on 20.04.2022.
//

#include "Interval.h"

/** Класс служит для организации периодического вызова обработчиков
 * через заданный интервал времени */

/** Экземпляр класса */
Interval arrIP;  // массив данных об интервалах



/**
 * Добавление интервала в массив
 *
 * @param millisecond   - длительность в миллисекундах между вызовами обработчика
 * @param foo           - указатель на обработчик интервала
 * @param time_offset   - смещение начала интервала при его инициализации
 *                         служит для того, чтобы при одинаковых длительностях интервалов
 *                         разнести вызов их обработчиков на фиксированное значение
 *                         в миллисекундах
 *
 */
void Interval::createNewInterval(uint32_t millisecond, void (*foo)(), uint32_t time_offset){
  if(count < INTERVALS_QUA){        // проверка количества интервалов

    /** объект с параметрами интервала */
    intervalParam ip;

    /** инициализация длительности интервала */
    ip.interval = millisecond;

    /** фиксация начального значения системного таймера */
    ip.startTick = HAL_GetTick();
    /** проверка и добавка смещения */
    if(count > 0){ // если уже есть интервалы в массиве
      ip.startTick += time_offset;
    }

    /** фиксируем указатель на функцию - обработчик при активации */
    ip.functionGran = foo;

    /** добавляем интервал в массив интервалов */
    arrInterval[arrIP.count] = ip;

    /** увеличиваем номер в массиве для следующего экземпляра интервала */
    count += 1;

  }
  else{
    /** === Ошибка. Превышено максимальное количество заданных интервалов === */
  }
}


/** обработчик прерывания от системного таймера */
void Interval::interruptOnSystemTick(){

  /** Если созданы интервалы */
  if(arrIP.count > 0){

    /** обход всех интервалов */
    for(uint8_t cnt = 0; cnt < arrIP.count; cnt++){

      if((HAL_GetTick() - arrIP.arrInterval[cnt].startTick) >= arrIP.arrInterval[cnt].interval){
        /** Интервал закончен */
        /** активируем флаг разрешения обработки */
        arrIP.arrInterval[cnt].numGran = INTERVAL_PERMIT_ON;
        /** перезагружаем стартовое значение для следующего срабатывания */
        arrIP.arrInterval[cnt].startTick = HAL_GetTick();
      }
    }
  }
}

/** Проверка флагов активации(номер гранулы) и включение обработчиков */
void Interval::mainIntervalActivate(){
  /** Если созданы интервалы */
  if(arrIP.count > 0){
    /** обход всех интервалов */
    for(uint8_t cnt = 0; cnt < arrIP.count; cnt++){
      /** проверка флага активации */
      if(arrIP.arrInterval[cnt].numGran != 0){
        /** вызов функции - обработчика */
        arrIP.arrInterval[cnt].functionGran();
        /** сброс флага активации */
        arrIP.arrInterval[cnt].numGran = 0;
      }
    }
  }
}

