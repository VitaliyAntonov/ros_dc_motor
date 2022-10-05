//
// Created by vitaliy on 20.04.2022.
//



#ifndef TANK_INTERVAL_H
#define TANK_INTERVAL_H

/** Класс служит для организации периодического вызова обработчиков
 * через заданный интервал времени */

#include "../Inc/preference.h"

typedef enum {
    /** Флаг - Разрешение процесса */
    INTERVAL_PERMIT_ON = 0x01,
}main_process_activate_flags;

/** Параметры одного интервала */
typedef struct {
    /** номер гранулы процесса. При выключенном процессе = 0 */
    uint8_t  numGran = 0;
    /** стартовое значение системного таймера при активации */
    uint32_t startTick;
    /** пауза как количество прерываний(обычно в миллисекундах) */
    uint32_t interval;
    /** указатель на функцию - обработчик при активации */
    void (*functionGran)();
}intervalParam;

/** Массив данных об интервалах */
class Interval {
public:
    /** количество созданных интервалов для обработки */
    uint8_t count;

    /** массив параметров интервалов */
    intervalParam arrInterval[INTERVALS_QUA];

    /** Конструктор со списком параметров инициализации */
    Interval(): count(0){};

    /** обработчик прерывания от системного таймера */
    void interruptOnSystemTick();

    /** Проверка флагов активации(номер гранулы) и включение обработчиков */
    void mainIntervalActivate();

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
     * @return  - возвращает 1 при успешном добавлении
     */
    void createNewInterval(uint32_t millisecond, void (*foo)(), uint32_t time_offset = 0);

};


#endif //TANK_INTERVAL_H
