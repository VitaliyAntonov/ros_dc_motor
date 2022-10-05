//
// Created by vitaliy on 26.04.2022.
//


#ifndef TANK_POWERMOTORPROCESS_H
#define TANK_POWERMOTORPROCESS_H


#include "../Inc/preference.h"
#include "../Process/Process.hpp"




/** Процесс контроля текущей и новой мощности */
typedef enum{
    /** Процесс остановлен */
    MPDC_FREE = 0,
    /** Старт процесса, первоначальные настройки процесса */
    MPDC_START = 1,
    /** мощность вперёд */
    MPDC_FORWARD = 2,
    /** мощность назад */
    MPDC_BACK = 3,
}motorPowerDifferenceControl;

/** Процесс контроля скорости двигателя в зависимости от показаний энкодера */
typedef enum{
    /** Процесс остановлен */
    ENC_SET_FREE = 0,
    /** Контроль энкодера */
    ENC_SET_ACTIVE = 1,
}motorEncoderSettingControlProcess;

/** Процесс контроля текущей и новой мощности двигателя */
class MotorDifferenceProcess {
public:

    uint8_t flags;      // Флаги
    uint8_t ss;         // состояние/этап

    uint8_t ss_enc;     // состояние/этап процесса контроля уставки энкодера
    int encoderSetting; // Задание скорости - щелчков энкодера за измерительный интервал

    /** Конструктор и список инициализации */
    MotorDifferenceProcess() : flags(0), ss(0), ss_enc(0), encoderSetting(0) {};

    /**
     * контроль процесса плавного изменения мощности двигателя
     * @param numMotor  - номер двигателя
     */
    void difProcess(int numMotor);

    /**
     * Процесс контроля уставки энкодера
     * Функция вызывается с заданной периодичностью и изменяет мощность двигателя
     * в зависимости от отклонения показаний энкодера от уставки энкодера encoderSetting
     */
    void encoderSettingControl(int numMotor);
};


#endif //TANK_POWERMOTORPROCESS_H
