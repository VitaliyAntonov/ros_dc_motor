//
// Created by vitaliy on 31.05.2022.
//

#ifndef BLDC_MOTORF401_BLDCMOTORS_H
#define BLDC_MOTORF401_BLDCMOTORS_H

#include "../Inc/preference.h"


/** Данные настройки BLDC двигателя */
typedef struct  {

    /** Захваченная длительность текущего импульса датчика Холла */
    uint16_t captureValue;

    /** счётчик захватов  */
    uint16_t captureQua;

    /** Усреднённое значение длительности импульса датчика Холла */
    uint16_t averageCaptureValue;

}motorBldcSetting;

class BldcMotors {

    motorBldcSetting m0setting;

    /** Конструктор класса для всех BLDC двигателей */
    BldcMotors();

};


#endif //BLDC_MOTORF401_BLDCMOTORS_H
