//
// Created by vitaliy on 29.04.2022.
/** Виталий Антонов  kaligraf@yandex.ru */
//

#ifndef TANK_ENCODER_H
#define TANK_ENCODER_H

#include "../Inc/preference.h"
#include "DriveRos.h"

/**
 * Экземпляр класса Encoder создаётся в конструкторе класса DriveRos
 * контроля всех двигателей устройства
 */

class EncoderData{
public:
    /** Предыдущее значение счётчика */
    int prevCounter;

    /** Считанное значение счётчика */
    int currCounter;

    /** Данные измерения - количество импульсов на интервал ENCODER_CONTROL_STEP */
    int measure;

    float speedRadian;

    /** Расчёт измерения энкодера */
    void calcMeasure(int numMotor);

    /** Чтение счётчика таймера энкодера */
    virtual void getCount(){};
};

class EncMotor0 : public EncoderData{
public:
    void getCount();
};

class EncMotor1 : public EncoderData{
public:
    void getCount();
};

class EncMotor2 : public EncoderData{
public:
    void getCount();
};

class EncMotor3 : public EncoderData{
public:
    void getCount();
};

class Encoder {
public:
    /** массив ссылок на данные энкодеров */
    EncoderData *encData[MOTORS_QUA];

    /** конструктор */
    Encoder();

    /** инициализация энкодеров */
    void init();

    /** Опрос состояния энкодеров */
    void getEncodersCount();

};


#endif //TANK_ENCODER_H
