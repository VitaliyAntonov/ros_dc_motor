//
// Created by vitaliy on 22.04.2022.
//

#ifndef TANK_DRIVEROS_H
#define TANK_DRIVEROS_H


#include "../Inc/preference.h"
#include "PowerMotorProcess.h"
#include "Encoder.h"

class Encoder;

typedef enum{
    NUM_MOTOR_0 = 0,
    NUM_MOTOR_1 = 1,
    NUM_MOTOR_2 = 2,
    NUM_MOTOR_3 = 3,

}motorNumbers;

typedef enum{
  MOTOR_STOP = 0,
  MOTOR_FORWARD = 1,
  MOTOR_BACK = 2
}motor_direction;


/** Класс ячеек для управления мощностью двигателя */
class PwrM{
public:
    /** Состояние мощности двигателя в данный момент */
    int mPwrState;

    /** Новое состояние мощности в которое переходит двигатель в результате команды */
    int mPwrNewState;

    /** Номер двигателя в созданном объекте управления */
    uint8_t motorNumber;

    /** Направление вращения двигателя */
    int direction;

    /** Указатель на процесс контроля мощности */
    MotorDifferenceProcess *powerProcess;

    /** Конструктор класса состояния мощности двигателя и список инициализации */
    PwrM(): mPwrState(0), mPwrNewState(0){};

    virtual void motorStop(){};

    virtual void motorForward(){};

    virtual void motorBack(){};

    virtual void setPwr(int pwr){};
};

class PwrMotor0 : public PwrM{
public:
    PwrMotor0(){motorNumber = NUM_MOTOR_0;}
    void motorStop();
    void motorForward();
    void motorBack();
    void setPwr(int pwr);
};

class PwrMotor1 : public PwrM{
public:
    PwrMotor1(){motorNumber = NUM_MOTOR_1;}
    void motorStop();
    void motorForward();
    void motorBack();
    void setPwr(int pwr);
};

class PwrMotor2 : public PwrM{
public:
    PwrMotor2(){motorNumber = NUM_MOTOR_2;}
    void motorStop();
    void motorForward();
    void motorBack();
    void setPwr(int pwr);
};

class PwrMotor3 : public PwrM{
public:
    PwrMotor3(){motorNumber = NUM_MOTOR_3;}
    void motorStop();
    void motorForward();
    void motorBack();
    void setPwr(int pwr);
};

/** Класс контроля всех двигателей устройства */
class DriveRos {
public:

    /** Массив указателей значений ячеек для управления мощностью двигателя */
    PwrM *arrPwrm[MOTORS_QUA];

    /** Конструктор класса контроля всех двигателей устройства */
    DriveRos();

    /** указатель на данные энкодеров двигателей */
    Encoder *encoderDrives;

    /** Инициализация двигателей */
    void initMotor();

    /** Приведение типа сообщения PWR_MOTOR_MSG к типу int */
    int msgRosToInt(PWR_MOTOR_MSG msg);

};


#endif //TANK_DRIVEROS_H
