//
// Created by vitaliy on 15.07.2022.
/** Виталий Антонов  kaligraf@yandex.ru */
//
#include "../Inc/cppMain.h"
#include "../App/ros_main.h"
#include "../Process/Interval.h"



/** массив данных об интервалах */
extern Interval arrIP;

void cppMain() {

  setup();


  while (1)
  {
    arrIP.mainIntervalActivate();

  }


}




