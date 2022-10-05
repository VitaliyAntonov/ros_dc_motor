//
// Created by vitaliy on 06.04.2022.
//

#ifndef C60APP_PROCESS_HPP
#define C60APP_PROCESS_HPP


#include "../Inc/preference.h"
#include "../../Drivers/CMSIS/Include/cmsis_compiler.h"


/*
 * Структура для контроля параметров процессов
 */
typedef struct{
    uint8_t flags;   // Флаги
    uint8_t ss;      // состояние/этап
}process_control_parametres;

typedef enum{
    BLOCK_FLAG = 0x01,
}process_control_sys_flags;

/**
 * Класс для организации последовательного процесса
 */
class Process {
public:
    uint8_t flags;   // Флаги
    uint8_t ss;      // состояние/этап

};


#endif //C60APP_PROCESS_HPP
