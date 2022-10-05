//
// Created by vitaliy on 29.03.2022.
//

#ifndef C60APP_LOGGER_HPP
#define C60APP_LOGGER_HPP


#include "../Inc/preference.h"
#include "string.h"
#include "stdio.h"


#define UART_LOG_TRANSMIT     // передача данных буфера лога через UART
//#define USB_COM_LOG_TRANSMIT  // передача данных буфера лога через USB виртуальный COM порт

/**
 * Лог отправляется в порт из кольцевого буфера через фиксированный интервал времени
 * Для реализации интервала вызова используется класс Interval
 *
 * При использовании STM32 с встроенным USB добавляется возможность отправки лога через
 * USB посредством виртуального COM порта
 *
 * */

typedef enum{
    RING_BUFFER_READY = 0,            // буфер готов к работе
    RING_BUFFER_OVERFLOW = 0x01,      // переполненние кольцевого буфера
    RING_BUFFER_BLOCK = 0x02,         // блокировка буфера
    RING_BUFFER_TX_SPLIT = 0x04       // передана половина сообщения
}log_buffer_flags; // флаги статуса кольцевого буфера

typedef enum{
	BLOCK_UART = 0x01,  // флаг занятсти UART при передаче лога
}logger_flags_and_variable;

/**
 * Макрос для отправки строки в лог
 * пример:
 *
 * log_string(Test log_string macros)
 */
#define log_string( inString )  uint8_t str[] = # inString ;\
    mLog.messageLog(str, sizeof(str) - 1);


/** Класс. Кольцевой буфер, накапливающий сообщения */
class RingBuffer {

public:
    /** стартовый адрес неотправленной последовательности */
    uint16_t addressStart;

    /** количество неотправленных символов */
    uint16_t count;

    /** адрес для загрузки нового сообщения */
    uint16_t addressEnd;

    /** Флаги кольцевого буфера */
    uint8_t flags;

    /** Кольцевой буфер в памяти */
    uint8_t *memBuffer;
    /** размер кольцевого буфера */
    uint16_t length;


    /**  конструктор */
    RingBuffer(uint8_t *logBuffer, uint16_t sizeBuffer){
      memBuffer = logBuffer;
      length = sizeBuffer;
      addressStart =0;
      count = 0;
      addressEnd = 0;
      flags = 0;
    }

//    ~RingBuffer() ; // деструктор

    /** Функция возвращает Количество непереданных символов в буфере */
    uint16_t getCount(void){
      return count;
    }

    /**
     * @brief Статус(флаги) состояния буфера лога
     * 
     * @return uint8_t 
     */
    uint8_t getStatus(void);

    /**
     * @brief Блокировка буфера
     */
    void bufferBlock(void);

    /**
     * @brief Разблокировка буфера
     */
    void bufferUnBlock(void);

    /**
     * Загрузка сообщения в кольцевой буфер
     * 
     * @param str      буфер сообщения
     * @param lengthInStr   длина сообщения
     */
    void messageToRingBuffer(uint8_t *str, uint16_t lengthInStr);

    /**
     * Отправка накопленных в кольцевом буфере сообщений TX UART через DMA
     * Вызывается из прерывания
     * 
     * @param uart  привязанный UART
     * @param dma   привязанный DMA
     */
    void txDmaUart(UART_HandleTypeDef uart, DMA_HandleTypeDef dma);

    /**
     * Отправка накопленных в кольцевом буфере данных через USB виртуальный COM порт
     */
    void txUsbComPort();

    /**
     * Сброс буфера
     */
    void resetBuffer(void){
      addressStart = 0;
      addressEnd = 0;
      count = 0;
    }

};


/**
 * @brief
 *
 * =====   В основном цикле   =======
 * сообщение размещается в кольцевой буфер RingBuffer функцией messageLog(uint8_t *str, uint16_t lenStr)
 * накопленные сообщения с интревалом LOG_UART_TX_DMA_TIMEOUT проверяются
 * в прерывании TIM1_CC_IRQHandler и отправляются в UART функцией logToDmaUart_IT()
 *
 * =====   В прерывании    ======
 * сообщение размещается в отдельный буфер logIT функцией messageLogIT(uint8_t *str, uint16_t lenStr)
 * в прерывании, перед отправкой в TX DMA заполненная часть буфера отправляется в буфер log
 *
 *
 */
class Logger {

  /** буфер лога основного цикла */
  uint8_t BufferLog[LOG_TX_SIZE];
  /** буфер лога основного цикла */
  RingBuffer log {BufferLog, LOG_TX_SIZE};

  /** буфер лога прерывания */
  uint8_t logBufferIT[LOG_IT_SIZE];
  /** буфер лога прерывания */
  RingBuffer logIT {logBufferIT, LOG_IT_SIZE};

public:

  /** Конструктор */
  Logger(){}

  /** Размещение сообщения в лог в основном цикле программы
   * @param str      буфер сообщения
   * @param lenStr   длина сообщения
   */
  void messageLog(uint8_t *str, uint16_t lenStr);

  /**
   * Отправка накопленных в кольцевом буфере сообщений TX UART через DMA
   */
  void logToDmaUart_IT();

  /** Размещение сообщения в лог в прерывании
   * @param str      буфер сообщения
   * @param lenStr   длина сообщения
   */
  void messageLogIT(uint8_t *str, uint16_t lenStr);

  /**
   * Перенос из буфера logIT накопленных данных в буфер log
   * со сбросом исходного logIT буфера
   * @param target
   */
  void logITDataToLog(void);

  /**
   * Отправка буфера лога в назначенный порт
   */
  void transmitLogBuffer();


};





#endif //C60APP_LOGGER_HPP
