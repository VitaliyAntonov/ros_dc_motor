//
// Created by vitaliy on 29.03.2022.
//

#include "Logger.hpp"
#include "../../USB_DEVICE/App/usbd_cdc_if.h"

/** Создаём объект класса Логгер */
Logger mLog;


/** реализация
 * Загрузка сообщения в кольцевой буфер
 * 
 * @param str      буфер сообщения
 * @param lengthInStr   длина сообщения
 */
void RingBuffer::messageToRingBuffer(uint8_t *str, uint16_t lengthInStr){
  
  if((count + lengthInStr) <= length ){ // Проверка переполнения буфера
    
    flags &= ~RING_BUFFER_OVERFLOW;  // Сброс флага переполнения

    if ((addressEnd + lengthInStr) < length){ // проверка достигнут ли последний адрес в памяти
      // всё сообщение размещается одним блоком без разрывов
      memcpy(memBuffer + addressEnd, str, lengthInStr); // copy string no bufer
      addressEnd += lengthInStr; // адрес для размещения следующего сообщения
    }
    else{ // часть сообщения размещаем в конце буфера, часть в начале буфера
      uint16_t lenEnd = length - addressEnd; // количество символов сообщенния в конец буфера
      if (lenEnd>0){
				memcpy(memBuffer + addressEnd, str, lenEnd);
			}
      memcpy(memBuffer, str + lenEnd, lengthInStr - lenEnd);
      addressEnd = lengthInStr - lenEnd;
    }
    /** новое количество символов для отправки */
    count += lengthInStr;
  }
  else{   // Buffer Overflow
    flags |= RING_BUFFER_OVERFLOW;
  }

}


/** реализация
 * Отправка накопленных в кольцевом буфере сообщений TX UART через DMA
 * Вызывается из прерывания
 * 
 * @param uart  привязанный UART
 * @param dma   привязанный DMA
 */
void RingBuffer::txDmaUart(UART_HandleTypeDef uart, DMA_HandleTypeDef dma){

  if((flags & RING_BUFFER_BLOCK) == 0){ if (count > 0 ){
			flags |= RING_BUFFER_BLOCK; // блокируем буфер

      if((addressStart + count) > length){  // разделённый пакет
				// при разделённом пакете данных отправляем их в два приёма
				// оставшиеся данные с адреса 0 отправятся при следующем вызове процедуры
				uint16_t countByte = length - addressStart;
        /** проверка - режим TX отключен */
				if ((HAL_UART_GetState(&uart) & HAL_UART_STATE_BUSY_TX) != HAL_UART_STATE_BUSY_TX ){
          /** проверка свободен ли DMA */
          if(dma.State == HAL_DMA_STATE_READY){
					HAL_UART_Transmit_DMA(&uart, memBuffer + addressStart, countByte);
					addressStart = 0;
					count -= countByte;
          // при разделённом пакете выставляем флаг RING_BUFFER_TX_SPLIT
          flags |= RING_BUFFER_TX_SPLIT;
				}}
			}
			else{  // слитный пакет
        /** проверка - режим TX отключен */
				if ((HAL_UART_GetState(&uart) & HAL_UART_STATE_BUSY_TX) != HAL_UART_STATE_BUSY_TX ){
          /** проверка свободен ли DMA */
          if(dma.State == HAL_DMA_STATE_READY){
						HAL_UART_Transmit_DMA(&uart, memBuffer + addressStart, count);
						addressStart = addressEnd;
						count = 0;
            // при слитном пакете сбрасываем флаг RING_BUFFER_TX_SPLIT
            flags &= ~RING_BUFFER_TX_SPLIT;
				}}
			}
      flags &= ~RING_BUFFER_BLOCK; // РАЗблокируем буфер
  }}
}


/**
 * Отправка накопленных в кольцевом буфере данных через USB виртуальный COM порт
 */
void RingBuffer::txUsbComPort(){

  if((flags & RING_BUFFER_BLOCK) == 0){ if (count > 0 ) {
    flags |= RING_BUFFER_BLOCK; // блокируем буфер

    if((addressStart + count) > length) {  // разделённый пакет
      uint16_t countByte = length - addressStart;
      uint8_t result = CDC_Transmit_FS((unsigned char*)(memBuffer + addressStart), countByte);
      if(result == USBD_OK){
        addressStart = 0;
        count -= countByte;
        // при разделённом пакете выставляем флаг RING_BUFFER_TX_SPLIT
        flags |= RING_BUFFER_TX_SPLIT;
      }
    }
    else {  // слитный пакет

      uint8_t result = CDC_Transmit_FS((unsigned char*)(memBuffer + addressStart), count);
      if(result == USBD_OK){
        addressStart = addressEnd;
        count = 0;
        // при слитном пакете сбрасываем флаг RING_BUFFER_TX_SPLIT
        flags &= ~RING_BUFFER_TX_SPLIT;
      }

    }

    flags &= ~RING_BUFFER_BLOCK; // РАЗблокируем буфер
  }}

}


/**
 * @brief Статус(флаги) состояния буфера лога
 * 
 * @return uint8_t 
 */
uint8_t RingBuffer::getStatus(void){
  return flags;
}

/**
 * @brief Блокировка буфера
 */
void RingBuffer::bufferBlock(void){
  flags |= RING_BUFFER_BLOCK;
}

/**
 * @brief Разблокировка буфера
 */
void RingBuffer::bufferUnBlock(void){
  flags &= ~RING_BUFFER_BLOCK; // РАЗблокируем буфер
}


/** Размещение сообщения в лог в основном цикле программы
 * @param str      буфер сообщения
 * @param lenStr   длина сообщения
 */
void Logger::messageLog(uint8_t *str, uint16_t lenStr){

  while((log.getStatus() & RING_BUFFER_BLOCK) != 0 ){} // ждём разблокировки
  log.bufferBlock();                    // блокируем буфер лога

  log.messageToRingBuffer(str, lenStr); // сообщение в буфер
  log.bufferUnBlock();                  // РАЗблокируем буфер
}


/** Размещение сообщения в лог в прерывании
 * @param str      буфер сообщения
 * @param lenStr   длина сообщения
 */
void Logger::messageLogIT(uint8_t *str, uint16_t lenStr){
  logIT.bufferBlock();                    // блокируем буфер лога
  logIT.messageToRingBuffer(str, lenStr); // сообщение в буфер
  logIT.bufferUnBlock();                  // РАЗблокируем буфер
}

/**
 * Отправка накопленных в кольцевом буфере сообщений TX UART через DMA
 * Эта функция вызывается из прерывания
 *
 * В этой функции проверяется флаг RING_BUFFER_TX_SPLIT(разделённого сообщения при отправке в UART_TX DMA)
 * если флаг сброшен, то буфер logIT копируется в буфер log
 */
void Logger::logToDmaUart_IT() {
  if((log.getStatus() & RING_BUFFER_TX_SPLIT) == 0){ // сообщение не разделено при отправке
    logITDataToLog();
  }

  log.txDmaUart(UART_LOGGER, DMA_TX_LOGGER);
}

/**
 * Перенос из буфера logIT накопленных данных в буфер log
 * со сбросом исходного logIT буфера
 * @param target
 */
void Logger::logITDataToLog(){
  if (logIT.getCount() > 0 ) { // в исходном буфере есть накопленные данные
    if ((logIT.getStatus() & RING_BUFFER_BLOCK) == 0) {  // исходный буфер не блокирован
      if ((log.getStatus() & RING_BUFFER_BLOCK) == 0) {  // целевой буфер не блокирован
        logIT.bufferBlock(); // блокируем исходный буфер
        log.bufferBlock(); // блокируем целевой буфер

        if((logIT.addressStart + logIT.count) > logIT.length){  // разделённый пакет
          log.messageToRingBuffer(logIT.memBuffer + logIT.addressStart, logIT.length - logIT.addressStart);
          log.messageToRingBuffer(logIT.memBuffer, logIT.count - (logIT.length - logIT.addressStart));
        }
        else{ // слитный пакет - перенос данных

          log.messageToRingBuffer(logIT.memBuffer + logIT.addressStart, logIT.count);
        }

        logIT.resetBuffer(); // сброс буфера

        logIT.bufferUnBlock(); // РАЗблокируем исходный буфер
        log.bufferUnBlock(); // РАЗблокируем целевой буфер
      }
    }
  }
}


/**
 * Отправка буфера лога в назначенный порт
 * В этой функции проверяется флаг RING_BUFFER_TX_SPLIT(разделённого сообщения при отправке в UART_TX DMA)
 * если флаг сброшен, то буфер logIT копируется в буфер log
 */
void Logger::transmitLogBuffer(){
#ifdef UART_LOG_TRANSMIT
  /** Отправка буфера лога через UART */
  if((log.getStatus() & RING_BUFFER_TX_SPLIT) == 0){ // сообщение не разделено при отправке
    logITDataToLog();
  }

  log.txDmaUart(UART_LOGGER, DMA_TX_LOGGER);
#endif

#ifdef USB_COM_LOG_TRANSMIT
  /** Отправка буфера лога через USB виртуальный COM порт */
  if((log.getStatus() & RING_BUFFER_TX_SPLIT) == 0){ // сообщение не разделено при отправке
    logITDataToLog();
  }
  log.txUsbComPort();
#endif

}
