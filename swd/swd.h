#ifndef _SWD_H_
#define _SWD_H_

#include <stdint.h>
#include "main.h"
// Заполните правильно. Используем LL везде.
#define SWD_IO			swd_io_GPIO_Port, 		swd_io_Pin
#define SWD_CLK			swd_clk_GPIO_Port, 		swd_clk_Pin
#define SWD_NRST		swd_reset_GPIO_Port, 	swd_reset_Pin

#define SWD_PIN_HIGH(pin) 		LL_GPIO_SetOutputPin(pin)
#define SWD_PIN_LOW(pin)     	LL_GPIO_ResetOutputPin(pin)
#define SWD_IO_GET()     		LL_GPIO_IsInputPinSet(SWD_IO)
#define SWD_IO_INPUT() 			LL_GPIO_SetPinMode(SWD_IO, LL_GPIO_MODE_INPUT)
#define SWD_IO_OUTPUT() 		LL_GPIO_SetPinMode(SWD_IO, LL_GPIO_MODE_OUTPUT)

// ============================================================================
// Типы данных и коды ошибок
// ============================================================================

typedef enum
{
    SWD_ERR_OK          = 0,// Успех
    SWD_ERR_FAULT,     		// Ошибка протокола (FAULT)
    SWD_ERR_PARITY,      	// Ошибка четности
	SWD_ERR_WAIT,      		// Не ошибка, просто жди
    SWD_ERR_TRY_COUNT,   	// Превышено количество попыток (Timeout)
} swd_err_e;

// ============================================================================
// Прототипы функций
// ============================================================================

/**
 * @brief Инициализация интерфейса SWD
 * Настраивает GPIO, тактирование, выполняет последовательность переключения JTAG->SWD
 * и настраивает Access Port (MEM-AP) для работы с памятью.
 */
swd_err_e swd_init(uint32_t *idcode);

/**
 * @brief Установка адреса для чтения/записи памяти
 * Записывает адрес в регистр TAR. Следующая операция чтения вернет "мусор",
 * а вторая — реальные данные по этому адресу (из-за конвейера).
 */
swd_err_e swd_set_word_addr(uint32_t addr);

/**
 * @brief Чтение одного слова (32 бит) из памяти
 * Учитывает конвейерную задержку. Если адрес был установлен ранее,
 * первое чтение сбрасывает старые данные.
 */
swd_err_e swd_read_word(uint32_t *word);

/**
 * @brief Запись одного слова (32 бит) в память
 */
swd_err_e swd_write_word(uint32_t word);

/**
 * @brief Чтение буфера памяти
 * @param addr Начальный адрес
 * @param data Указатель на буфер для записи данных
 * @param size_w Размер в словах (количество uint32_t)
 */
swd_err_e swd_read_buff(uint32_t addr, uint32_t *data, uint32_t size_w);

/**
 * @brief Запись буфера в память
 * @param addr Начальный адрес
 * @param data Указатель на буфер с данными
 * @param size_w Размер в словах (количество uint32_t)
 */
swd_err_e swd_write_buff(uint32_t addr, uint32_t *data, uint32_t size_w);
// ============================================================================
// Управление ядром (CPU Debug Control)
// ============================================================================

/**
 * @brief Чтение регистра ядра (R0-R15, PC, SP, xPSR и др.)
 * @param reg_val Указатель для сохранения значения
 * @param reg_num Номер регистра (0-15 для R0-R15, 16 для xPSR, 17 для MSP, 18 для PSP и т.д.)
 */
swd_err_e swd_read_cpu_reg(uint32_t *reg_val, uint32_t reg_num);

/**
 * @brief Запись в регистр ядра
 * @param reg_val Значение для записи
 * @param reg_num Номер регистра
 */
swd_err_e swd_write_cpu_reg(uint32_t reg_val, uint32_t reg_num);

/**
 * @brief Управление состоянием ядра (Остановить/Запустить)
 * @param state 0 - Остановить (Halt), 1 - Запустить (Run)
 */
swd_err_e swd_set_cpu_state(uint8_t state);

/**
 * @brief Аппаратный сброс устройства через линию NRST
 */
swd_err_e swd_reset_device(void);





#endif // _SWD_H_
