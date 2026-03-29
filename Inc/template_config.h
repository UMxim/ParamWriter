#ifndef TEMPLATE_CONFIG_H_
#define TEMPLATE_CONFIG_H_

/*
 * 1. Конфигурация шаблона. Вынесена так как у всех проектов настройка своя, чтобы не тянуть в репозиторий
 * 2. Если FLASH то не забыть отрезать кусок из ld файла
 * 3. Не забыть подключить template.h в main.h
 * 4. И в прерывании таймера systick добавить systime_ticks += US_TO_TICKS(1000);
 * 5. Ну и заполнить параметры :)
 * 6. В ld или CubeMX выставить стек - пара КБ (в записи флеша выделю сектор - 1КБ - не хочу в куче). Ну и кучу тоже пару КБ или по усмотрению.
 */

// ============================================================================
// СПИСОК ПАРАМЕТРОВ
// ID — без пробелов (используется в enum и макросах)
// Строка — может содержать пробелы, спецсимволы и т.д.
// ============================================================================
#define PARAM_LIST \
    X(SAVE_TIMER,  24*3600, "Period save to flash"	) 	\
    X(VOLT_OUT,    1200,    "Output Voltage"		)   \
    X(CURR_MOTOR,  340,     "Motor Current"			)   \
    X(FREQ_PWM,    50,      "PWM Frequency"			)   \
    X(PWR_LIMIT,   1000,    "Power Limit"			)

// ============================================================================
// СПИСОК СТАТУСОВ
// ============================================================================
#define STAT_LIST \
    X(UPTIME_SEC,  0,       "Uptime (seconds)"		)   \
    X(ERR_COUNT,   0,       "Error Count"			)   \
    X(HEARTBEAT,   0,       "Heartbeat"				)


// ============================================================================
// Выбери одно
#ifdef STM32L011xx
 #define TEMPLATE_PARAM_EEPROM
#elif defined(STM32F103xB)
 #define TEMPLATE_PARAM_FLASH
#endif

#ifdef TEMPLATE_PARAM_EEPROM

 #define PARAM_EEPROM_BEGIN		0x08080000U
 #define PARAM_EEPROM_SIZE		512U

#elif defined (TEMPLATE_PARAM_FLASH)

 #define PARAM_FLASH_SIZE 		0x400U  // 1K
 #define PARAM_FLASH_BEGIN		(0x08000000U + 0x10000U - PARAM_FLASH_SIZE)

#endif

// ============================================================================

#define TEMPLATE_SYSTICK_FREQ	72000000


#endif // TEMPLATE_CONFIG_H_
