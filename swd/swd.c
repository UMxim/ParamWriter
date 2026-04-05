
#include "swd.h"
#include "main.h"





#include "swd.h"
#include <stdint.h>

// ============================================================================
// Локальные определения и константы
// ============================================================================

// Адреса регистров отладки ядра (CoreSight)
#define NVIC_DHCSR_ADDR     0xE000EDF0
#define NVIC_DCRSR_ADDR     0xE000EDF4
#define NVIC_DCRDR_ADDR     0xE000EDF8
#define NVIC_AIRCR_ADDR     0xE000ED0C

// Биты DHCSR
#define DHCSR_C_DEBUGEN     (1U << 0)
#define DHCSR_C_HALT        (1U << 1)
#define DHCSR_C_STEP        (1U << 2)
#define DHCSR_S_REGRDY      (1U << 16)
#define DHCSR_S_HALT        (1U << 17)
#define DHCSR_KEY           0xA05F0000

// Биты AIRCR для сброса
#define AIRCR_VECTKEY       (0x5FA << 16)
#define AIRCR_SYSRESETREQ   (1U << 2)

// Макрос для безопасной записи в регистры с маской
#define SET_REG_MASK(reg, mask, val) ( ((reg) & ~(mask)) | ((val) & (mask) ) )

// Формирование пакета запроса
// code содержит биты: [0:APnDP], [1:A2], [2:A3] (смещенные согласно enum)
// Реальная структура байта: Start(1) | APnDP | RnW | A2 | A3 | Parity | Stop(0) | Park(1)
//#define _PARITY(RnW, code) (((RnW) + ((code >> 1) & 1) + ((code >> 3) & 1) + ((code >> 4) & 1)) & 1)
//#define _CODE(RnW, code) ( (1U << 0) | (code) | ((RnW) << 2) | (_PARITY(RnW, code) << 5) | (0b01U << 6) )

typedef enum
{
    // Debug Port (APnDP = 0)
    // Биты задаются так, чтобы попасть в позиции байта запроса
	//                0b10p32rA1
	DP_ABORT_W		= 0b10000001,
	DP_IDCODE_R		= 0b10100101,
	DP_CTRL_STAT_W  = 0b10101001,
	DP_CTRL_STAT_R  = 0b10001101,
	DP_SELECT_W		= 0b10110001,
	DP_RESEND_R		= 0b10010101,
	DP_RDBUFF_R		= 0b10111101,
	AP_CSW_R		= 0b10000111,
	AP_CSW_W		= 0b10100011,
	AP_TAR_R		= 0b10101111,
	AP_TAR_W		= 0b10001011,
	AP_DRW_R		= 0b10011111,
	AP_DRW_W		= 0b10111011,
	AP_IDR_R		= 0b10011111,
	AP_BASE_R		= 0b10110111,
} port_code_e;

// Глобальные состояния
static uint8_t isTARset = 0;
static uint8_t isGPIOinit = 0;


// Задержка для стабильной работы на 72 МГц
static inline void swd_delay(void)
{
    //for (volatile int i = 0; i < 8; i++) __asm volatile ("nop");
	timer_delay_ticks(72);
}



static uint8_t parity(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v &= 0xf;
    return (0x6996 >> v) & 1;
}

static void swd_write_bit(uint8_t bit)
{
    if (bit)
    {
    	SWD_PIN_HIGH(SWD_IO);
    }
    else
    {
    	SWD_PIN_LOW(SWD_IO);
    }
    swd_delay();
    SWD_PIN_LOW(SWD_CLK);
    swd_delay();
    SWD_PIN_HIGH(SWD_CLK);
}

static uint8_t swd_read_bit(void)
{
    swd_delay();
    SWD_PIN_LOW(SWD_CLK);
    swd_delay();
    uint8_t bit = !!SWD_IO_GET();
    SWD_PIN_HIGH(SWD_CLK);
    return bit;
}

static void swd_idle(void)
{
	// 2 такта простоя
	swd_write_bit(0);
	swd_write_bit(0);
}
// ============================================================================
// Низкоуровневые функции протокола
// ============================================================================
static swd_err_e swd_read_reg_single(port_code_e code, uint32_t *data)
{
	swd_err_e res;
	//swd_write_bit(1);
	for (int i = 0; i < 8; i++)
	{
		swd_write_bit(code & 1);
		code >>= 1;
	}
	// Turnaround
	SWD_IO_INPUT();
	swd_read_bit();
	// Чтение ACK
	uint8_t ack = 0;
	for (int i = 0; i < 3; i++)
	{
		ack |= (swd_read_bit() << i);
	}
	if (ack == 1) // OK
	{
		uint32_t val = 0;
		uint8_t p_calc = 0;
		for (int i = 0; i < 32; i++)
		{
			uint8_t b = swd_read_bit();
			val |= ((uint32_t)b << i);
			p_calc ^= b;
		}
		uint8_t p_rx = swd_read_bit();
		if (p_calc == p_rx)
		{
			*data = val;
			res = SWD_ERR_OK;
		}
		else
		{
			res = SWD_ERR_PARITY;
		}
	}
	else if (ack == 2) // WAIT
	{
		res = SWD_ERR_WAIT;
	}
	else // FAULT
	{
		res = SWD_ERR_FAULT;
	}

	swd_read_bit(); // Turnaround
	SWD_IO_OUTPUT();
	//swd_idle();
	//SWD_PIN_HIGH(SWD_IO);
	return res;

}

static swd_err_e swd_read_reg(port_code_e code, uint32_t *data)
{
	swd_err_e res;
	ASSERT_DEBUG((code & 0b100) == 0);
    for (int retry = 0; retry < 50; retry++)
    {
      res = swd_read_reg_single(code, data);
      if (res == SWD_ERR_WAIT)
      {
    	  timer_delay_us(10);
    	  continue;
      }
      if (res == SWD_ERR_PARITY)
      {
    	  continue;
      }
      return res; // OK или FAULT
    }
    return SWD_ERR_TRY_COUNT;
}

static swd_err_e swd_write_reg_single(port_code_e code, uint32_t data)
{
	swd_err_e res;
	for (int i = 0; i < 8; i++)
	{
		swd_write_bit(code & 1);
		code >>= 1;
	}
	SWD_IO_INPUT();
	swd_read_bit(); // Turnaround

	uint8_t ack = 0;
	for (int i = 0; i < 3; i++)
	{
		ack |= (swd_read_bit() << i);
	}
	swd_read_bit(); // Turnaround перед данными
	SWD_IO_OUTPUT();
	if (ack == 1) // OK
	{
		uint8_t p = parity(data);
		for (int i = 0; i < 32; i++)
		{
			swd_write_bit(data & 1);
			data >>= 1;
		}
		swd_write_bit(p);
		res = SWD_ERR_OK;
	}
	else if (ack == 2) // WAIT
	{
		res = SWD_ERR_WAIT;
	}
	else
	{
		res = SWD_ERR_FAULT;
	}
	//swd_idle();
	//SWD_PIN_HIGH(SWD_IO);
	return res;
}

static swd_err_e swd_write_reg(port_code_e code, uint32_t data)
{
	swd_err_e res;
	ASSERT_DEBUG((code & 0b100) == 1);
    for (int retry = 0; retry < 50; retry++)
    {
    	res = swd_write_reg_single(code, data);
        if (res == SWD_ERR_WAIT)
        {
        	timer_delay_us(10);
        	continue;
        }
        if (res == SWD_ERR_PARITY)
        {
        	continue;
        }
        return res; // OK или FAULT
    }
    return SWD_ERR_TRY_COUNT;
}

// ============================================================================
// Инициализация
// ============================================================================

static void swd_init_gpio(void)
{
	LL_GPIO_SetPinPull(SWD_IO, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(SWD_IO, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinOutputType(SWD_IO, LL_GPIO_OUTPUT_PUSHPULL);

    // Включаем тактирование ВСЕХ используемых портов (A, B, C)
/*    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

    // NRST (PC0) - Open-drain output (0x5)
    SWD_NRST_PORT->CRL &= ~(0xFU << (SWD_NRST_PIN * 4));
    SWD_NRST_PORT->CRL |=  (0x5U << (SWD_NRST_PIN * 4));
    rst_high();

    // SWCLK (PB2) - Push-pull 50MHz (0x3)
    SWD_CLK_PORT->CRL &= ~(0xFU << (SWD_CLK_PIN * 4));
    SWD_CLK_PORT->CRL |=  (0x3U << (SWD_CLK_PIN * 4));

    // SWDIO (PA1) - Output
    swdio_output();
    swdio_high();
    swclk_high();*/
}

swd_err_e swd_init(uint32_t *idcode)
{
	uint32_t idr, base;
    if (!isGPIOinit)
    {
        swd_init_gpio();
        isGPIOinit = 1;
    }

    // Сброс линии (>50 тактов HIGH)
    SWD_IO_OUTPUT();
    SWD_PIN_HIGH(SWD_IO);
    SWD_PIN_HIGH(SWD_CLK);
    timer_delay_us(100);
    for (int i = 0; i < 55; i++)
    {
    	swd_write_bit(1);
    }
    // Команда переключения JTAG -> SWD: 0xE79E
    uint16_t switch_cmd = 0xE79E;
    for (int i = 0; i < 16; i++)
    {
        swd_write_bit(switch_cmd & 1);
        switch_cmd >>= 1;
    }
    for (int i = 0; i < 55; i++)
    {
    	swd_write_bit(1);
    }
    swd_idle();
    timer_delay_us(100);
    // Настройка MEM-AP
    swd_err_e res;
    uint32_t data = 0;
    res = swd_read_reg(DP_IDCODE_R, idcode);
    if (res != SWD_ERR_OK) return res;
    timer_delay_us(10);
    // reset errors
    res = swd_write_reg(DP_ABORT_W, 0x1E);
    if (res != SWD_ERR_OK) return res;

    res = swd_read_reg(DP_CTRL_STAT_R, &data);
    if (res != SWD_ERR_OK) return res;
    timer_delay_us(10);

    res = swd_write_reg(DP_CTRL_STAT_W, 0x50000000);
    if (res != SWD_ERR_OK) return res;
    timer_delay_us(10);
    do
    {
        res = swd_read_reg(DP_CTRL_STAT_R, &data);
        if (res != SWD_ERR_OK) return res;
    }
    while ((data & 0xF0000000) != 0xF0000000);

    // Выбрать AP 0 (через DP SELECT)
    res = swd_write_reg(DP_SELECT_W, 0xF0);
    if (res != SWD_ERR_OK) return res;

    res = swd_read_reg(AP_IDR_R, &idr);
    if (res != SWD_ERR_OK) return res;

    res = swd_read_reg(AP_BASE_R, &base);
    if (res != SWD_ERR_OK) return res;

    res = swd_read_reg(DP_RDBUFF_R, &base);
    if (res != SWD_ERR_OK) return res;

    res = swd_write_reg(DP_SELECT_W, 0x0);
    if (res != SWD_ERR_OK) return res;

    // Настроить CSW (Word size, Auto-increment enabled)
    // 0x23000023: MasterNumber=0x23, Size=Word(2), AddrInc=Single(1)
    res = swd_write_reg(AP_CSW_W, 0x23000012);
    if (res != SWD_ERR_OK) return res;


    res = swd_write_reg(AP_TAR_W, 0xE000EDF0); // Адрес DHCSR
    res = swd_write_reg(AP_DRW_W, 0xA05F0003); // Команда Halt

    isTARset = 0;
    return SWD_ERR_OK;
}

// ============================================================================
// Работа с памятью (с учетом конвейера)
// ============================================================================

swd_err_e swd_set_word_addr(uint32_t addr)
{
    isTARset = 0;
    swd_err_e res = swd_write_reg(AP_TAR_W, addr);
    if (res == SWD_ERR_OK) isTARset = 1;
    return res;
}

swd_err_e swd_read_word(uint32_t *word)
{
    swd_err_e res;
    if(isTARset)
    {
        // Первое чтение после установки TAR возвращает мусор (предыдущее значение DRW)
        uint32_t dummy;
        res = swd_read_reg(AP_DRW_R, &dummy);
        if (res != SWD_ERR_OK) return res;
        isTARset = 0;
    }
    // Второе чтение дает реальные данные по адресу TAR
    res = swd_read_reg(DP_RDBUFF_R, word);
    return res;
}

swd_err_e swd_write_word(uint32_t word)
{
    swd_err_e res; // Исправлено: добавлено объявление
    res = swd_write_reg(AP_DRW_W, word);
    return res;
}

swd_err_e swd_read_buff(uint32_t addr, uint32_t *data, uint32_t size_w)
{
    swd_err_e res;
    res = swd_set_word_addr(addr); // Исправлено: добавлена точка с запятой
    if (res != SWD_ERR_OK) return res;

    while(size_w--)
    {
        res = swd_read_word(data++);
        if (res != SWD_ERR_OK) return res;
    }
    return SWD_ERR_OK;
}

swd_err_e swd_write_buff(uint32_t addr, uint32_t *data, uint32_t size_w)
{
    swd_err_e res;
    res = swd_set_word_addr(addr); // Исправлено: добавлена точка с запятой
    if (res != SWD_ERR_OK) return res;

    while(size_w--)
    {
        res = swd_write_word(*data++);
        if (res != SWD_ERR_OK) return res;
    }
    return SWD_ERR_OK;
}

// ============================================================================
// Управление ядром (CPU Control) - РЕАЛИЗАЦИЯ
// ============================================================================

// Вспомогательная функция для прямой записи в память (для регистров отладки)
static swd_err_e mem_write_word(uint32_t addr, uint32_t val)
{
    // Для записи в системные регистры нужно временно отключить авто-инкремент или аккуратно управлять TAR
    // Простой способ: установить TAR, записать, но это сломает поток если был активен.
    // Надежный способ для единичных записей:
    uint32_t old_csw;
    // Читаем текущий CSW, чтобы сохранить настройки (упрощенно считаем что можем перезаписать TAR)
    // Но проще сделать полную последовательность, так как это редкие операции

    // Сохраним состояние TAR флага локально, но глобальный isTARset может рассинхронизироваться
    // Для безопасности в рамках этой простой реализации просто делаем прямую запись через AP_DRW
    // предварительно установив адрес. Это прервет любой блочный режим.

    swd_write_reg(AP_TAR_W, addr);
    // Сбрасываем глобальный флаг, так как мы его испортили
    isTARset = 0;

    // Первое чтение/запись после TAR - мусорное для чтения, но для записи DRW данные уходят по адресу TAR сразу
    // Однако из-за конвейера, запись в DRW идет по СТАРОМУ адресу TAR, если не сделать даммми.
    // Для записи: Запись в TAR -> Запись в DRW (данные уйдут по новому адресу? Нет, с задержкой).
    // Правильная последовательность записи с конвейером:
    // 1. Записать TAR
    // 2. Записать DRW (данные уйдут по ПРЕДЫДУЩЕМУ адресу TAR!) -> Это ошибка.
    // Поэтому для единичной записи нужно:
    // Записать TAR -> Сделать фиктивную запись/чтение -> Записать DRW.

    // Упрощенный надежный вариант для отладки (медленнее, но верно):
    // Отключаем автоинкремент временно? Слишком сложно.
    // Просто сделаем "Dummy write" в любой адрес или перечитаем CSW?

    // Самый простой рабочий вариант для битбанга без сложного планирования:
    // Записать TAR.
    // Прочитать любой регистр (например IDCODE), чтобы протолкнуть конвейер (flush).
    // Затем записать DRW.

    uint32_t dummy;
    swd_read_reg(DP_IDCODE_R, &dummy); // Flush pipeline

    return swd_write_reg(AP_DRW_W, val);
}

static swd_err_e mem_read_word(uint32_t addr, uint32_t *val)
{
    swd_write_reg(AP_TAR_W, addr);
    isTARset = 0; // Сброс флага

    uint32_t dummy;
    swd_read_reg(AP_DRW_R, &dummy); // Dummy read (старые данные)

    return swd_read_reg(AP_DRW_R, val); // Реальные данные
}

swd_err_e swd_set_cpu_state(uint8_t state)
{
    uint32_t dhcsr_val;

    if (state == 0) // STOP (Halt)
    {
        dhcsr_val = DHCSR_KEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT;
        if (mem_write_word(NVIC_DHCSR_ADDR, dhcsr_val) != SWD_ERR_OK) return SWD_ERR_FAULT;

        // Ждем пока ядро остановится
        for(int i=0; i<1000; i++) {
            uint32_t status;
            if (mem_read_word(NVIC_DHCSR_ADDR, &status) == SWD_ERR_OK) {
                if (status & DHCSR_S_HALT) return SWD_ERR_OK;
            }
        }
        return SWD_ERR_TRY_COUNT; // Не остановилось
    }
    else // RUN
    {
        dhcsr_val = DHCSR_KEY | DHCSR_C_DEBUGEN; // C_HALT = 0
        if (mem_write_word(NVIC_DHCSR_ADDR, dhcsr_val) != SWD_ERR_OK) return SWD_ERR_FAULT;
        return SWD_ERR_OK;
    }
}

swd_err_e swd_read_cpu_reg(uint32_t *reg_val, uint32_t reg_num)
{
    // 1. Запрос на чтение (DCRSR)
    // Bit 16 = 0 (Read), Bits [4:0] = номер регистра
    if (mem_write_word(NVIC_DCRSR_ADDR, reg_num & 0x1F) != SWD_ERR_OK)
        return SWD_ERR_FAULT;

    // 2. Ждем S_REGRDY
    int timeout = 1000;
    uint32_t dhcsr;
    do {
        if (mem_read_word(NVIC_DHCSR_ADDR, &dhcsr) != SWD_ERR_OK) return SWD_ERR_FAULT;
        if (dhcsr & DHCSR_S_REGRDY) break;
        timeout--;
    } while (timeout > 0);

    if (timeout <= 0) return SWD_ERR_TRY_COUNT;

    // 3. Читаем данные (DCRDR)
    return mem_read_word(NVIC_DCRDR_ADDR, reg_val);
}

swd_err_e swd_write_cpu_reg(uint32_t reg_val, uint32_t reg_num)
{
    // 1. Пишем данные в буфер (DCRDR)
    if (mem_write_word(NVIC_DCRDR_ADDR, reg_val) != SWD_ERR_OK)
        return SWD_ERR_FAULT;

    // 2. Запрос на запись (DCRSR)
    // Bit 16 = 1 (Write), Bits [4:0] = номер регистра
    uint32_t req = (reg_num & 0x1F) | (1U << 16);
    if (mem_write_word(NVIC_DCRSR_ADDR, req) != SWD_ERR_OK)
        return SWD_ERR_FAULT;

    // 3. Ждем S_REGRDY
    int timeout = 1000;
    uint32_t dhcsr;
    do {
        if (mem_read_word(NVIC_DHCSR_ADDR, &dhcsr) != SWD_ERR_OK) return SWD_ERR_FAULT;
        if (dhcsr & DHCSR_S_REGRDY) break;
        timeout--;
    } while (timeout > 0);

    if (timeout <= 0) return SWD_ERR_TRY_COUNT;

    return SWD_ERR_OK;
}

swd_err_e swd_reset_device(void)
{
    // 1. Pull NRST low
	SWD_PIN_LOW(SWD_NRST);

    // Задержка ~10ms
    for (volatile int i = 0; i < 100000; i++) __asm volatile("nop");

    // 2. Release NRST
    SWD_PIN_HIGH(SWD_NRST);

    // 3. Небольшая пауза для старта чипа
    for (volatile int i = 0; i < 10000; i++) __asm volatile("nop");

    // Опционально: можно сразу попробовать остановить ядро, чтобы поймать старт
    // swd_set_cpu_state(0);

    return SWD_ERR_OK;
}
