// BUILD MARKER: UNOQ_FGEN_SCOPE_V18_6_IRQ_ADC_MULTI_RATE_DMA_VALIDATED
#include <Arduino_RouterBridge.h>
#include <math.h>

/*
 * v19.0 UNO Q fixed-rate low-latency/stable-autoset backend
 *
 * Arduino UNO Q sketches are Zephyr LLEXT modules.  The LLEXT EDK does not
 * necessarily expose ST's umbrella CMSIS header <stm32u5xx.h>, which is why
 * v17.6 compiled into software_fallback even on the real UNO Q.
 *
 * v19.0 therefore uses:
 *   - Zephyr dynamic interrupt API (exported by ArduinoCore-zephyr on UNO Q)
 *   - the STM32U585 non-secure peripheral addresses from Zephyr's STM32U5 DTS
 *   - the RCC register offsets / TIM6 bit defined by the STM32U5 register map
 *
 * This avoids a build-time dependency on ST CMSIS typedefs/macros.
 */
#if defined(ARDUINO_UNO_Q)
  #define UNOQ_RAW_MMIO 1
#else
  #define UNOQ_RAW_MMIO 0
#endif

#if defined(ARDUINO_UNO_Q) && defined(__has_include)
  #if __has_include(<zephyr/irq.h>)
    #include <zephyr/irq.h>
    #define UNOQ_HAS_ZEPHYR_IRQ_HEADER 1
  #else
    #define UNOQ_HAS_ZEPHYR_IRQ_HEADER 0
  #endif
#else
  #define UNOQ_HAS_ZEPHYR_IRQ_HEADER 0
#endif

#if defined(CONFIG_DYNAMIC_INTERRUPTS) && CONFIG_DYNAMIC_INTERRUPTS
  #define UNOQ_HAS_DYNAMIC_IRQ 1
#else
  #define UNOQ_HAS_DYNAMIC_IRQ 0
#endif

#if UNOQ_RAW_MMIO && UNOQ_HAS_ZEPHYR_IRQ_HEADER && UNOQ_HAS_DYNAMIC_IRQ
  #define UNOQ_DAC_HW_TIMER 1
  #define UNOQ_ADC_HW_TRIGGER 1
#else
  #define UNOQ_DAC_HW_TIMER 0
  #define UNOQ_ADC_HW_TRIGGER 0
#endif

// Direct DAC register access is safe on UNO Q after analogWrite(DAC0, 0)
// performs the one-time Zephyr DAC channel/pinctrl initialization.
#define UNOQ_FAST_DAC_REGISTER UNOQ_RAW_MMIO

#if UNOQ_RAW_MMIO
  // Zephyr STM32U5 device tree / STM32U585 non-secure memory map.
  static constexpr uint32_t UNOQ_TIM6_BASE  = 0x40001000UL;
  static constexpr uint32_t UNOQ_TIM15_BASE = 0x40014000UL;
  static constexpr uint32_t UNOQ_RCC_BASE   = 0x46020C00UL;
  static constexpr uint32_t UNOQ_ADC1_BASE  = 0x42028000UL;
  static constexpr uint32_t UNOQ_DAC1_BASE  = 0x46021800UL;
  static constexpr uint32_t UNOQ_GPDMA1_BASE = 0x40020000UL;
  // GPDMA1 channel 0: base + 0x50. Channels 0..11 are the general
  // GPDMA channels used by ST's conventional peripheral-to-memory examples.
  static constexpr uint32_t UNOQ_GPDMA1_CH_BASE =
      UNOQ_GPDMA1_BASE + 0x0050UL;

  // Common timer register offsets.
  static constexpr uint32_t UNOQ_TIM_CR1   = 0x00UL;
  static constexpr uint32_t UNOQ_TIM_CR2   = 0x04UL;
  static constexpr uint32_t UNOQ_TIM_DIER  = 0x0CUL;
  static constexpr uint32_t UNOQ_TIM_SR    = 0x10UL;
  static constexpr uint32_t UNOQ_TIM_EGR   = 0x14UL;
  static constexpr uint32_t UNOQ_TIM_CNT   = 0x24UL;
  static constexpr uint32_t UNOQ_TIM_PSC   = 0x28UL;
  static constexpr uint32_t UNOQ_TIM_ARR   = 0x2CUL;

  // RCC STM32U5 offsets.
  static constexpr uint32_t UNOQ_RCC_AHB1RSTR  = 0x60UL;
  static constexpr uint32_t UNOQ_RCC_APB1RSTR1 = 0x74UL;
  static constexpr uint32_t UNOQ_RCC_APB2RSTR  = 0x7CUL;
  static constexpr uint32_t UNOQ_RCC_AHB1ENR   = 0x88UL;
  static constexpr uint32_t UNOQ_RCC_APB1ENR1  = 0x9CUL;
  static constexpr uint32_t UNOQ_RCC_APB2ENR   = 0xA4UL;
  static constexpr uint32_t UNOQ_RCC_GPDMA1_BIT = (1UL << 0);
  static constexpr uint32_t UNOQ_RCC_TIM6_BIT  = (1UL << 4);
  static constexpr uint32_t UNOQ_RCC_TIM15_BIT = (1UL << 16);

  // ADC1 register offsets used by the 14-bit A1 acquisition path.
  static constexpr uint32_t UNOQ_ADC_ISR    = 0x00UL;
  static constexpr uint32_t UNOQ_ADC_IER    = 0x04UL;
  static constexpr uint32_t UNOQ_ADC_CR     = 0x08UL;
  static constexpr uint32_t UNOQ_ADC_CFGR1  = 0x0CUL;
  static constexpr uint32_t UNOQ_ADC_PCSEL  = 0x1CUL;
  static constexpr uint32_t UNOQ_ADC_SQR1   = 0x30UL;
  static constexpr uint32_t UNOQ_ADC_DR     = 0x40UL;

  // DAC1 channel-1 12-bit right-aligned holding register.
  static constexpr uint32_t UNOQ_DAC_DHR12R1 = 0x08UL;

  // STM32U5 GPDMA channel register offsets.
  static constexpr uint32_t UNOQ_GPDMA_CLBAR = 0x00UL;
  static constexpr uint32_t UNOQ_GPDMA_CFCR  = 0x0CUL;
  static constexpr uint32_t UNOQ_GPDMA_CSR   = 0x10UL;
  static constexpr uint32_t UNOQ_GPDMA_CCR   = 0x14UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR1  = 0x40UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR2  = 0x44UL;
  static constexpr uint32_t UNOQ_GPDMA_CBR1  = 0x48UL;
  static constexpr uint32_t UNOQ_GPDMA_CSAR  = 0x4CUL;
  static constexpr uint32_t UNOQ_GPDMA_CDAR  = 0x50UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR3  = 0x54UL;
  static constexpr uint32_t UNOQ_GPDMA_CBR2  = 0x58UL;
  static constexpr uint32_t UNOQ_GPDMA_CLLR  = 0x7CUL;

  // GPDMA status / interrupt bits. IDLEF is status-only; CFCR clear bits
  // begin at TCF (bit 8).
  static constexpr uint32_t UNOQ_GPDMA_FLAG_IDLEF = (1UL << 0);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_TCF   = (1UL << 8);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_HTF   = (1UL << 9);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_DTEF  = (1UL << 10);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_ULEF  = (1UL << 11);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_USEF  = (1UL << 12);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_SUSPF = (1UL << 13);
  static constexpr uint32_t UNOQ_GPDMA_FLAG_TOF   = (1UL << 14);
  static constexpr uint32_t UNOQ_GPDMA_ALL_FLAGS =
      UNOQ_GPDMA_FLAG_TCF | UNOQ_GPDMA_FLAG_HTF |
      UNOQ_GPDMA_FLAG_DTEF | UNOQ_GPDMA_FLAG_ULEF |
      UNOQ_GPDMA_FLAG_USEF | UNOQ_GPDMA_FLAG_SUSPF |
      UNOQ_GPDMA_FLAG_TOF;
  static constexpr uint32_t UNOQ_GPDMA_ERROR_FLAGS =
      UNOQ_GPDMA_FLAG_DTEF | UNOQ_GPDMA_FLAG_ULEF |
      UNOQ_GPDMA_FLAG_USEF | UNOQ_GPDMA_FLAG_TOF;

  static constexpr uint32_t UNOQ_GPDMA_CCR_EN    = (1UL << 0);
  static constexpr uint32_t UNOQ_GPDMA_CCR_RESET = (1UL << 1);
  static constexpr uint32_t UNOQ_GPDMA_CCR_SUSP  = (1UL << 2);
  static constexpr uint32_t UNOQ_GPDMA_CCR_TCIE  = (1UL << 8);
  static constexpr uint32_t UNOQ_GPDMA_CCR_HTIE  = (1UL << 9);
  static constexpr uint32_t UNOQ_GPDMA_CCR_DTEIE = (1UL << 10);
  static constexpr uint32_t UNOQ_GPDMA_CCR_ULEIE = (1UL << 11);
  static constexpr uint32_t UNOQ_GPDMA_CCR_USEIE = (1UL << 12);
  static constexpr uint32_t UNOQ_GPDMA_CCR_TOIE  = (1UL << 14);
  static constexpr uint32_t UNOQ_GPDMA_CCR_IRQ_MASK =
      UNOQ_GPDMA_CCR_TCIE | UNOQ_GPDMA_CCR_DTEIE |
      UNOQ_GPDMA_CCR_ULEIE | UNOQ_GPDMA_CCR_USEIE |
      UNOQ_GPDMA_CCR_TOIE;

  // CTR1 field values used by the staged v18.4 diagnostics.
  // GPDMA source/destination data-width fields are LOG2(bytes):
  // halfword=01, word=10. Source increment is bit 3, destination increment
  // bit 19. ST recommends source port 0 / destination port 1 for ADC->SRAM.
  static constexpr uint32_t UNOQ_GPDMA_CTR1_SDW_HALFWORD = (1UL << 0);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_SDW_WORD     = (1UL << 1);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_SINC         = (1UL << 3);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_SRC_PORT0    = 0UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DDW_HALFWORD = (1UL << 16);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DDW_WORD     = (1UL << 17);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DINC         = (1UL << 19);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DEST_PORT0   = 0UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DEST_PORT1   = (1UL << 30);
  static constexpr uint32_t UNOQ_GPDMA_CTR1_SRC_BURST1   = 0UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR1_DEST_BURST1  = 0UL;

  // ADC1 is native 14-bit, so transport each conversion as one 16-bit
  // halfword. This keeps one ADC conversion == one DMA halfword and makes the
  // 8-sample block exactly 16 bytes. Source stays fixed at ADC_DR; destination
  // increments through the uint16_t DMA buffer.
  static constexpr uint32_t UNOQ_GPDMA_CTR1_ADC16_TO_MEM16 =
      UNOQ_GPDMA_CTR1_SDW_HALFWORD |
      UNOQ_GPDMA_CTR1_DDW_HALFWORD |
      UNOQ_GPDMA_CTR1_DINC |
      UNOQ_GPDMA_CTR1_SRC_PORT0 |
      UNOQ_GPDMA_CTR1_DEST_PORT1 |
      UNOQ_GPDMA_CTR1_SRC_BURST1 |
      UNOQ_GPDMA_CTR1_DEST_BURST1;

  static constexpr uint32_t UNOQ_GPDMA_CTR1_M2M32 =
      UNOQ_GPDMA_CTR1_SDW_WORD |
      UNOQ_GPDMA_CTR1_SINC |
      UNOQ_GPDMA_CTR1_DDW_WORD |
      UNOQ_GPDMA_CTR1_DINC |
      UNOQ_GPDMA_CTR1_SRC_PORT0 |
      UNOQ_GPDMA_CTR1_DEST_PORT1;

  // CTR2 bit 9 SWREQ=1 requests memory-to-memory transfer. For ADC1,
  // SWREQ=0, DREQ=0 (source peripheral) and BREQ=0 (single/burst request).
  static constexpr uint32_t UNOQ_GPDMA_CTR2_SWREQ = (1UL << 9);
  static constexpr uint32_t UNOQ_GPDMA_CTR2_REQSEL_ADC1 = 0UL;
  static constexpr uint32_t UNOQ_GPDMA_CTR2_M2M_SW = UNOQ_GPDMA_CTR2_SWREQ;
  static constexpr uint32_t UNOQ_GPDMA_CTR2_ADC1_P2M_SINGLE =
      UNOQ_GPDMA_CTR2_REQSEL_ADC1;

  // Common timer bits.
  static constexpr uint32_t UNOQ_TIM_CR1_CEN = (1UL << 0);
  static constexpr uint32_t UNOQ_TIM_DIER_UIE = (1UL << 0);
  static constexpr uint32_t UNOQ_TIM_SR_UIF = (1UL << 0);
  static constexpr uint32_t UNOQ_TIM_EGR_UG = (1UL << 0);

  // TIM15 TRGO = update event (MMS = 010, CR2 bits 6:4).
  static constexpr uint32_t UNOQ_TIM_CR2_MMS_MASK = (7UL << 4);
  static constexpr uint32_t UNOQ_TIM_CR2_MMS_UPDATE = (2UL << 4);

  // ADC1 control/status bits.
  static constexpr uint32_t UNOQ_ADC_ISR_ADRDY = (1UL << 0);
  static constexpr uint32_t UNOQ_ADC_ISR_EOC   = (1UL << 2);
  static constexpr uint32_t UNOQ_ADC_ISR_EOS   = (1UL << 3);
  static constexpr uint32_t UNOQ_ADC_ISR_OVR   = (1UL << 4);

  static constexpr uint32_t UNOQ_ADC_CR_ADEN   = (1UL << 0);
  static constexpr uint32_t UNOQ_ADC_CR_ADDIS  = (1UL << 1);
  static constexpr uint32_t UNOQ_ADC_CR_ADSTART = (1UL << 2);
  static constexpr uint32_t UNOQ_ADC_CR_ADSTP  = (1UL << 4);

  // STM32U585 ADC1 CFGR1:
  // DMNGT[1:0], RES[3:2], EXTSEL[9:5], EXTEN[11:10].
  static constexpr uint32_t UNOQ_ADC_CFGR1_DMNGT_MASK = (3UL << 0);
  static constexpr uint32_t UNOQ_ADC_CFGR1_RES_MASK   = (3UL << 2);
  static constexpr uint32_t UNOQ_ADC_CFGR1_EXTSEL_MASK = (31UL << 5);
  static constexpr uint32_t UNOQ_ADC_CFGR1_EXTEN_MASK = (3UL << 10);
  static constexpr uint32_t UNOQ_ADC_CFGR1_OVRMOD = (1UL << 12);
  static constexpr uint32_t UNOQ_ADC_CFGR1_CONT   = (1UL << 13);
  static constexpr uint32_t UNOQ_ADC_CFGR1_AUTDLY = (1UL << 14);
  static constexpr uint32_t UNOQ_ADC_CFGR1_DISCEN = (1UL << 16);

  // ADC1 native 14-bit is RES = 00.
  static constexpr uint32_t UNOQ_ADC_CFGR1_RES_14BIT = 0UL;
  // STM32U5 LL semantics: 00=DR/no DMA, 01=DMA limited/one-shot,
  // 11=DMA unlimited/circular. v18.4 intentionally uses limited because the
  // GPDMA block itself is linear/non-circular and is explicitly re-armed.
  static constexpr uint32_t UNOQ_ADC_DMA_MODE_NONE = 0UL;
  static constexpr uint32_t UNOQ_ADC_DMA_MODE_LIMITED = (1UL << 0);
  static constexpr uint32_t UNOQ_ADC_DMA_MODE_UNLIMITED = (3UL << 0);

  // ADC1 regular external trigger 14 = TIM15 TRGO; rising edge = EXTEN 01.
  static constexpr uint32_t UNOQ_ADC_CFGR1_EXTSEL_TIM15_TRGO = (14UL << 5);
  static constexpr uint32_t UNOQ_ADC_CFGR1_EXTEN_RISING = (1UL << 10);

  // A1 on UNO Q = PA5 = ADC1 channel 10.
  static constexpr uint32_t UNOQ_ADC_A1_CHANNEL = 10UL;
  static constexpr uint32_t UNOQ_ADC_PCSEL_A1 = (1UL << UNOQ_ADC_A1_CHANNEL);
  static constexpr uint32_t UNOQ_ADC_SQR1_L_MASK = 0xFUL;
  static constexpr uint32_t UNOQ_ADC_SQR1_SQ1_MASK = (31UL << 6);
  static constexpr uint32_t UNOQ_ADC_SQR1_SQ1_A1 =
      (UNOQ_ADC_A1_CHANNEL << 6);

  #define UNOQ_MMIO32(addr) (*(volatile uint32_t *)(addr))
  #define UNOQ_TIM6_REG(off)  UNOQ_MMIO32(UNOQ_TIM6_BASE + (off))
  #define UNOQ_TIM15_REG(off) UNOQ_MMIO32(UNOQ_TIM15_BASE + (off))
  #define UNOQ_RCC_REG(off)   UNOQ_MMIO32(UNOQ_RCC_BASE + (off))
  #define UNOQ_ADC1_REG(off)  UNOQ_MMIO32(UNOQ_ADC1_BASE + (off))
  #define UNOQ_DAC1_REG(off)  UNOQ_MMIO32(UNOQ_DAC1_BASE + (off))
  #define UNOQ_GPDMA1_CH_REG(off) \
      UNOQ_MMIO32(UNOQ_GPDMA1_CH_BASE + (off))
#endif

// ArduinoCore-zephyr exports SystemCoreClock to LLEXT on ARM.
#if defined(ARDUINO_UNO_Q)
extern "C" uint32_t SystemCoreClock;
#endif

static constexpr uint32_t MCU_SAMPLE_RATE_HZ = 2000;  // fixed low-latency ADC production rate
static constexpr uint32_t RATE_MEASURE_TICKS = 128;
static constexpr uint16_t ADC_BUFFER_SIZE = 2048;
static constexpr uint16_t RPC_BATCH_SIZE = 256;

// v19.16 final-stability candidate: ADC is reduced to an exact 2.0 kS/s
// to give the Bridge more transport margin, while TIM6 is set to an exact
// 10.0 kHz DAC update cadence. On a 160 MHz core the DAC divider is 16000.
// DAC output range remains 1–100 Hz.
static constexpr uint32_t DAC_HW_TIMER_EXACT_DIVIDER = 16000;
static constexpr uint32_t DAC_HW_TIMER_FALLBACK_HZ = 10000;

// TIM6 global interrupt number on STM32U585 (Zephyr DTS: timers6 IRQ 49).
#define UNOQ_DAC_TIMER_IRQ 49
#define UNOQ_DAC_TIMER_IRQ_PRIORITY 1

// v19.16 ADC hardware sampling production baseline.
// TIM15 update event physically triggers ADC1 conversion at an exact 2000 S/s.
// The verified low-latency IRQ-read backend is the production acquisition path.
// The GPDMA implementation remains in source but is excluded from runtime initialization.
static constexpr uint32_t ADC_HW_TIMER_DEFAULT_HZ = 2000;
static constexpr uint32_t ADC_HW_TIMER_COUNTER_TARGET_HZ = 1000000;  // keep CNT diagnostics in true microseconds
static constexpr uint32_t ADC_HW_EOC_TIMEOUT_US = 200;
static constexpr uint16_t ADC_DMA_BLOCK_SAMPLES = 8;
static constexpr uint32_t ADC_DMA_SELFTEST_US = 100000;

#define UNOQ_ADC_TIMER_IRQ 69
#define UNOQ_ADC_TIMER_IRQ_PRIORITY 2
#define UNOQ_ADC_DMA_IRQ 29
#define UNOQ_ADC_DMA_IRQ_PRIORITY 3
#define UNOQ_ADC_DMA_CHANNEL 0
#define UNOQ_ADC_DMA_REQUEST 0

// v19.11: GPDMA was validated in v18.5 but is intentionally disabled at runtime.
// Keep the complete implementation below for future higher-rate experiments.
#define UNOQ_ADC_DMA_RUNTIME_ENABLED 0

// Software fallback retained for non-UNO-Q / unsupported builds.
static constexpr uint32_t DAC_MIN_UPDATE_HZ = 200;
static constexpr uint32_t DAC_MAX_UPDATE_HZ_ADC_STOP = 1600;
static constexpr uint32_t DAC_MAX_UPDATE_HZ_ADC_RUN = 1600;
static constexpr uint32_t DAC_SAMPLES_PER_CYCLE = 64;
static constexpr uint32_t DAC_SQUARE_EDGE_GUARD_US = 180;

volatile int g_waveform = 1;          // Sine
volatile int g_frequency_mHz = 10000; // 10 Hz
volatile int g_amplitude_mV = 1000;   // 1.00 Vpk
volatile int g_offset_mV = 1650;      // 1.65 V

// v19.20 optional additive noise injection. 100 % corresponds to a
// uniform random peak of +/-50 % of the configured Vpk. Noise is OFF by
// default and is applied only at the final DAC-code stage, after the existing
// DDS waveform calculation and before the existing 0..4095 clamp.
volatile bool g_noise_enabled = false;
volatile int g_noise_level_percent = 30;
volatile uint16_t g_noise_peak_code = 0;
volatile uint32_t g_noise_rng_state = 0xA341316CUL;

volatile bool g_dac_running = false;
volatile bool g_adc_running = false;

// This counter remains the shared acquisition timebase. It advances once per
// executed ADC clock tick even while ADC is STOPPED, preserving the existing
// phase-origin/counter protocol used by Linux and the browser.
volatile uint32_t g_sample_counter = 0;
volatile uint32_t g_phase_origin = 0;

volatile uint16_t g_adc_buffer[ADC_BUFFER_SIZE];
volatile uint16_t g_adc_write_index = 0;
volatile uint16_t g_adc_count = 0;

volatile uint32_t g_dac_phase_start_us = 0;
volatile uint32_t g_dac_next_update_us = 0;
volatile uint32_t g_square_edge_index = 0;
volatile uint16_t g_dac_last_code = 0xFFFF;

// Hardware-timer DDS state. ISR is integer-only.
volatile uint32_t g_dac_phase_accum = 0;
volatile uint32_t g_dac_phase_step = 0;
volatile uint16_t g_dac_offset_code = 2048;
volatile uint16_t g_dac_amplitude_code = 1241;

// v19.11 DAC diagnostics: output-code diagnostics remain on-demand.
// The ISR hot path records only lightweight TIM6 counter timing maxima and an
// overrun count so the 10 kHz timing margin can be verified without micros().

// Truthful runtime status: zero means hardware timer is NOT active.
volatile uint32_t g_dac_timer_rate_hz = 0;
volatile uint32_t g_dac_timer_tick_count = 0;
volatile uint32_t g_dac_irq_count = 0;
volatile uint32_t g_dac_isr_max_cycles = 0;
volatile uint32_t g_dac_irq_late_max_cycles = 0;
volatile uint32_t g_dac_overrun_count = 0;
volatile uint32_t g_dac_timer_divider = DAC_HW_TIMER_EXACT_DIVIDER;
volatile bool g_dac_hw_timer_active = false;

// Diagnostic bitmap:
// bit 0  UNO Q board build
// bit 1  Zephyr IRQ header available
// bit 2  CONFIG_DYNAMIC_INTERRUPTS enabled
// bit 3  TIM6 RCC clock enable verified
// bit 4  TIM6 counter verified running
// bit 5  dynamic IRQ connected
// bit 6  IRQ line enabled
// bit 7  TIM6 IRQ self-test fired
volatile uint32_t g_dac_timer_diag = 0;
volatile int32_t g_dac_irq_connect_result = -999;

int16_t g_sine_lut_q15[256];

volatile uint32_t g_actual_sample_rate_mHz = 2000000;
volatile uint32_t g_adc_requested_rate_hz = ADC_HW_TIMER_DEFAULT_HZ;
volatile uint32_t g_software_sample_period_us = 417;

// ADC hardware-trigger state.
// Truthful runtime status: timer rate is zero unless TIM15->ADC1 self-test
// has actually completed.
volatile bool g_adc_hw_timer_active = false;
volatile bool g_adc_selftest_mode = false;
volatile uint32_t g_adc_timer_rate_hz = 0;
volatile uint32_t g_adc_timer_counter_hz = 0;
volatile uint32_t g_adc_timer_diag = 0;
volatile int32_t g_adc_irq_connect_result = -999;
volatile uint32_t g_adc_timer_irq_count = 0;
volatile uint32_t g_adc_hw_conversion_count = 0;
volatile uint32_t g_adc_conversion_timeouts = 0;
volatile uint16_t g_adc_last_code = 0;

// v19.0 retained GPDMA staged diagnostic backend (runtime-disabled).
// The ADC remains 14-bit and DMA transports each result in a uint16_t.
// Boot diagnostics compare 4-sample and 8-sample blocks with measured elapsed
// time, TIM15 IRQ count, remaining CBR1 bytes and DMA half/complete IRQs.
alignas(32) volatile uint16_t g_adc_dma_buffer[ADC_DMA_BLOCK_SAMPLES];
alignas(32) volatile uint32_t g_dma_m2m_src[4] = {
    0x13579BDFUL, 0x2468ACE0UL, 0x0BADB002UL, 0x55AA33CCUL
};
alignas(32) volatile uint32_t g_dma_m2m_dst[4] = {0, 0, 0, 0};
alignas(32) volatile uint16_t g_dma_adc_single_value = 0xFFFFU;

volatile bool g_adc_dma_active = false;
volatile bool g_adc_dma_selftest_mode = false;
volatile bool g_adc_dma_observed_data = false;
volatile uint32_t g_adc_dma_diag = 0;
volatile int32_t g_adc_dma_irq_connect_result = -999;
volatile uint32_t g_adc_dma_transfer_count = 0;
volatile uint32_t g_adc_dma_block_count = 0;
volatile uint32_t g_adc_dma_error_count = 0;
volatile uint32_t g_adc_overrun_count = 0;
volatile uint32_t g_adc_dma_service_last_us = 0;
volatile uint32_t g_adc_dma_service_max_us = 0;
volatile uint32_t g_adc_dma_last_status = 0;
volatile uint32_t g_adc_dma_run_start_counter = 0;
volatile uint32_t g_adc_latest_counter = 0;

// Persistent boot diagnostics. These snapshots survive fallback so /api/status
// shows the exact stage/register state that failed rather than the registers
// after the channel has already been reset.
volatile uint32_t g_dma_test_stage = 0;
volatile uint32_t g_dma_m2m_pass = 0;
volatile uint32_t g_dma_m2m_status = 0;
volatile uint32_t g_dma_m2m_dst0 = 0;
volatile uint32_t g_dma_adc_single_pass = 0;
volatile uint32_t g_dma_adc_single_status = 0;
volatile uint32_t g_dma_adc_single_sample = 0xFFFFUL;
volatile uint32_t g_dma_adc_block_pass = 0;
volatile uint32_t g_dma_snap_ccr = 0;
volatile uint32_t g_dma_snap_ctr1 = 0;
volatile uint32_t g_dma_snap_ctr2 = 0;
volatile uint32_t g_dma_snap_cbr1 = 0;
volatile uint32_t g_dma_snap_csar = 0;
volatile uint32_t g_dma_snap_cdar = 0;
volatile uint32_t g_dma_snap_csr = 0;
volatile uint32_t g_dma_snap_adc_cfgr1 = 0;
volatile uint32_t g_dma_snap_adc_cr = 0;
volatile uint32_t g_dma_snap_adc_isr = 0;
volatile uint32_t g_adc_baseline_eoc_count = 0;

// v19.0 retained block-level evidence. These values survive fallback so /api/status can
// distinguish an actual request-count limitation from a self-test timing issue.
volatile uint16_t g_adc_dma_programmed_samples = ADC_DMA_BLOCK_SAMPLES;
volatile bool g_adc_dma_diag_no_rearm = false;
volatile uint32_t g_dma_block_elapsed_us = 0;
volatile uint32_t g_dma_block_tim15_irq_count = 0;
volatile uint32_t g_dma_block_adc_requests_observed = 0;
volatile uint32_t g_dma_block_cbr1_start = 0;
volatile uint32_t g_dma_block_cbr1_end = 0;
volatile uint32_t g_dma_irq_count = 0;
volatile uint32_t g_dma_half_count = 0;
volatile uint32_t g_dma_complete_count = 0;
volatile uint32_t g_dma_idle_complete_count = 0;
volatile uint32_t g_dma_adc_4sample_pass = 0;
volatile uint32_t g_dma_adc_4sample_status = 0;
volatile uint32_t g_dma_adc_4sample_elapsed_us = 0;
volatile uint32_t g_dma_adc_4sample_tim15_irq_count = 0;
volatile uint32_t g_dma_adc_4sample_cbr1_end = 0;
volatile uint32_t g_dma_adc_4sample_requests = 0;
volatile uint32_t g_dma_adc_8sample_pass = 0;
volatile uint32_t g_dma_adc_8sample_status = 0;
volatile uint32_t g_dma_adc_8sample_elapsed_us = 0;
volatile uint32_t g_dma_adc_8sample_tim15_irq_count = 0;
volatile uint32_t g_dma_adc_8sample_cbr1_end = 0;
volatile uint32_t g_dma_adc_8sample_requests = 0;

// DMA diagnostic bitmap:
// bit 0  UNO Q board build
// bit 1  GPDMA1 RCC clock enable verified
// bit 2  GPDMA1 channel 0 idle / claimable
// bit 3  RAM -> RAM software-request DMA self-test passed
// bit 4  GPDMA1 channel 0 dynamic IRQ connected and enabled
// bit 5  TIM15 -> ADC1 -> one-sample GPDMA handshake passed
// bit 6  8-sample ADC DMA block/IRQ/re-arm self-test passed
// bit 7  runtime block path ready

// Diagnostic bitmap:
// bit 0  UNO Q board build
// bit 1  Zephyr IRQ header available
// bit 2  CONFIG_DYNAMIC_INTERRUPTS enabled
// bit 3  TIM15 RCC clock enable verified
// bit 4  TIM15 counter verified running
// bit 5  TIM15 dynamic IRQ connected
// bit 6  TIM15 IRQ line enabled
// bit 7  TIM15 TRGO -> ADC1 CH10 14-bit conversion self-test passed

// Existing timing fields keep the API stable.
// In the IRQ-read fallback, adc_read_* = trigger-to-result latency.
// In DMA mode adc_read_* is zero; adc_dma_service_* measures the DMA block
// completion service instead. scheduler_late_max remains TIM15 IRQ-entry
// latency for the lightweight shared acquisition counter.
volatile uint32_t g_adc_read_last_us = 0;
volatile uint32_t g_adc_read_max_us = 0;
volatile uint32_t g_scheduler_late_max_us = 0;
uint32_t g_rate_start_counter = 0;
uint32_t g_rate_start_us = 0;
uint32_t g_next_sample_us = 0;

static inline int clampInt(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static void appendHex4(String &out, uint16_t value) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  out += HEX_DIGITS[(value >> 12) & 0x0F];
  out += HEX_DIGITS[(value >> 8) & 0x0F];
  out += HEX_DIGITS[(value >> 4) & 0x0F];
  out += HEX_DIGITS[value & 0x0F];
}

static void appendPackedAdcPair(String &out, uint16_t first, uint16_t second) {
  // Two native 14-bit samples fit in 28 bits.  Five printable 6-bit symbols
  // carry the pair (30 available bits), reducing payload from 8 to 5 chars.
  static const char PACK64[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  const uint32_t packed =
      ((uint32_t)(first & 0x3FFFU) << 14) | (uint32_t)(second & 0x3FFFU);
  out += PACK64[(packed >> 24) & 0x3FU];
  out += PACK64[(packed >> 18) & 0x3FU];
  out += PACK64[(packed >> 12) & 0x3FU];
  out += PACK64[(packed >> 6) & 0x3FU];
  out += PACK64[packed & 0x3FU];
}

static inline uint16_t millivoltsToDacCode(float millivolts) {
  if (millivolts < 0.0f) millivolts = 0.0f;
  if (millivolts > 3300.0f) millivolts = 3300.0f;
  return (uint16_t)lroundf(millivolts * 4095.0f / 3300.0f);
}

static inline void writeDacHardware(uint16_t code) {
  code &= 0x0FFF;

#if UNOQ_FAST_DAC_REGISTER
  // analogWrite(DAC0, 0) in setup() performs one-time Arduino/Zephyr
  // DAC pin/channel setup.  The hot path then writes the verified UNO Q
  // DAC1 DHR12R1 MMIO register directly, including from the TIM6 ISR.
  UNOQ_DAC1_REG(UNOQ_DAC_DHR12R1) = (uint32_t)code;
#else
  analogWrite(DAC0, code);
#endif
}

static inline void writeDacCodeIfChanged(uint16_t code) {
  if (code == g_dac_last_code) return;
  writeDacHardware(code);
  g_dac_last_code = code;
}

static inline uint16_t clampDacCode(int32_t code) {
  if (code < 0) return 0;
  if (code > 4095) return 4095;
  return (uint16_t)code;
}

static inline uint16_t millivoltsToDacCodeInt(int millivolts) {
  int mv = millivolts;
  if (mv < 0) mv = 0;
  if (mv > 3300) mv = 3300;
  return (uint16_t)(((uint32_t)mv * 4095UL + 1650UL) / 3300UL);
}

static inline void updateNoiseDerivedStateLocked() {
  const uint32_t amplitude_code = millivoltsToDacCodeInt(g_amplitude_mV);
  const uint32_t level = (uint32_t)clampInt(g_noise_level_percent, 0, 100);

  // level=100 -> 0.5 * Vpk, expressed in DAC codes.
  g_noise_peak_code = (uint16_t)((amplitude_code * level + 100UL) / 200UL);
}

static inline uint16_t applyDacNoise(uint16_t clean_code) {
  if (!g_noise_enabled) return clean_code;

  const uint16_t peak = g_noise_peak_code;
  if (peak == 0U) return clean_code;

  // xorshift32: tiny integer-only PRNG suitable for the 10 kHz ISR hot path.
  uint32_t x = g_noise_rng_state;
  if (x == 0U) x = 0xA341316CUL;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  g_noise_rng_state = x;

  const int32_t signed_q15 = (int32_t)(int16_t)(x >> 16);
  const int32_t delta = (signed_q15 * (int32_t)peak) >> 15;
  return clampDacCode((int32_t)clean_code + delta);
}

static void buildSineLut() {
  for (int i = 0; i < 256; ++i) {
    const float phase = (float)i / 256.0f;
    g_sine_lut_q15[i] =
        (int16_t)lroundf(32767.0f * sinf(2.0f * PI * phase));
  }
}

static inline void updateDacDerivedStateLocked() {
  g_dac_offset_code = millivoltsToDacCodeInt(g_offset_mV);
  g_dac_amplitude_code = millivoltsToDacCodeInt(g_amplitude_mV);
  updateNoiseDerivedStateLocked();

  const uint32_t timer_hz =
      (g_dac_hw_timer_active && g_dac_timer_rate_hz > 0)
          ? g_dac_timer_rate_hz
          : DAC_HW_TIMER_FALLBACK_HZ;

  g_dac_phase_step = (uint32_t)(
      ((uint64_t)(uint32_t)g_frequency_mHz << 32) /
      ((uint64_t)timer_hz * 1000ULL)
  );
}

static inline int32_t triangleQ15(uint32_t phase) {
  const uint32_t p = phase >> 16; // 0..65535
  if (p < 32768U) {
    return ((int32_t)p << 1) - 32767;
  }
  return 98303 - ((int32_t)p << 1);
}

static inline uint16_t dacCodeFromPhase(uint32_t phase) {
  const int waveform = g_waveform;
  const int32_t offset = (int32_t)g_dac_offset_code;
  const int32_t amp = (int32_t)g_dac_amplitude_code;

  if (waveform == 0) {
    return clampDacCode(offset);
  }

  if (waveform == 2) {
    // phase MSB is the exact 50 % duty boundary.
    return clampDacCode(
        offset + ((phase & 0x80000000UL) ? -amp : amp)
    );
  }

  int32_t q15 = 0;
  if (waveform == 1) {
    q15 = (int32_t)g_sine_lut_q15[(phase >> 24) & 0xFFU];
  } else {
    q15 = triangleQ15(phase);
  }

  const int32_t delta = (amp * q15) >> 15;
  return clampDacCode(offset + delta);
}

#if UNOQ_DAC_HW_TIMER
static void dacTimerIsr(const void *arg) {
  (void)arg;

  // TIM6 runs with PSC=0, so CNT directly measures core-clock timer cycles
  // elapsed since the update event. Two CNT reads add only a few MMIO ops and
  // avoid the much heavier micros() call in this 10 kHz hot path.
  const uint32_t entry_cnt = UNOQ_TIM6_REG(UNOQ_TIM_CNT);
  if (entry_cnt > g_dac_irq_late_max_cycles) {
    g_dac_irq_late_max_cycles = entry_cnt;
  }

  // Clear update interrupt first.
  UNOQ_TIM6_REG(UNOQ_TIM_SR) &= ~UNOQ_TIM_SR_UIF;
  g_dac_irq_count++;

  if (g_dac_running) {
    const uint32_t phase = g_dac_phase_accum;
    g_dac_phase_accum += g_dac_phase_step;
    g_dac_timer_tick_count++;
    writeDacCodeIfChanged(applyDacNoise(dacCodeFromPhase(phase)));
  }

  const uint32_t exit_cnt = UNOQ_TIM6_REG(UNOQ_TIM_CNT);
  const bool counter_wrapped = exit_cnt < entry_cnt;
  uint32_t elapsed_cycles = counter_wrapped
      ? (g_dac_timer_divider - entry_cnt) + exit_cnt
      : exit_cnt - entry_cnt;
  if (elapsed_cycles > g_dac_isr_max_cycles) {
    g_dac_isr_max_cycles = elapsed_cycles;
  }

  // If another update arrived before ISR exit, UIF is set again. Count one
  // overrun event even when the counter-wrap test detects the same period miss.
  const bool update_arrived_during_isr =
      (UNOQ_TIM6_REG(UNOQ_TIM_SR) & UNOQ_TIM_SR_UIF) != 0U;
  if (counter_wrapped || update_arrived_during_isr) {
    g_dac_overrun_count++;
  }
}

static void disableTim6Hardware() {
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM6_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;
  g_dac_hw_timer_active = false;
  g_dac_timer_rate_hz = 0;
}

static bool setupDacHardwareTimer() {
  g_dac_hw_timer_active = false;
  g_dac_timer_rate_hz = 0;
  g_dac_timer_diag = 0;
  g_dac_irq_connect_result = -999;
  g_dac_irq_count = 0;
  g_dac_isr_max_cycles = 0;
  g_dac_irq_late_max_cycles = 0;
  g_dac_overrun_count = 0;

  g_dac_timer_diag |= (1UL << 0); // UNO Q
#if UNOQ_HAS_ZEPHYR_IRQ_HEADER
  g_dac_timer_diag |= (1UL << 1);
#endif
#if UNOQ_HAS_DYNAMIC_IRQ
  g_dac_timer_diag |= (1UL << 2);
#endif

  // Enable and reset TIM6 through the verified STM32U5 RCC MMIO registers.
  UNOQ_RCC_REG(UNOQ_RCC_APB1ENR1) |= UNOQ_RCC_TIM6_BIT;
  (void)UNOQ_RCC_REG(UNOQ_RCC_APB1ENR1);

  if ((UNOQ_RCC_REG(UNOQ_RCC_APB1ENR1) & UNOQ_RCC_TIM6_BIT) == 0) {
    return false;
  }
  g_dac_timer_diag |= (1UL << 3);

  UNOQ_RCC_REG(UNOQ_RCC_APB1RSTR1) |= UNOQ_RCC_TIM6_BIT;
  UNOQ_RCC_REG(UNOQ_RCC_APB1RSTR1) &= ~UNOQ_RCC_TIM6_BIT;

  uint32_t timer_clock_hz = SystemCoreClock;
  if (timer_clock_hz == 0) timer_clock_hz = 160000000UL;

  // v19.16 DAC timing: TIM6 runs at an exact 10 kHz. ADC production is
  // independently fixed at 2.0 kS/s to improve Bridge transport margin.
  uint32_t divider = DAC_HW_TIMER_EXACT_DIVIDER;
  if (divider < 2U) divider = 2U;
  if (divider > 65536U) divider = 65536U;
  g_dac_timer_divider = divider;

  const uint32_t actual_hz = timer_clock_hz / divider;

  // Configure TIM6 with IRQ disabled first.
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_PSC) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_ARR) = divider - 1U;
  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_EGR) = UNOQ_TIM_EGR_UG;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;

  // First prove the peripheral clock/counter really runs.
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
  const uint32_t c0 = UNOQ_TIM6_REG(UNOQ_TIM_CNT);
  delayMicroseconds(20);
  const uint32_t c1 = UNOQ_TIM6_REG(UNOQ_TIM_CNT);
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;

  if (c0 == c1) {
    disableTim6Hardware();
    return false;
  }
  g_dac_timer_diag |= (1UL << 4);

  // LLEXT cannot use IRQ_CONNECT(), because that is a static link-time table
  // operation. ArduinoCore-zephyr exports the dynamic IRQ backend on UNO Q.
  const int vector = irq_connect_dynamic(
      UNOQ_DAC_TIMER_IRQ,
      UNOQ_DAC_TIMER_IRQ_PRIORITY,
      dacTimerIsr,
      NULL,
      0
  );
  g_dac_irq_connect_result = vector;

  if (vector < 0) {
    disableTim6Hardware();
    return false;
  }
  g_dac_timer_diag |= (1UL << 5);

  irq_enable(UNOQ_DAC_TIMER_IRQ);
  if (!irq_is_enabled(UNOQ_DAC_TIMER_IRQ)) {
    disableTim6Hardware();
    return false;
  }
  g_dac_timer_diag |= (1UL << 6);

  // Runtime IRQ self-test. DAC is still STOPPED, so the ISR only clears UIF
  // and increments g_dac_irq_count. About 10 interrupts are expected in 1 ms.
  g_dac_irq_count = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;

  delayMicroseconds(1200);

  UNOQ_TIM6_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;

  if (g_dac_irq_count == 0) {
    disableTim6Hardware();
    return false;
  }
  g_dac_timer_diag |= (1UL << 7);

  // Only NOW is the hardware timer considered valid and reportable.
  g_dac_timer_rate_hz = actual_hz;
  g_dac_hw_timer_active = true;
  updateDacDerivedStateLocked();

  // Keep UIE armed; CEN remains stopped until DAC RUN.
  UNOQ_TIM6_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  return true;
}

static inline void startDacHardwareTimerLocked() {
  if (!g_dac_hw_timer_active) return;

  g_dac_phase_accum = 0;
  g_dac_timer_tick_count = 0;
  g_dac_last_code = 0xFFFF;
  g_dac_isr_max_cycles = 0;
  g_dac_irq_late_max_cycles = 0;
  g_dac_overrun_count = 0;

  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM6_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
}

static inline void stopDacHardwareTimerLocked() {
  if (!g_dac_hw_timer_active) return;

  UNOQ_TIM6_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM6_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM6_REG(UNOQ_TIM_SR) = 0;
}
#endif

float waveformValue(float phase, int waveform) {
  switch (waveform) {
    case 0: return 0.0f;
    case 1: return sinf(2.0f * PI * phase);
    case 2: return (phase < 0.5f) ? 1.0f : -1.0f;
    case 3: return 1.0f - 4.0f * fabsf(phase - 0.5f);
    default: return 0.0f;
  }
}

static inline float dacPhaseAt(uint32_t now_us) {
  const float frequency_hz = (float)g_frequency_mHz / 1000.0f;
  const uint32_t elapsed_us = now_us - g_dac_phase_start_us;
  float phase = frequency_hz * ((float)elapsed_us / 1000000.0f);
  phase -= floorf(phase);
  return phase;
}

static inline uint32_t squareEdgeOffsetUs(uint32_t edge_index) {
  const uint32_t frequency_mHz = (uint32_t)g_frequency_mHz;
  if (frequency_mHz == 0) return 0;

  // Half-period = 500,000,000 / frequency_mHz microseconds.
  // Multiplying before division preserves fractional-microsecond scheduling
  // across successive edges instead of quantizing every half-cycle.
  return (uint32_t)(
      ((uint64_t)edge_index * 500000000ULL) /
      (uint64_t)frequency_mHz
  );
}

static inline uint32_t squareHalfIndexAt(uint32_t now_us) {
  const uint32_t frequency_mHz = (uint32_t)g_frequency_mHz;
  if (frequency_mHz == 0) return 0;

  const uint32_t elapsed_us = now_us - g_dac_phase_start_us;
  return (uint32_t)(
      ((uint64_t)elapsed_us * (uint64_t)frequency_mHz) /
      500000000ULL
  );
}

static uint32_t dacUpdatePeriodUs() {
  const float frequency_hz = (float)g_frequency_mHz / 1000.0f;
  uint32_t update_hz = (uint32_t)lroundf(
      fmaxf((float)DAC_MIN_UPDATE_HZ,
            frequency_hz * (float)DAC_SAMPLES_PER_CYCLE));

  const uint32_t max_hz = g_adc_running
      ? DAC_MAX_UPDATE_HZ_ADC_RUN
      : DAC_MAX_UPDATE_HZ_ADC_STOP;

  if (update_hz > max_hz) update_hz = max_hz;
  if (update_hz < 1) update_hz = 1;
  return 1000000UL / update_hz;
}

void serviceDAC(uint32_t now_us) {
#if UNOQ_DAC_HW_TIMER
  if (g_dac_hw_timer_active) {
    // DAC timing is owned by the verified TIM6 dynamic-IRQ backend.
    (void)now_us;
    return;
  }
#endif

  // Safe software fallback remains available if TIM6 self-test fails.
  if (!g_dac_running) return;

  const int waveform = g_waveform;

  // DC is static: write once after RUN/config change.
  if (waveform == 0) {
    writeDacCodeIfChanged(applyDacNoise(millivoltsToDacCode((float)g_offset_mV)));
    return;
  }

  // Square uses absolute half-cycle deadlines. It is not tied to the
  // ADC clock. A short busy-wait guard catches an imminent edge before the
  // sketch enters another potentially blocking operation.
  if (waveform == 2) {
    if (g_dac_next_update_us == 0) {
      g_square_edge_index = 1;
      const float mv =
          (float)g_offset_mV + (float)g_amplitude_mV;
      writeDacCodeIfChanged(applyDacNoise(millivoltsToDacCode(mv)));
      g_dac_next_update_us =
          g_dac_phase_start_us + squareEdgeOffsetUs(g_square_edge_index);
      return;
    }

    int32_t until_edge = (int32_t)(g_dac_next_update_us - now_us);

    if (until_edge > 0) {
      if ((uint32_t)until_edge > DAC_SQUARE_EDGE_GUARD_US) {
        return;
      }

      delayMicroseconds((uint32_t)until_edge);
      now_us = micros();
    }

    const uint32_t half_index = squareHalfIndexAt(now_us);
    const bool high = ((half_index & 1U) == 0U);
    const float mv = (float)g_offset_mV +
        (float)g_amplitude_mV * (high ? 1.0f : -1.0f);
    writeDacCodeIfChanged(applyDacNoise(millivoltsToDacCode(mv)));

    g_square_edge_index = half_index + 1U;
    g_dac_next_update_us =
        g_dac_phase_start_us + squareEdgeOffsetUs(g_square_edge_index);
    return;
  }

  if (g_dac_next_update_us != 0 &&
      (int32_t)(now_us - g_dac_next_update_us) < 0) {
    return;
  }

  const float phase = dacPhaseAt(now_us);
  const float mv = (float)g_offset_mV +
      (float)g_amplitude_mV * waveformValue(phase, waveform);
  writeDacCodeIfChanged(applyDacNoise(millivoltsToDacCode(mv)));

  const uint32_t period_us = dacUpdatePeriodUs();
  if (g_dac_next_update_us == 0) {
    g_dac_next_update_us = now_us + period_us;
  } else {
    g_dac_next_update_us += period_us;
    if ((int32_t)(now_us - g_dac_next_update_us) >= (int32_t)period_us) {
      // Wall-clock phase already preserves the requested frequency; after a
      // long delay, resume from now instead of emitting burst catch-up writes.
      g_dac_next_update_us = now_us + period_us;
    }
  }
}

#if UNOQ_ADC_HW_TRIGGER

static inline void stopAdcRegularConversionLocked() {
  if (UNOQ_ADC1_REG(UNOQ_ADC_CR) & UNOQ_ADC_CR_ADSTART) {
    UNOQ_ADC1_REG(UNOQ_ADC_CR) |= UNOQ_ADC_CR_ADSTP;

    const uint32_t t0 = micros();
    while (UNOQ_ADC1_REG(UNOQ_ADC_CR) & UNOQ_ADC_CR_ADSTART) {
      if ((uint32_t)(micros() - t0) > ADC_HW_EOC_TIMEOUT_US) break;
    }
  }
}

static inline void clearAdcFlags() {
  // STM32 ADC ISR flags are cleared by writing 1.
  UNOQ_ADC1_REG(UNOQ_ADC_ISR) =
      UNOQ_ADC_ISR_EOC | UNOQ_ADC_ISR_EOS | UNOQ_ADC_ISR_OVR;
}

static inline bool ensureAdcEnabled() {
  if (UNOQ_ADC1_REG(UNOQ_ADC_CR) & UNOQ_ADC_CR_ADEN) return true;

  // analogRead(A1) is called once before this backend is configured, so the
  // ADC regulator/calibration/device clock are already initialized by Zephyr.
  UNOQ_ADC1_REG(UNOQ_ADC_ISR) = UNOQ_ADC_ISR_ADRDY;
  UNOQ_ADC1_REG(UNOQ_ADC_CR) |= UNOQ_ADC_CR_ADEN;

  const uint32_t t0 = micros();
  while ((UNOQ_ADC1_REG(UNOQ_ADC_ISR) & UNOQ_ADC_ISR_ADRDY) == 0) {
    if ((uint32_t)(micros() - t0) > ADC_HW_EOC_TIMEOUT_US) {
      return false;
    }
  }
  return true;
}

static bool configureAdc1A1HardwareTrigger(uint32_t dma_mode = UNOQ_ADC_DMA_MODE_NONE) {
  stopAdcRegularConversionLocked();

  // Do not use the ADC1 IRQ vector. In fallback mode TIM15 reads DR; in DMA
  // mode GPDMA1 consumes DR directly from the ADC request line.
  UNOQ_ADC1_REG(UNOQ_ADC_IER) &=
      ~(UNOQ_ADC_ISR_EOC | UNOQ_ADC_ISR_EOS | UNOQ_ADC_ISR_OVR);

  uint32_t cfgr1 = UNOQ_ADC1_REG(UNOQ_ADC_CFGR1);
  cfgr1 &= ~(
      UNOQ_ADC_CFGR1_DMNGT_MASK |
      UNOQ_ADC_CFGR1_RES_MASK |
      UNOQ_ADC_CFGR1_EXTSEL_MASK |
      UNOQ_ADC_CFGR1_EXTEN_MASK |
      UNOQ_ADC_CFGR1_CONT |
      UNOQ_ADC_CFGR1_AUTDLY |
      UNOQ_ADC_CFGR1_DISCEN
  );

  // Native 14-bit, single conversion per TIM15 rising TRGO. v18.4 uses
  // DMNGT=01 (limited/one-shot) for the linear GPDMA block and toggles it
  // between blocks. Direct IRQ-read fallback uses DMNGT=00.
  cfgr1 |= (dma_mode & UNOQ_ADC_CFGR1_DMNGT_MASK);
  cfgr1 |=
      UNOQ_ADC_CFGR1_RES_14BIT |
      UNOQ_ADC_CFGR1_EXTSEL_TIM15_TRGO |
      UNOQ_ADC_CFGR1_EXTEN_RISING |
      UNOQ_ADC_CFGR1_OVRMOD;

  UNOQ_ADC1_REG(UNOQ_ADC_CFGR1) = cfgr1;

  // A1 is ADC1 channel 10. Keep the sampling-time settings created by
  // Zephyr's one-time analogRead() initialization; only select channel/rank.
  UNOQ_ADC1_REG(UNOQ_ADC_PCSEL) |= UNOQ_ADC_PCSEL_A1;

  uint32_t sqr1 = UNOQ_ADC1_REG(UNOQ_ADC_SQR1);
  sqr1 &= ~(UNOQ_ADC_SQR1_L_MASK | UNOQ_ADC_SQR1_SQ1_MASK);
  sqr1 |= UNOQ_ADC_SQR1_SQ1_A1;  // L=0 => one conversion, SQ1=CH10
  UNOQ_ADC1_REG(UNOQ_ADC_SQR1) = sqr1;

  clearAdcFlags();
  return ensureAdcEnabled();
}

static inline void armAdcHardwareTriggerLocked() {
  clearAdcFlags();
  if ((UNOQ_ADC1_REG(UNOQ_ADC_CR) & UNOQ_ADC_CR_ADSTART) == 0) {
    // With EXTEN != 0 this arms regular conversions for external TIM15 TRGO.
    UNOQ_ADC1_REG(UNOQ_ADC_CR) |= UNOQ_ADC_CR_ADSTART;
  }
}

static inline void disarmAdcHardwareTriggerLocked() {
  stopAdcRegularConversionLocked();
  clearAdcFlags();
}

static inline void pushAdcSampleFromIsr(uint16_t code) {
  g_adc_buffer[g_adc_write_index] = code & 0x3FFFU;
  g_adc_write_index = (g_adc_write_index + 1U) % ADC_BUFFER_SIZE;
  if (g_adc_count < ADC_BUFFER_SIZE) {
    g_adc_count++;
  }
}

static inline void clearAdcDmaFlags() {
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CFCR) = UNOQ_GPDMA_ALL_FLAGS;
}

static bool stopAdcDmaChannelLocked() {
  // Follow STM32U5 LL disable semantics: request suspend + channel reset.
  // Never reset the whole GPDMA1 peripheral because Zephyr may own other
  // channels. RESET clears the local channel/FIFO state.
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) |=
      (UNOQ_GPDMA_CCR_SUSP | UNOQ_GPDMA_CCR_RESET);

  const uint32_t t0 = micros();
  while (UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) & UNOQ_GPDMA_CCR_EN) {
    if ((uint32_t)(micros() - t0) > 1000U) return false;
  }

  delayMicroseconds(2);
  clearAdcDmaFlags();
  return true;
}

static inline void disableAdcDmaIrqIfConnected() {
  if (g_adc_dma_irq_connect_result >= 0) {
    irq_disable(UNOQ_ADC_DMA_IRQ);
  }
}

static inline void programAdcDmaBlockSamplesLocked(
    uint16_t sample_count,
    bool fill_sentinel,
    bool enable_half_irq) {
  if (sample_count < 1U) sample_count = 1U;
  if (sample_count > ADC_DMA_BLOCK_SAMPLES) sample_count = ADC_DMA_BLOCK_SAMPLES;
  g_adc_dma_programmed_samples = sample_count;

  if (fill_sentinel) {
    for (uint16_t i = 0; i < ADC_DMA_BLOCK_SAMPLES; ++i) {
      g_adc_dma_buffer[i] = 0xFFFFU;
    }
  }

  clearAdcDmaFlags();
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLLR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLBAR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR3) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR2) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR1) =
      UNOQ_GPDMA_CTR1_ADC16_TO_MEM16;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR2) =
      UNOQ_GPDMA_CTR2_ADC1_P2M_SINGLE;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1) =
      (uint32_t)sample_count * sizeof(uint16_t);
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSAR) =
      UNOQ_ADC1_BASE + UNOQ_ADC_DR;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CDAR) =
      (uint32_t)(uintptr_t)&g_adc_dma_buffer[0];

  uint32_t ccr = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR);
  ccr &= ~(UNOQ_GPDMA_CCR_EN | UNOQ_GPDMA_CCR_RESET |
           UNOQ_GPDMA_CCR_SUSP | UNOQ_GPDMA_CCR_HTIE);
  ccr |= UNOQ_GPDMA_CCR_IRQ_MASK;
  if (enable_half_irq && sample_count >= 2U) {
    ccr |= UNOQ_GPDMA_CCR_HTIE;
  }
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) = ccr;
}

static inline void programAdcDmaBlockLocked(bool fill_sentinel) {
  programAdcDmaBlockSamplesLocked(
      ADC_DMA_BLOCK_SAMPLES,
      fill_sentinel,
      false);
}

static inline void captureDmaDebugSnapshot() {
  g_dma_snap_ccr = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR);
  g_dma_snap_ctr1 = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR1);
  g_dma_snap_ctr2 = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR2);
  g_dma_snap_cbr1 = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  g_dma_snap_csar = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSAR);
  g_dma_snap_cdar = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CDAR);
  g_dma_snap_csr = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSR);
  g_dma_snap_adc_cfgr1 = UNOQ_ADC1_REG(UNOQ_ADC_CFGR1);
  g_dma_snap_adc_cr = UNOQ_ADC1_REG(UNOQ_ADC_CR);
  g_dma_snap_adc_isr = UNOQ_ADC1_REG(UNOQ_ADC_ISR);
}

static bool waitDmaPolledComplete(uint32_t timeout_us, uint32_t &status_out) {
  const uint32_t t0 = micros();
  while (true) {
    const uint32_t status = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSR);
    if (status & (UNOQ_GPDMA_FLAG_TCF | UNOQ_GPDMA_ERROR_FLAGS)) {
      status_out = status;
      return (status & UNOQ_GPDMA_FLAG_TCF) != 0U &&
             (status & UNOQ_GPDMA_ERROR_FLAGS) == 0U;
    }
    if ((uint32_t)(micros() - t0) >= timeout_us) {
      status_out = status;
      return false;
    }
  }
}

static bool runGpdmaM2mSelftest() {
  g_dma_test_stage = 2;
  for (uint32_t i = 0; i < 4U; ++i) g_dma_m2m_dst[i] = 0U;

  if (!stopAdcDmaChannelLocked()) {
    captureDmaDebugSnapshot();
    return false;
  }

  clearAdcDmaFlags();
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLLR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLBAR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR3) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR2) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR1) = UNOQ_GPDMA_CTR1_M2M32;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR2) = UNOQ_GPDMA_CTR2_M2M_SW;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1) = 4U * sizeof(uint32_t);
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSAR) =
      (uint32_t)(uintptr_t)&g_dma_m2m_src[0];
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CDAR) =
      (uint32_t)(uintptr_t)&g_dma_m2m_dst[0];
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) = UNOQ_GPDMA_CCR_EN;

  uint32_t status = 0;
  const bool completed = waitDmaPolledComplete(5000U, status);
  g_dma_m2m_status = status;
  g_dma_m2m_dst0 = g_dma_m2m_dst[0];

  bool data_ok = true;
  for (uint32_t i = 0; i < 4U; ++i) {
    if (g_dma_m2m_dst[i] != g_dma_m2m_src[i]) data_ok = false;
  }
  g_dma_m2m_pass = (completed && data_ok) ? 1U : 0U;
  captureDmaDebugSnapshot();
  (void)stopAdcDmaChannelLocked();
  return g_dma_m2m_pass != 0U;
}

static bool runAdcSingleDmaSelftest() {
  g_dma_test_stage = 4;
  g_dma_adc_single_value = 0xFFFFU;
  g_dma_adc_single_sample = 0xFFFFUL;
  g_dma_adc_single_status = 0;
  g_dma_adc_single_pass = 0;

  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();
  if (!stopAdcDmaChannelLocked()) {
    captureDmaDebugSnapshot();
    return false;
  }

  clearAdcDmaFlags();
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLLR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CLBAR) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR3) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR2) = 0;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR1) = UNOQ_GPDMA_CTR1_ADC16_TO_MEM16;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CTR2) = UNOQ_GPDMA_CTR2_ADC1_P2M_SINGLE;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1) = sizeof(uint16_t);
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSAR) = UNOQ_ADC1_BASE + UNOQ_ADC_DR;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CDAR) =
      (uint32_t)(uintptr_t)&g_dma_adc_single_value;
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) = UNOQ_GPDMA_CCR_EN;

  armAdcHardwareTriggerLocked();
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;

  uint32_t status = 0;
  const bool completed = waitDmaPolledComplete(20000U, status);
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  g_dma_adc_single_status = status;
  g_dma_adc_single_sample = g_dma_adc_single_value;
  const uint32_t sample = (uint32_t)g_dma_adc_single_value & 0x3FFFUL;
  const bool data_ok = (g_dma_adc_single_value != 0xFFFFU) &&
                       (sample <= 0x3FFFUL);
  g_dma_adc_single_pass = (completed && data_ok) ? 1U : 0U;
  captureDmaDebugSnapshot();

  disarmAdcHardwareTriggerLocked();
  (void)stopAdcDmaChannelLocked();
  return g_dma_adc_single_pass != 0U;
}

static inline bool startAdcDmaBlockSamplesLocked(
    uint16_t sample_count,
    bool fill_sentinel,
    bool enable_half_irq) {
  if (UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) & UNOQ_GPDMA_CCR_EN) {
    return false;
  }

  programAdcDmaBlockSamplesLocked(sample_count, fill_sentinel, enable_half_irq);
  UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) |= UNOQ_GPDMA_CCR_EN;
  return (UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) & UNOQ_GPDMA_CCR_EN) != 0;
}

static inline bool startAdcDmaBlockLocked(bool fill_sentinel = false) {
  return startAdcDmaBlockSamplesLocked(
      ADC_DMA_BLOCK_SAMPLES,
      fill_sentinel,
      false);
}

static inline void waitExactMicros(uint32_t duration_us) {
  const uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < duration_us) {
    // Deliberately keep interrupts enabled. TIM15 and GPDMA IRQs are the
    // signals being measured; only the foreground thread busy-waits here.
  }
}

// In ADC limited-DMA mode the request generator is one-shot. Reset the DMA
// request state between linear GPDMA blocks while TIM15 is still the physical
// sampling clock. The diagnostic remains source-preserved and can be re-enabled explicitly.
static inline bool rearmAdcDmaLimitedBlockLocked(bool fill_sentinel = false) {
  disarmAdcHardwareTriggerLocked();

  // DMNGT may be changed while ADC is enabled as long as no regular conversion
  // is in progress. Toggle 01 -> 00 -> 01 to restart the limited request state.
  uint32_t cfgr1 = UNOQ_ADC1_REG(UNOQ_ADC_CFGR1);
  cfgr1 &= ~UNOQ_ADC_CFGR1_DMNGT_MASK;
  UNOQ_ADC1_REG(UNOQ_ADC_CFGR1) = cfgr1;
  cfgr1 |= UNOQ_ADC_DMA_MODE_LIMITED;
  UNOQ_ADC1_REG(UNOQ_ADC_CFGR1) = cfgr1;

  clearAdcFlags();
  if (!startAdcDmaBlockLocked(fill_sentinel)) {
    return false;
  }
  armAdcHardwareTriggerLocked();
  return true;
}

static void adcDmaIsr(const void *arg) {
  (void)arg;

  const uint32_t service_begin_us = micros();
  const uint32_t status = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSR);
  const uint32_t cbr1 = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  const bool tcf_complete = (status & UNOQ_GPDMA_FLAG_TCF) != 0U;
  // On the UNO Q/Zephyr path the DMA IRQ can be delivered after GPDMA has
  // already returned to IDLE and TCF is no longer visible in CSR. A zero
  // remaining block count proves the programmed block was fully consumed.
  const bool idle_complete =
      !tcf_complete &&
      ((status & UNOQ_GPDMA_FLAG_IDLEF) != 0U) &&
      (cbr1 == 0U);
  const bool block_complete = tcf_complete || idle_complete;

  g_adc_dma_last_status = status;
  g_dma_irq_count++;
  if (status & UNOQ_GPDMA_FLAG_HTF) g_dma_half_count++;
  if (tcf_complete) g_dma_complete_count++;
  if (idle_complete) g_dma_idle_complete_count++;

  const uint32_t error_flags = status & UNOQ_GPDMA_ERROR_FLAGS;
  if (error_flags != 0U) {
    g_adc_dma_error_count++;
  }

  if (UNOQ_ADC1_REG(UNOQ_ADC_ISR) & UNOQ_ADC_ISR_OVR) {
    g_adc_overrun_count++;
    UNOQ_ADC1_REG(UNOQ_ADC_ISR) = UNOQ_ADC_ISR_OVR;
  }

  if (block_complete) {
    bool observed_data = false;
    const uint16_t sample_count = g_adc_dma_programmed_samples;

    for (uint16_t i = 0; i < sample_count; ++i) {
      const uint16_t raw = g_adc_dma_buffer[i];
      if (raw != 0xFFFFU) observed_data = true;

      const uint16_t code = (uint16_t)(raw & 0x3FFFUL);
      g_adc_last_code = code;
      if (!g_adc_dma_selftest_mode) {
        pushAdcSampleFromIsr(code);
      }
    }

    if (observed_data) {
      g_adc_dma_observed_data = true;
    }

    g_adc_dma_block_count++;
    g_adc_dma_transfer_count += sample_count;
    g_adc_hw_conversion_count += sample_count;

    if (!g_adc_dma_selftest_mode) {
      g_adc_latest_counter =
          g_adc_dma_run_start_counter + g_adc_hw_conversion_count;
    }
  }

  clearAdcDmaFlags();

  // Re-arm only after a complete block. Completion may be reported either
  // with TCF or, on UNO Q/Zephyr, as IDLEF with CBR1 already at zero.
  if (block_complete &&
      !g_adc_dma_diag_no_rearm &&
      (g_adc_running || g_adc_dma_selftest_mode) &&
      error_flags == 0U) {
    if (!rearmAdcDmaLimitedBlockLocked(false)) {
      g_adc_dma_error_count++;
    }
  }

  const uint32_t elapsed_us = micros() - service_begin_us;
  g_adc_dma_service_last_us = elapsed_us;
  if (elapsed_us > g_adc_dma_service_max_us) {
    g_adc_dma_service_max_us = elapsed_us;
  }
}

static bool runAdcBlockDmaOneShotDiagnostic(
    uint16_t sample_count,
    uint32_t &status_out,
    uint32_t &elapsed_out,
    uint32_t &tim15_irq_out,
    uint32_t &cbr1_end_out,
    uint32_t &requests_out) {
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();
  if (!stopAdcDmaChannelLocked()) return false;

  if (!configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_LIMITED)) {
    return false;
  }

  g_adc_dma_selftest_mode = true;
  g_adc_selftest_mode = true;
  g_adc_dma_diag_no_rearm = true;
  g_adc_dma_observed_data = false;
  g_adc_dma_transfer_count = 0;
  g_adc_dma_block_count = 0;
  g_adc_dma_error_count = 0;
  g_adc_overrun_count = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_timer_irq_count = 0;
  g_dma_irq_count = 0;
  g_dma_half_count = 0;
  g_dma_complete_count = 0;
  g_dma_idle_complete_count = 0;
  g_adc_dma_last_status = 0;

  // Match the final runtime path: no half-transfer IRQ, only block completion.
  if (!startAdcDmaBlockSamplesLocked(sample_count, true, false)) {
    g_adc_dma_diag_no_rearm = false;
    g_adc_dma_selftest_mode = false;
    g_adc_selftest_mode = false;
    return false;
  }

  const uint32_t cbr1_start = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  armAdcHardwareTriggerLocked();
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  const uint32_t t0 = micros();
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
  waitExactMicros(ADC_DMA_SELFTEST_US);
  elapsed_out = (uint32_t)(micros() - t0);

  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();

  const uint32_t current_status = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CSR);
  status_out = g_adc_dma_last_status != 0U ? g_adc_dma_last_status : current_status;
  cbr1_end_out = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  tim15_irq_out = g_adc_timer_irq_count;
  const uint32_t transferred_bytes =
      (cbr1_start >= cbr1_end_out) ? (cbr1_start - cbr1_end_out) : 0U;
  requests_out = transferred_bytes / sizeof(uint16_t);

  captureDmaDebugSnapshot();
  const bool pass =
      (g_dma_complete_count + g_dma_idle_complete_count) >= 1U &&
      cbr1_end_out == 0U &&
      g_adc_dma_observed_data &&
      g_adc_dma_error_count == 0U &&
      g_adc_overrun_count == 0U;

  (void)stopAdcDmaChannelLocked();
  g_adc_dma_diag_no_rearm = false;
  g_adc_dma_selftest_mode = false;
  g_adc_selftest_mode = false;
  return pass;
}

static bool setupAdcDmaUpgrade() {
  g_adc_dma_active = false;
  g_adc_dma_selftest_mode = false;
  g_adc_dma_observed_data = false;
  g_adc_dma_diag = 0;
  g_adc_dma_irq_connect_result = -999;
  g_adc_dma_transfer_count = 0;
  g_adc_dma_block_count = 0;
  g_adc_dma_error_count = 0;
  g_adc_overrun_count = 0;
  g_adc_dma_service_last_us = 0;
  g_adc_dma_service_max_us = 0;
  g_adc_dma_last_status = 0;
  g_dma_test_stage = 0;
  g_dma_m2m_pass = 0;
  g_dma_m2m_status = 0;
  g_dma_m2m_dst0 = 0;
  g_dma_adc_single_pass = 0;
  g_dma_adc_single_status = 0;
  g_dma_adc_single_sample = 0xFFFFUL;
  g_dma_adc_block_pass = 0;
  g_dma_snap_ccr = g_dma_snap_ctr1 = g_dma_snap_ctr2 = 0;
  g_dma_snap_cbr1 = g_dma_snap_csar = g_dma_snap_cdar = 0;
  g_dma_snap_csr = 0;
  g_dma_snap_adc_cfgr1 = g_dma_snap_adc_cr = g_dma_snap_adc_isr = 0;
  g_adc_dma_programmed_samples = ADC_DMA_BLOCK_SAMPLES;
  g_adc_dma_diag_no_rearm = false;
  g_dma_block_elapsed_us = 0;
  g_dma_block_tim15_irq_count = 0;
  g_dma_block_adc_requests_observed = 0;
  g_dma_block_cbr1_start = 0;
  g_dma_block_cbr1_end = 0;
  g_dma_irq_count = g_dma_half_count = g_dma_complete_count = 0;
  g_dma_idle_complete_count = 0;
  g_dma_adc_4sample_pass = 0;
  g_dma_adc_4sample_status = 0;
  g_dma_adc_4sample_elapsed_us = 0;
  g_dma_adc_4sample_tim15_irq_count = 0;
  g_dma_adc_4sample_cbr1_end = 0;
  g_dma_adc_4sample_requests = 0;
  g_dma_adc_8sample_pass = 0;
  g_dma_adc_8sample_status = 0;
  g_dma_adc_8sample_elapsed_us = 0;
  g_dma_adc_8sample_tim15_irq_count = 0;
  g_dma_adc_8sample_cbr1_end = 0;
  g_dma_adc_8sample_requests = 0;

  g_adc_dma_diag |= (1UL << 0); // UNO Q
  g_dma_test_stage = 1;

  // Stage 1: clock + channel claim.
  UNOQ_RCC_REG(UNOQ_RCC_AHB1ENR) |= UNOQ_RCC_GPDMA1_BIT;
  (void)UNOQ_RCC_REG(UNOQ_RCC_AHB1ENR);
  if ((UNOQ_RCC_REG(UNOQ_RCC_AHB1ENR) & UNOQ_RCC_GPDMA1_BIT) == 0U) {
    return false;
  }
  g_adc_dma_diag |= (1UL << 1);

  if (UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CCR) & UNOQ_GPDMA_CCR_EN) {
    captureDmaDebugSnapshot();
    return false;
  }
  g_adc_dma_diag |= (1UL << 2);
  if (!stopAdcDmaChannelLocked()) {
    captureDmaDebugSnapshot();
    return false;
  }

  // Stage 2: prove the GPDMA engine and SRAM path without ADC/TIM15/IRQ.
  if (!runGpdmaM2mSelftest()) {
    return false;
  }
  g_adc_dma_diag |= (1UL << 3);

  // Stage 3: connect the DMA IRQ only after the engine itself is proven.
  g_dma_test_stage = 3;
  const int vector = irq_connect_dynamic(
      UNOQ_ADC_DMA_IRQ,
      UNOQ_ADC_DMA_IRQ_PRIORITY,
      adcDmaIsr,
      NULL,
      0
  );
  g_adc_dma_irq_connect_result = vector;
  if (vector < 0) {
    return false;
  }
  irq_enable(UNOQ_ADC_DMA_IRQ);
  if (!irq_is_enabled(UNOQ_ADC_DMA_IRQ)) {
    disableAdcDmaIrqIfConnected();
    return false;
  }
  g_adc_dma_diag |= (1UL << 4);

  // Switch ADC from the proven direct-DR mode to limited DMA request mode.
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();

  if (!configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_LIMITED)) {
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    captureDmaDebugSnapshot();
    return false;
  }

  // Stage 4: one conversion, one 16-bit ADC_DR DMA transfer, polled completion.
  // This isolates the ADC request handshake from block IRQ/re-arm logic.
  irq_disable(UNOQ_ADC_DMA_IRQ);
  if (!runAdcSingleDmaSelftest()) {
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }
  g_adc_dma_diag |= (1UL << 5);
  irq_enable(UNOQ_ADC_DMA_IRQ);

  // Stage 5A/5B: compare one-shot 4-sample and 8-sample ADC DMA blocks.
  // Each diagnostic test runs for a measured 100 ms at the configured TIM15 rate.
  // HTIE stays disabled; completion accepts TCF or IDLEF + CBR1==0 evidence.
  g_dma_test_stage = 5;

  uint32_t diag_status = 0;
  uint32_t diag_elapsed = 0;
  uint32_t diag_tim15 = 0;
  uint32_t diag_cbr1_end = 0;
  uint32_t diag_requests = 0;

  const bool four_ok = runAdcBlockDmaOneShotDiagnostic(
      4U,
      diag_status,
      diag_elapsed,
      diag_tim15,
      diag_cbr1_end,
      diag_requests);
  g_dma_adc_4sample_pass = four_ok ? 1U : 0U;
  g_dma_adc_4sample_status = diag_status;
  g_dma_adc_4sample_elapsed_us = diag_elapsed;
  g_dma_adc_4sample_tim15_irq_count = diag_tim15;
  g_dma_adc_4sample_cbr1_end = diag_cbr1_end;
  g_dma_adc_4sample_requests = diag_requests;

  diag_status = diag_elapsed = diag_tim15 = diag_cbr1_end = diag_requests = 0;
  const bool eight_ok = runAdcBlockDmaOneShotDiagnostic(
      ADC_DMA_BLOCK_SAMPLES,
      diag_status,
      diag_elapsed,
      diag_tim15,
      diag_cbr1_end,
      diag_requests);
  g_dma_adc_8sample_pass = eight_ok ? 1U : 0U;
  g_dma_adc_8sample_status = diag_status;
  g_dma_adc_8sample_elapsed_us = diag_elapsed;
  g_dma_adc_8sample_tim15_irq_count = diag_tim15;
  g_dma_adc_8sample_cbr1_end = diag_cbr1_end;
  g_dma_adc_8sample_requests = diag_requests;

  // Preserve the 8-sample one-shot evidence in the generic block fields even
  // if the upgrade falls back here.
  g_dma_block_elapsed_us = diag_elapsed;
  g_dma_block_tim15_irq_count = diag_tim15;
  g_dma_block_adc_requests_observed = diag_requests;
  g_dma_block_cbr1_start =
      (uint32_t)ADC_DMA_BLOCK_SAMPLES * sizeof(uint16_t);
  g_dma_block_cbr1_end = diag_cbr1_end;

  if (!four_ok || !eight_ok) {
    g_dma_adc_block_pass = 0U;
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }

  // Stage 5C: only after both one-shot sizes pass, prove the existing 8-sample
  // IRQ + limited-mode re-arm runtime bridge for at least two complete blocks.
  if (!configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_LIMITED)) {
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }

  g_adc_dma_selftest_mode = true;
  g_adc_selftest_mode = true;
  g_adc_dma_diag_no_rearm = false;
  g_adc_dma_observed_data = false;
  g_adc_dma_transfer_count = 0;
  g_adc_dma_block_count = 0;
  g_adc_dma_error_count = 0;
  g_adc_overrun_count = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_timer_irq_count = 0;
  g_dma_irq_count = g_dma_half_count = g_dma_complete_count = 0;
  g_dma_idle_complete_count = 0;

  if (!startAdcDmaBlockLocked(true)) {
    g_adc_dma_selftest_mode = false;
    g_adc_selftest_mode = false;
    captureDmaDebugSnapshot();
    (void)stopAdcDmaChannelLocked();
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }

  g_dma_block_cbr1_start = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  armAdcHardwareTriggerLocked();
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  const uint32_t block_t0 = micros();
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
  waitExactMicros(ADC_DMA_SELFTEST_US);
  g_dma_block_elapsed_us = (uint32_t)(micros() - block_t0);

  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();
  g_dma_block_tim15_irq_count = g_adc_timer_irq_count;
  g_dma_block_cbr1_end = UNOQ_GPDMA1_CH_REG(UNOQ_GPDMA_CBR1);
  const uint32_t current_block_bytes =
      (g_dma_block_cbr1_start >= g_dma_block_cbr1_end)
          ? (g_dma_block_cbr1_start - g_dma_block_cbr1_end)
          : 0U;
  g_dma_block_adc_requests_observed =
      g_adc_dma_transfer_count + current_block_bytes / sizeof(uint16_t);
  captureDmaDebugSnapshot();
  (void)stopAdcDmaChannelLocked();
  g_adc_dma_selftest_mode = false;
  g_adc_selftest_mode = false;

  const bool block_ok =
      g_adc_dma_block_count >= 2U &&
      g_adc_dma_transfer_count >= (uint32_t)(2U * ADC_DMA_BLOCK_SAMPLES) &&
      g_adc_dma_observed_data &&
      g_adc_dma_error_count == 0U &&
      g_adc_overrun_count == 0U;
  g_dma_adc_block_pass = block_ok ? 1U : 0U;

  if (!block_ok) {
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }
  g_adc_dma_diag |= (1UL << 6);

  if (!configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_LIMITED)) {
    disableAdcDmaIrqIfConnected();
    (void)configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE);
    return false;
  }

  g_dma_test_stage = 6;
  g_adc_dma_active = true;
  g_adc_dma_diag |= (1UL << 7);
  g_adc_dma_transfer_count = 0;
  g_adc_dma_block_count = 0;
  g_adc_dma_error_count = 0;
  g_adc_overrun_count = 0;
  g_adc_dma_service_last_us = 0;
  g_adc_dma_service_max_us = 0;
  g_adc_dma_last_status = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_latest_counter = g_sample_counter;
  return true;
}

static void adcTimerIsr(const void *arg) {
  (void)arg;

  // The update event already generated TIM15 TRGO and therefore started ADC1.
  // CPU timing only affects when we retrieve the result, not the sample instant.
  UNOQ_TIM15_REG(UNOQ_TIM_SR) &= ~UNOQ_TIM_SR_UIF;

  const uint32_t irq_latency_us = UNOQ_TIM15_REG(UNOQ_TIM_CNT);
  g_adc_timer_irq_count++;

  if (!g_adc_selftest_mode) {
    g_sample_counter++;

    if (irq_latency_us > g_scheduler_late_max_us) {
      g_scheduler_late_max_us = irq_latency_us;
    }
  }

  if (!(g_adc_running || g_adc_selftest_mode)) {
    return;
  }

  // DMA remains implemented/validated in source, but v19.0 defaults to immediate
  // TIM15 IRQ-read for lower display latency. If DMA is explicitly re-enabled
  // in a future build, this guard hands ADC result retrieval back to GPDMA1.
  if (g_adc_dma_active || g_adc_dma_selftest_mode) {
    return;
  }

  bool got_sample = false;
  uint32_t elapsed_us = UNOQ_TIM15_REG(UNOQ_TIM_CNT);

  while (true) {
    const uint32_t adc_isr = UNOQ_ADC1_REG(UNOQ_ADC_ISR);
    if (adc_isr & UNOQ_ADC_ISR_EOC) {
      const uint16_t code =
          (uint16_t)(UNOQ_ADC1_REG(UNOQ_ADC_DR) & 0x3FFFUL);
      g_adc_last_code = code;
      g_adc_hw_conversion_count++;

      elapsed_us = UNOQ_TIM15_REG(UNOQ_TIM_CNT);
      g_adc_read_last_us = elapsed_us;
      if (elapsed_us > g_adc_read_max_us) {
        g_adc_read_max_us = elapsed_us;
      }

      clearAdcFlags();

      if (!g_adc_selftest_mode) {
        pushAdcSampleFromIsr(code);
      }
      got_sample = true;
      break;
    }

    elapsed_us = UNOQ_TIM15_REG(UNOQ_TIM_CNT);
    if (elapsed_us >= ADC_HW_EOC_TIMEOUT_US) {
      break;
    }
  }

  if (!got_sample) {
    g_adc_conversion_timeouts++;

    // Preserve one sample per hardware timebase tick so the existing
    // incremental counter/ring protocol remains exact. The repeated value is
    // explicitly diagnosable through adc_conversion_timeouts.
    if (!g_adc_selftest_mode) {
      pushAdcSampleFromIsr(g_adc_last_code);
    }
  }
}

static void disableAdcHardwareTiming() {
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  disarmAdcHardwareTriggerLocked();

  g_adc_hw_timer_active = false;
  g_adc_timer_rate_hz = 0;
  g_adc_timer_counter_hz = 0;
}

static bool setupAdcHardwareTiming() {
  g_adc_hw_timer_active = false;
  g_adc_timer_rate_hz = 0;
  g_adc_timer_counter_hz = 0;
  g_adc_timer_diag = 0;
  g_adc_irq_connect_result = -999;
  g_adc_timer_irq_count = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_conversion_timeouts = 0;
  g_adc_selftest_mode = false;
  g_adc_dma_active = false;
  g_adc_dma_selftest_mode = false;
  g_adc_latest_counter = 0;

  g_adc_timer_diag |= (1UL << 0); // UNO Q
#if UNOQ_HAS_ZEPHYR_IRQ_HEADER
  g_adc_timer_diag |= (1UL << 1);
#endif
#if UNOQ_HAS_DYNAMIC_IRQ
  g_adc_timer_diag |= (1UL << 2);
#endif

  // Enable/reset TIM15 through RCC APB2.
  UNOQ_RCC_REG(UNOQ_RCC_APB2ENR) |= UNOQ_RCC_TIM15_BIT;
  (void)UNOQ_RCC_REG(UNOQ_RCC_APB2ENR);

  if ((UNOQ_RCC_REG(UNOQ_RCC_APB2ENR) & UNOQ_RCC_TIM15_BIT) == 0) {
    return false;
  }
  g_adc_timer_diag |= (1UL << 3);

  UNOQ_RCC_REG(UNOQ_RCC_APB2RSTR) |= UNOQ_RCC_TIM15_BIT;
  UNOQ_RCC_REG(UNOQ_RCC_APB2RSTR) &= ~UNOQ_RCC_TIM15_BIT;

  uint32_t timer_clock_hz = SystemCoreClock;
  if (timer_clock_hz == 0) timer_clock_hz = 160000000UL;

  uint32_t prescaler_div =
      (timer_clock_hz + ADC_HW_TIMER_COUNTER_TARGET_HZ / 2U) /
      ADC_HW_TIMER_COUNTER_TARGET_HZ;
  if (prescaler_div < 1U) prescaler_div = 1U;
  if (prescaler_div > 65536U) prescaler_div = 65536U;

  const uint32_t counter_hz = timer_clock_hz / prescaler_div;

  uint32_t period_counts =
      (counter_hz + g_adc_requested_rate_hz / 2U) /
      g_adc_requested_rate_hz;
  if (period_counts < 2U) period_counts = 2U;
  if (period_counts > 65536U) period_counts = 65536U;

  const uint32_t actual_rate_hz = counter_hz / period_counts;

  // Configure TIM15. MMS=update makes every timer update a hardware ADC trigger.
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CR2) =
      (UNOQ_TIM15_REG(UNOQ_TIM_CR2) & ~UNOQ_TIM_CR2_MMS_MASK) |
      UNOQ_TIM_CR2_MMS_UPDATE;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_PSC) = prescaler_div - 1U;
  UNOQ_TIM15_REG(UNOQ_TIM_ARR) = period_counts - 1U;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_EGR) = UNOQ_TIM_EGR_UG;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  // Prove TIM15 itself counts before connecting the IRQ.
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
  const uint32_t c0 = UNOQ_TIM15_REG(UNOQ_TIM_CNT);
  delayMicroseconds(30);
  const uint32_t c1 = UNOQ_TIM15_REG(UNOQ_TIM_CNT);
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  if (c0 == c1) {
    disableAdcHardwareTiming();
    return false;
  }
  g_adc_timer_diag |= (1UL << 4);

  const int vector = irq_connect_dynamic(
      UNOQ_ADC_TIMER_IRQ,
      UNOQ_ADC_TIMER_IRQ_PRIORITY,
      adcTimerIsr,
      NULL,
      0
  );
  g_adc_irq_connect_result = vector;

  if (vector < 0) {
    disableAdcHardwareTiming();
    return false;
  }
  g_adc_timer_diag |= (1UL << 5);

  irq_enable(UNOQ_ADC_TIMER_IRQ);
  if (!irq_is_enabled(UNOQ_ADC_TIMER_IRQ)) {
    disableAdcHardwareTiming();
    return false;
  }
  g_adc_timer_diag |= (1UL << 6);

  if (!configureAdc1A1HardwareTrigger()) {
    disableAdcHardwareTiming();
    return false;
  }

  // End-to-end self-test:
  // TIM15 update -> TRGO -> ADC1 CH10 14-bit conversion -> TIM15 ISR -> DR.
  g_adc_selftest_mode = true;
  g_adc_hw_conversion_count = 0;
  g_adc_conversion_timeouts = 0;
  g_adc_timer_irq_count = 0;
  g_adc_read_last_us = 0;
  g_adc_read_max_us = 0;

  armAdcHardwareTriggerLocked();

  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;

  // A few timer periods at the configured startup rate.
  delayMicroseconds((3000000UL / (actual_rate_hz > 0U ? actual_rate_hz : 1U)) + 1000UL);

  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  disarmAdcHardwareTriggerLocked();
  g_adc_selftest_mode = false;

  if (g_adc_timer_irq_count < 2U ||
      g_adc_hw_conversion_count < 2U ||
      g_adc_conversion_timeouts != 0U) {
    disableAdcHardwareTiming();

    // Restore the Arduino/Zephyr software ADC path as a safe fallback.
    (void)analogRead(A1);
    return false;
  }

  g_adc_timer_diag |= (1UL << 7);
  g_adc_baseline_eoc_count = g_adc_hw_conversion_count;

  // Only after the complete trigger/conversion self-test succeeds do we
  // advertise hardware timing.
  g_adc_timer_counter_hz = counter_hz;
  g_adc_timer_rate_hz = actual_rate_hz;
  g_actual_sample_rate_mHz = actual_rate_hz * 1000UL;
  g_adc_hw_timer_active = true;

  // Reset user-visible acquisition state after the self-test.
  g_adc_timer_irq_count = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_conversion_timeouts = 0;
  g_adc_read_last_us = 0;
  g_adc_read_max_us = 0;
  g_scheduler_late_max_us = 0;
  g_sample_counter = 0;
  g_adc_write_index = 0;
  g_adc_count = 0;
  g_adc_latest_counter = 0;

  // v19.0 production path: do not touch GPDMA/IRQ29 during boot.
  // v18.5 already validated the full DMA implementation; the code is retained
  // below behind a compile-time flag for future higher-rate experiments.
#if UNOQ_ADC_DMA_RUNTIME_ENABLED
  const bool dma_validated = setupAdcDmaUpgrade();
  if (dma_validated) {
    disarmAdcHardwareTriggerLocked();
    (void)stopAdcDmaChannelLocked();
    disableAdcDmaIrqIfConnected();
    g_adc_dma_active = false;
    g_adc_dma_selftest_mode = false;
    if (!configureAdc1A1HardwareTrigger(UNOQ_ADC_DMA_MODE_NONE)) {
      disableAdcHardwareTiming();
      return false;
    }
  }
#else
  g_adc_dma_active = false;
  g_adc_dma_selftest_mode = false;
  g_adc_dma_diag = 0;
  g_adc_dma_irq_connect_result = -999;
  g_dma_test_stage = 0;
#endif

  // Reset user-visible timing/ring state after hardware initialization.
  g_adc_timer_irq_count = 0;
  g_adc_hw_conversion_count = 0;
  g_adc_conversion_timeouts = 0;
  g_adc_read_last_us = 0;
  g_adc_read_max_us = 0;
  g_scheduler_late_max_us = 0;
  g_sample_counter = 0;
  g_adc_write_index = 0;
  g_adc_count = 0;
  g_adc_latest_counter = 0;

  // TIM15 remains running even while ADC is STOPPED, preserving the existing
  // continuously advancing acquisition counter/phase-origin protocol.
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;

  return true;
}

#endif  // UNOQ_ADC_HW_TRIGGER

// Software fallback only. v18.4 normal UNO Q operation uses TIM15 TRGO.
// This path remains available if the hardware self-test cannot be completed.
uint16_t readA1Fast() {
  int raw = analogRead(A1);
  if (raw < 0) raw = 0;
  if (raw > 16383) raw = 16383;
  return (uint16_t)raw;
}

void serviceAcquisitionClock(uint32_t now_us) {
#if UNOQ_ADC_HW_TRIGGER
  if (g_adc_hw_timer_active) {
    // TIM15 owns the acquisition timebase. The ADC conversion starts from
    // hardware TRGO; loop() is completely outside the sample-start path.
    (void)now_us;
    return;
  }
#endif
  if (g_next_sample_us == 0) {
    g_next_sample_us = now_us;
  }

  if ((int32_t)(now_us - g_next_sample_us) < 0) return;

  // Track worst scheduler lateness against the selected software-fallback
  // sample period.
  const int32_t late_us = (int32_t)(now_us - g_next_sample_us);
  if (late_us > 0 && (uint32_t)late_us > g_scheduler_late_max_us) {
    g_scheduler_late_max_us = (uint32_t)late_us;
  }

  // Start timestamp is the physical timing reference for the current sample.
  const uint32_t sample_start_us = micros();

  if (g_adc_running) {
    const uint32_t adc_begin_us = micros();
    const uint16_t adc_code = readA1Fast();
    const uint32_t adc_elapsed_us = micros() - adc_begin_us;
    g_adc_read_last_us = adc_elapsed_us;
    if (adc_elapsed_us > g_adc_read_max_us) {
      g_adc_read_max_us = adc_elapsed_us;
    }
    g_adc_buffer[g_adc_write_index] = adc_code;
    g_adc_write_index = (g_adc_write_index + 1) % ADC_BUFFER_SIZE;
    if (g_adc_count < ADC_BUFFER_SIZE) {
      g_adc_count++;
    }
  }

  g_sample_counter++;

  if (g_rate_start_us == 0) {
    g_rate_start_us = sample_start_us;
    g_rate_start_counter = g_sample_counter;
  } else {
    const uint32_t tick_delta = g_sample_counter - g_rate_start_counter;
    if (tick_delta >= RATE_MEASURE_TICKS) {
      const uint32_t elapsed_us = sample_start_us - g_rate_start_us;
      if (elapsed_us > 0) {
        const uint64_t measured_mHz =
            ((uint64_t)tick_delta * 1000000000ULL) /
            (uint64_t)elapsed_us;
        if (measured_mHz >= 100000ULL && measured_mHz <= 5000000ULL) {
          const uint32_t measured = (uint32_t)measured_mHz;

          // 1/2 old + 1/2 new: enough smoothing for a stable timebase readout
          // while still following a real sustained rate change.
          g_actual_sample_rate_mHz =
              (g_actual_sample_rate_mHz + measured) / 2U;
        }
      }
      g_rate_start_us = sample_start_us;
      g_rate_start_counter = g_sample_counter;
    }
  }

  // Selected cadence, but never synthesize back-to-back ADC samples.
  const uint32_t sample_period_us = g_software_sample_period_us > 0U ? g_software_sample_period_us : 1U;
  g_next_sample_us += sample_period_us;
  const uint32_t after_us = micros();
  if ((int32_t)(after_us - g_next_sample_us) >= (int32_t)sample_period_us) {
    g_next_sample_us = after_us + sample_period_us;
  }
}

uint32_t set_signal(int waveform, int frequency_mHz, int amplitude_mV, int offset_mV) {
#if UNOQ_DAC_HW_TIMER
  const unsigned int key = irq_lock();
#endif

  g_waveform = clampInt(waveform, 0, 3);
  {
    const int clamped_frequency_mHz = clampInt(frequency_mHz, 1000, 100000);
    g_frequency_mHz = ((clamped_frequency_mHz + 500) / 1000) * 1000;
  }
  g_amplitude_mV = clampInt(amplitude_mV, 0, 1650);
  g_offset_mV = clampInt(offset_mV, 0, 3300);
  updateNoiseDerivedStateLocked();

  g_phase_origin = g_sample_counter;
  g_dac_phase_start_us = micros();
  g_dac_next_update_us = 0;
  g_square_edge_index = 0;
  g_dac_last_code = 0xFFFF;

#if UNOQ_DAC_HW_TIMER
  if (g_dac_hw_timer_active) {
    updateDacDerivedStateLocked();
    g_dac_phase_accum = 0;
  }
  irq_unlock(key);
#endif

  return g_phase_origin;
}

String set_noise(int enabled, int level_percent) {
#if UNOQ_DAC_HW_TIMER
  const unsigned int key = irq_lock();
#endif

  g_noise_enabled = (enabled != 0);
  g_noise_level_percent = clampInt(level_percent, 0, 100);
  updateNoiseDerivedStateLocked();

  // Do NOT touch DDS phase/origin. Noise control is deliberately independent
  // of set_signal(), so toggling or dragging LEVEL cannot restart the waveform.
  g_dac_last_code = 0xFFFF;

#if UNOQ_DAC_HW_TIMER
  irq_unlock(key);
#endif

  return String(g_noise_enabled ? 1 : 0) + "," + String(g_noise_level_percent);
}

uint32_t set_dac_run(int running) {
#if UNOQ_DAC_HW_TIMER
  const unsigned int key = irq_lock();
#endif

  g_dac_running = (running != 0);
  g_phase_origin = g_sample_counter;
  g_dac_phase_start_us = micros();
  g_dac_next_update_us = 0;
  g_square_edge_index = 0;
  g_dac_last_code = 0xFFFF;

#if UNOQ_DAC_HW_TIMER
  if (g_dac_hw_timer_active) {
    updateDacDerivedStateLocked();

    if (g_dac_running) {
      startDacHardwareTimerLocked();
    } else {
      stopDacHardwareTimerLocked();
      writeDacHardware(0);
      g_dac_last_code = 0;
    }
  } else if (!g_dac_running) {
    writeDacHardware(0);
    g_dac_last_code = 0;
  }

  irq_unlock(key);
#else
  if (!g_dac_running) {
    writeDacHardware(0);
    g_dac_last_code = 0;
  }
#endif

  return g_phase_origin;
}

static uint32_t normalizeAdcSampleRate(uint32_t requested_hz) {
  // v19.11 exposes a single production rate. Keep the RPC for backward
  // compatibility, but every request resolves to the fixed hardware rate.
  (void)requested_hz;
  return ADC_HW_TIMER_DEFAULT_HZ;
}

#if UNOQ_ADC_HW_TRIGGER
static uint32_t applyAdcHardwareRateLocked(uint32_t requested_hz) {
  const uint32_t target_hz = normalizeAdcSampleRate(requested_hz);
  const uint32_t counter_hz = g_adc_timer_counter_hz;
  if (!g_adc_hw_timer_active || counter_hz == 0U) {
    return g_adc_timer_rate_hz;
  }

  const bool was_running = g_adc_running;
  g_adc_running = false;
  disarmAdcHardwareTriggerLocked();
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) &= ~UNOQ_TIM_CR1_CEN;
  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = 0;

  uint32_t period_counts = (counter_hz + target_hz / 2U) / target_hz;
  if (period_counts < 2U) period_counts = 2U;
  if (period_counts > 65536U) period_counts = 65536U;

  UNOQ_TIM15_REG(UNOQ_TIM_ARR) = period_counts - 1U;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_EGR) = UNOQ_TIM_EGR_UG;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;

  const uint32_t actual_hz = counter_hz / period_counts;
  g_adc_requested_rate_hz = target_hz;
  g_adc_timer_rate_hz = actual_hz;
  g_actual_sample_rate_mHz = actual_hz * 1000UL;
  g_software_sample_period_us = actual_hz > 0U ? (1000000UL / actual_hz) : 5000UL;

  g_adc_write_index = 0;
  g_adc_count = 0;
  g_adc_read_last_us = 0;
  g_adc_read_max_us = 0;
  g_scheduler_late_max_us = 0;
  g_adc_conversion_timeouts = 0;
  g_adc_hw_conversion_count = 0;
  g_phase_origin = g_sample_counter;

  if (was_running) {
    clearAdcFlags();
    armAdcHardwareTriggerLocked();
    g_adc_running = true;
  }

  UNOQ_TIM15_REG(UNOQ_TIM_DIER) = UNOQ_TIM_DIER_UIE;
  UNOQ_TIM15_REG(UNOQ_TIM_CNT) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_SR) = 0;
  UNOQ_TIM15_REG(UNOQ_TIM_CR1) |= UNOQ_TIM_CR1_CEN;
  return actual_hz;
}
#endif

String set_adc_sample_rate(int requested_hz) {
  const uint32_t target_hz = normalizeAdcSampleRate(
      requested_hz > 0 ? (uint32_t)requested_hz : ADC_HW_TIMER_DEFAULT_HZ);
  uint32_t actual_hz = target_hz;

#if UNOQ_ADC_HW_TRIGGER
  if (g_adc_hw_timer_active) {
    const unsigned int key = irq_lock();
    actual_hz = applyAdcHardwareRateLocked(target_hz);
    const uint32_t counter = g_sample_counter;
    irq_unlock(key);
    return String(actual_hz) + "," + String(counter);
  }
#endif

  g_adc_requested_rate_hz = target_hz;
  g_actual_sample_rate_mHz = target_hz * 1000UL;
  g_software_sample_period_us = target_hz > 0U ? (1000000UL / target_hz) : 5000UL;
  g_phase_origin = g_sample_counter;
  g_adc_write_index = 0;
  g_adc_count = 0;
  return String(target_hz) + "," + String(g_sample_counter);
}

uint32_t set_adc_run(int running) {
#if UNOQ_ADC_HW_TRIGGER
  if (g_adc_hw_timer_active) {
    const unsigned int key = irq_lock();

    g_adc_running = false;
    g_adc_write_index = 0;
    g_adc_count = 0;
    g_adc_read_last_us = 0;
    g_adc_read_max_us = 0;
    g_scheduler_late_max_us = 0;
    g_adc_conversion_timeouts = 0;
    g_adc_hw_conversion_count = 0;
    g_actual_sample_rate_mHz = g_adc_timer_rate_hz * 1000UL;

    if (g_adc_dma_active) {
      disarmAdcHardwareTriggerLocked();
      (void)stopAdcDmaChannelLocked();
      g_adc_dma_transfer_count = 0;
      g_adc_dma_block_count = 0;
      g_adc_dma_error_count = 0;
      g_adc_overrun_count = 0;
      g_adc_dma_service_last_us = 0;
      g_adc_dma_service_max_us = 0;
      g_adc_dma_last_status = 0;
      g_adc_dma_run_start_counter = g_sample_counter;
      g_adc_latest_counter = g_sample_counter;

      if (running != 0) {
        // A previous limited-DMA run may have exhausted the ADC request
        // sequence. Re-arm both ADC DMNGT and the linear GPDMA block so every
        // RUN starts from a known one-shot state.
        if (rearmAdcDmaLimitedBlockLocked(false)) {
          g_adc_running = true;
        } else {
          g_adc_dma_error_count++;
          g_adc_running = false;
        }
      } else {
        g_adc_running = false;
      }
    } else if (running != 0) {
      clearAdcFlags();
      armAdcHardwareTriggerLocked();
      g_adc_running = true;
    } else {
      disarmAdcHardwareTriggerLocked();
      g_adc_running = false;
    }

    const uint32_t counter = g_sample_counter;
    irq_unlock(key);
    return counter;
  }
#endif

  // Software fallback. The requested sample rate is mirrored here when the
  // hardware-trigger backend is unavailable.
  g_adc_running = (running != 0);
  g_adc_write_index = 0;
  g_adc_count = 0;

  if (g_adc_running) {
    g_rate_start_us = micros();
    g_rate_start_counter = g_sample_counter;
    g_actual_sample_rate_mHz = g_adc_requested_rate_hz * 1000UL;
    g_software_sample_period_us = g_adc_requested_rate_hz > 0U ? (1000000UL / g_adc_requested_rate_hz) : 5000UL;
    g_next_sample_us = g_rate_start_us;
    g_adc_read_last_us = 0;
    g_adc_read_max_us = 0;
    g_scheduler_late_max_us = 0;
  }

  return g_sample_counter;
}

// Read-only state snapshot for Linux/browser synchronization.
String get_dac_diag() {
#if UNOQ_HAS_ZEPHYR_IRQ_HEADER
  const unsigned int key = irq_lock();
#endif
  const uint32_t frequency_mHz = (uint32_t)g_frequency_mHz;
  const uint32_t timer_rate_hz =
      (g_dac_hw_timer_active && g_dac_timer_rate_hz > 0U)
          ? g_dac_timer_rate_hz
          : DAC_HW_TIMER_FALLBACK_HZ;
  const uint32_t phase_step = g_dac_phase_step;
  const uint16_t offset_code = g_dac_offset_code;
  const uint16_t amplitude_code = g_dac_amplitude_code;
  int32_t raw_min_code = (int32_t)offset_code;
  int32_t raw_max_code = (int32_t)offset_code;
  if (g_waveform != 0) {
    raw_min_code -= (int32_t)amplitude_code;
    raw_max_code += (int32_t)amplitude_code;
  }
  const uint16_t observed_min_code =
      (uint16_t)(raw_min_code < 0 ? 0 : (raw_min_code > 4095 ? 4095 : raw_min_code));
  const uint16_t observed_max_code =
      (uint16_t)(raw_max_code < 0 ? 0 : (raw_max_code > 4095 ? 4095 : raw_max_code));
  const uint32_t clip_count = (raw_min_code < 0 || raw_max_code > 4095) ? 1U : 0U;
  const uint32_t irq_count = g_dac_irq_count;
  const uint32_t timer_tick_count = g_dac_timer_tick_count;
  const uint32_t isr_max_cycles = g_dac_isr_max_cycles;
  const uint32_t irq_late_max_cycles = g_dac_irq_late_max_cycles;
  const uint32_t overrun_count = g_dac_overrun_count;
#if UNOQ_HAS_ZEPHYR_IRQ_HEADER
  irq_unlock(key);
#endif

  const uint32_t dds_actual_mHz = (uint32_t)(
      (((uint64_t)phase_step * (uint64_t)timer_rate_hz * 1000ULL) +
       0x80000000ULL) >> 32
  );
  const uint32_t samples_per_cycle_x1000 =
      frequency_mHz > 0U
          ? (uint32_t)(((uint64_t)timer_rate_hz * 1000000ULL) / frequency_mHz)
          : 0U;
  uint32_t timer_clock_hz = SystemCoreClock;
  if (timer_clock_hz == 0U) timer_clock_hz = 160000000UL;
  const uint32_t isr_max_us = (uint32_t)(
      ((uint64_t)isr_max_cycles * 1000000ULL + timer_clock_hz / 2U) / timer_clock_hz
  );
  const uint32_t irq_late_max_us = (uint32_t)(
      ((uint64_t)irq_late_max_cycles * 1000000ULL + timer_clock_hz / 2U) / timer_clock_hz
  );

  String out;
  out.reserve(220);
  out += String(frequency_mHz);
  out += ",";
  out += String(timer_rate_hz);
  out += ",";
  out += String(phase_step);
  out += ",";
  out += String(dds_actual_mHz);
  out += ",";
  out += String(samples_per_cycle_x1000);
  out += ",";
  out += String(offset_code);
  out += ",";
  out += String(amplitude_code);
  out += ",";
  out += String(observed_min_code);
  out += ",";
  out += String(observed_max_code);
  out += ",";
  out += String(clip_count);
  out += ",";
  out += String(irq_count);
  out += ",";
  out += String(timer_tick_count);
  out += ",";
  out += String(isr_max_us);
  out += ",";
  out += String(irq_late_max_us);
  out += ",";
  out += String(overrun_count);
  return out;
}

String get_signal_state() {
  const int waveform = g_waveform;
  const int frequency_mHz = g_frequency_mHz;
  const int amplitude_mV = g_amplitude_mV;
  const int offset_mV = g_offset_mV;
  const int dac_running = g_dac_running ? 1 : 0;
  const int adc_running = g_adc_running ? 1 : 0;
  const uint32_t phase_origin = g_phase_origin;
  const uint32_t sample_counter = g_sample_counter;
  const uint32_t sample_rate_mHz = g_actual_sample_rate_mHz;
  const uint32_t adc_read_last_us = g_adc_read_last_us;
  const uint32_t adc_read_max_us = g_adc_read_max_us;
  const uint32_t scheduler_late_max_us = g_scheduler_late_max_us;
  const uint32_t dac_timer_rate_hz =
      g_dac_hw_timer_active ? g_dac_timer_rate_hz : 0U;
  const uint32_t dac_hw_timer = g_dac_hw_timer_active ? 1U : 0U;
  const uint32_t dac_timer_diag = g_dac_timer_diag;
  const int32_t dac_irq_connect_result = g_dac_irq_connect_result;
  uint32_t timer_clock_hz = SystemCoreClock;
  if (timer_clock_hz == 0U) timer_clock_hz = 160000000UL;
  const uint32_t dac_isr_max_us = (uint32_t)(
      ((uint64_t)g_dac_isr_max_cycles * 1000000ULL + timer_clock_hz / 2U) / timer_clock_hz
  );
  const uint32_t dac_irq_late_max_us = (uint32_t)(
      ((uint64_t)g_dac_irq_late_max_cycles * 1000000ULL + timer_clock_hz / 2U) / timer_clock_hz
  );
  const uint32_t dac_overrun_count = g_dac_overrun_count;

  const uint32_t adc_timer_rate_hz =
      g_adc_hw_timer_active ? g_adc_timer_rate_hz : 0U;
  const uint32_t adc_hw_timer = g_adc_hw_timer_active ? 1U : 0U;
  const uint32_t adc_timer_diag = g_adc_timer_diag;
  const int32_t adc_irq_connect_result = g_adc_irq_connect_result;
  const uint32_t adc_resolution_bits = 14U;
  const uint32_t adc_conversion_timeouts = g_adc_conversion_timeouts;
  const uint32_t adc_dma_active = g_adc_dma_active ? 1U : 0U;
  const uint32_t adc_dma_diag = g_adc_dma_diag;
  const int32_t adc_dma_irq_connect_result = g_adc_dma_irq_connect_result;
  const uint32_t adc_dma_channel = UNOQ_ADC_DMA_CHANNEL;
  const uint32_t adc_dma_request = UNOQ_ADC_DMA_REQUEST;
  const uint32_t adc_dma_buffer_size = ADC_DMA_BLOCK_SAMPLES;
  const uint32_t adc_dma_transfer_count = g_adc_dma_transfer_count;
  const uint32_t adc_dma_block_count = g_adc_dma_block_count;
  const uint32_t adc_dma_error_count = g_adc_dma_error_count;
  const uint32_t adc_overrun_count = g_adc_overrun_count;
  const uint32_t adc_dma_service_last_us = g_adc_dma_service_last_us;
  const uint32_t adc_dma_service_max_us = g_adc_dma_service_max_us;
  const uint32_t adc_dma_last_status = g_adc_dma_last_status;
  const uint32_t adc_latest_counter =
      (g_adc_dma_active && g_adc_running)
          ? g_adc_latest_counter
          : g_sample_counter;

  const uint32_t dma_test_stage = g_dma_test_stage;
  const uint32_t dma_m2m_pass = g_dma_m2m_pass;
  const uint32_t dma_m2m_status = g_dma_m2m_status;
  const uint32_t dma_m2m_dst0 = g_dma_m2m_dst0;
  const uint32_t dma_adc_single_pass = g_dma_adc_single_pass;
  const uint32_t dma_adc_single_status = g_dma_adc_single_status;
  const uint32_t dma_adc_single_sample = g_dma_adc_single_sample;
  const uint32_t dma_adc_block_pass = g_dma_adc_block_pass;

  String out;
  out.reserve(1200);
  out += String(waveform);
  out += ",";
  out += String(frequency_mHz);
  out += ",";
  out += String(amplitude_mV);
  out += ",";
  out += String(offset_mV);
  out += ",";
  out += String(dac_running);
  out += ",";
  out += String(adc_running);
  out += ",";
  out += String(phase_origin);
  out += ",";
  out += String(sample_counter);
  out += ",";
  out += String(sample_rate_mHz);
  out += ",";
  out += String(adc_read_last_us);
  out += ",";
  out += String(adc_read_max_us);
  out += ",";
  out += String(scheduler_late_max_us);
  out += ",";
  out += String(dac_timer_rate_hz);
  out += ",";
  out += String(dac_hw_timer);
  out += ",";
  out += String(dac_timer_diag);
  out += ",";
  out += String(dac_irq_connect_result);
  out += ",";
  out += String(adc_timer_rate_hz);
  out += ",";
  out += String(adc_hw_timer);
  out += ",";
  out += String(adc_timer_diag);
  out += ",";
  out += String(adc_irq_connect_result);
  out += ",";
  out += String(adc_resolution_bits);
  out += ",";
  out += String(adc_conversion_timeouts);
  out += ",";
  out += String(adc_dma_active);
  out += ",";
  out += String(adc_dma_diag);
  out += ",";
  out += String(adc_dma_irq_connect_result);
  out += ",";
  out += String(adc_dma_channel);
  out += ",";
  out += String(adc_dma_request);
  out += ",";
  out += String(adc_dma_buffer_size);
  out += ",";
  out += String(adc_dma_transfer_count);
  out += ",";
  out += String(adc_dma_block_count);
  out += ",";
  out += String(adc_dma_error_count);
  out += ",";
  out += String(adc_overrun_count);
  out += ",";
  out += String(adc_dma_service_last_us);
  out += ",";
  out += String(adc_dma_service_max_us);
  out += ",";
  out += String(adc_dma_last_status);
  out += ",";
  out += String(adc_latest_counter);
  out += ",";
  out += String(dma_test_stage);
  out += ",";
  out += String(dma_m2m_pass);
  out += ",";
  out += String(dma_m2m_status);
  out += ",";
  out += String(dma_m2m_dst0);
  out += ",";
  out += String(dma_adc_single_pass);
  out += ",";
  out += String(dma_adc_single_status);
  out += ",";
  out += String(dma_adc_single_sample);
  out += ",";
  out += String(dma_adc_block_pass);
  out += ",";
  out += String(g_dma_snap_ccr);
  out += ",";
  out += String(g_dma_snap_ctr1);
  out += ",";
  out += String(g_dma_snap_ctr2);
  out += ",";
  out += String(g_dma_snap_cbr1);
  out += ",";
  out += String(g_dma_snap_csar);
  out += ",";
  out += String(g_dma_snap_cdar);
  out += ",";
  out += String(g_dma_snap_csr);
  out += ",";
  out += String(g_dma_snap_adc_cfgr1);
  out += ",";
  out += String(g_dma_snap_adc_cr);
  out += ",";
  out += String(g_dma_snap_adc_isr);
  out += ",";
  out += String(g_adc_baseline_eoc_count);
  out += ",";
  out += String(g_dma_block_elapsed_us);
  out += ",";
  out += String(g_dma_block_tim15_irq_count);
  out += ",";
  out += String(g_dma_block_adc_requests_observed);
  out += ",";
  out += String(g_dma_block_cbr1_start);
  out += ",";
  out += String(g_dma_block_cbr1_end);
  out += ",";
  out += String(g_dma_irq_count);
  out += ",";
  out += String(g_dma_half_count);
  out += ",";
  out += String(g_dma_complete_count);
  out += ",";
  out += String(g_dma_adc_4sample_pass);
  out += ",";
  out += String(g_dma_adc_4sample_status);
  out += ",";
  out += String(g_dma_adc_4sample_elapsed_us);
  out += ",";
  out += String(g_dma_adc_4sample_tim15_irq_count);
  out += ",";
  out += String(g_dma_adc_4sample_cbr1_end);
  out += ",";
  out += String(g_dma_adc_4sample_requests);
  out += ",";
  out += String(g_dma_adc_8sample_pass);
  out += ",";
  out += String(g_dma_adc_8sample_status);
  out += ",";
  out += String(g_dma_adc_8sample_elapsed_us);
  out += ",";
  out += String(g_dma_adc_8sample_tim15_irq_count);
  out += ",";
  out += String(g_dma_adc_8sample_cbr1_end);
  out += ",";
  out += String(g_dma_adc_8sample_requests);
  out += ",";
  out += String(g_dma_idle_complete_count);
  out += ",";
  out += String(dac_isr_max_us);
  out += ",";
  out += String(dac_irq_late_max_us);
  out += ",";
  out += String(dac_overrun_count);
  out += ",";
  out += String(g_noise_enabled ? 1 : 0);
  out += ",";
  out += String(g_noise_level_percent);
  return out;
}

// ADC only. No DAC samples are transported back to Linux.
// Incremental transport: Linux sends the exclusive acquisition counter it has
// already consumed. The MCU returns the earliest unseen samples, bounded by
// the 2048-sample MCU ring and RPC_BATCH_SIZE.
String get_adc_since(int after_hi, int after_lo) {
  serviceDAC(micros());

  const uint32_t requested_counter =
      ((uint32_t)(after_hi & 0xFFFF) << 16) |
      (uint32_t)(after_lo & 0xFFFF);

#if UNOQ_ADC_HW_TRIGGER
  const unsigned int adc_snapshot_key = irq_lock();
#endif
  const uint32_t latest_counter =
      (g_adc_dma_active && g_adc_running)
          ? g_adc_latest_counter
          : g_sample_counter;
  const uint32_t phase_origin = g_phase_origin;
  const uint16_t available = g_adc_count;
  const uint16_t end_index = g_adc_write_index;
#if UNOQ_ADC_HW_TRIGGER
  irq_unlock(adc_snapshot_key);
#endif
  const uint32_t oldest_counter = latest_counter - available;

  uint32_t start_counter = requested_counter;
  if (start_counter < oldest_counter || start_counter > latest_counter) {
    start_counter = oldest_counter;
  }

  uint32_t unseen = latest_counter - start_counter;
  uint16_t count =
      (unseen < RPC_BATCH_SIZE) ? (uint16_t)unseen : RPC_BATCH_SIZE;

  String out;
  // Lean 8-field runtime header; DMA diagnostics are available from status
  // and do not need to be repeated in every high-rate sample RPC.
  out.reserve(850);
  out += String(latest_counter);
  out += ",";
  out += String(phase_origin);
  out += ",";
  out += String(start_counter);
  out += ",";
  out += String(count);
  out += ",";
  out += String(g_actual_sample_rate_mHz);
  out += ",";
  out += String(g_adc_read_last_us);
  out += ",";
  out += String(g_adc_read_max_us);
  out += ",";
  out += String(g_scheduler_late_max_us);
  out += "|~";

  if (count == 0) {
    serviceDAC(micros());
    return out;
  }

  const uint16_t oldest_index =
      (end_index + ADC_BUFFER_SIZE - available) % ADC_BUFFER_SIZE;
  const uint16_t offset = (uint16_t)(start_counter - oldest_counter);
  const uint16_t start_index = (oldest_index + offset) % ADC_BUFFER_SIZE;

  for (uint16_t i = 0; i < count; i += 2U) {
    const uint16_t idx0 = (start_index + i) % ADC_BUFFER_SIZE;
    const uint16_t first = g_adc_buffer[idx0] & 0x3FFFU;
    uint16_t second = 0U;

    if ((uint16_t)(i + 1U) < count) {
      const uint16_t idx1 = (start_index + i + 1U) % ADC_BUFFER_SIZE;
      second = g_adc_buffer[idx1] & 0x3FFFU;
    }

    appendPackedAdcPair(out, first, second);

    if ((i & 0x07U) == 0x06U) {
      serviceDAC(micros());
    }
  }

  serviceDAC(micros());
  return out;
}

void setup() {
  // Register RouterBridge endpoints first. After an App Lab flash the Linux
  // process can come up before the MCU has finished analog/timer setup; making
  // the RPC routes available at the earliest safe point reduces that startup
  // race. Python still waits through its startup quiet window before calling.
  Bridge.begin();
  Bridge.provide("set_signal", set_signal);
  Bridge.provide("set_noise", set_noise);
  Bridge.provide("set_dac_run", set_dac_run);
  Bridge.provide("set_adc_run", set_adc_run);
  Bridge.provide("set_adc_sample_rate", set_adc_sample_rate);
  Bridge.provide("get_signal_state", get_signal_state);
  Bridge.provide("get_dac_diag", get_dac_diag);
  Bridge.provide("get_adc_since", get_adc_since);

  analogWriteResolution(12);
  analogReadResolution(14);
  pinMode(A1, INPUT);

  // One-time Arduino/Zephyr ADC initialization: applies PA5 analog pinctrl,
  // ADC1 channel configuration, regulator/calibration and native channel
  // sampling-time setup. v19.0 then keeps that analog front-end configuration
  // and moves conversion START timing to TIM15 TRGO.
  const int adc_warm_raw = analogRead(A1);
  if (adc_warm_raw >= 0 && adc_warm_raw <= 16383) {
    g_adc_last_code = (uint16_t)adc_warm_raw;
  }

  // One-time Arduino/Zephyr DAC initialization. Subsequent waveform writes
  // use the direct DAC register path.
  analogWrite(DAC0, 0);
  g_dac_last_code = 0;

  buildSineLut();

#if defined(ARDUINO_UNO_Q)
  g_dac_timer_diag |= (1UL << 0);
#endif
#if UNOQ_HAS_ZEPHYR_IRQ_HEADER
  g_dac_timer_diag |= (1UL << 1);
#endif
#if UNOQ_HAS_DYNAMIC_IRQ
  g_dac_timer_diag |= (1UL << 2);
#endif

#if UNOQ_DAC_HW_TIMER
  (void)setupDacHardwareTimer();
#else
  // Hardware backend is unavailable in this loader/core build.
  g_dac_hw_timer_active = false;
  g_dac_timer_rate_hz = 0;
#endif

#if UNOQ_ADC_HW_TRIGGER
  (void)setupAdcHardwareTiming();
#else
  g_adc_hw_timer_active = false;
  g_adc_timer_rate_hz = 0;
#endif

  g_phase_origin = g_sample_counter;
  g_dac_phase_start_us = micros();

  if (!g_adc_hw_timer_active) {
    g_rate_start_us = g_dac_phase_start_us;
    g_rate_start_counter = g_sample_counter;
    g_next_sample_us = g_dac_phase_start_us;
  }
}

void loop() {
  // On UNO Q v19.11, TIM6 owns DAC output timing and TIM15 TRGO owns ADC
  // sample-start timing. The default ADC backend reads DR immediately in the
  // TIM15 ISR; loop() remains outside both real-time timebases.
  serviceDAC(micros());
  serviceAcquisitionClock(micros());
  serviceDAC(micros());
}
