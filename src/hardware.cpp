#include <stdint.h>

#include "ela_configuration.h"
#include "hardware.hpp"
#include "hardware/structs/clocks.h"
#include "hw_helper_func.hpp"

#include "analyzer.hpp"
#include "ela.h"

#include "hardware/structs/bus_ctrl.h"

#include "hardware/timer.h"
#include "usb_stream.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "debug_print_uart.hpp"
#include "pins_input.pio.h"

#ifndef NDEBUG
#define DEBUG_UART_MSG
#endif

Hardware hw_agent;

static const USBStream usb_stream{stdio_usb};
static constexpr uint led_pin{25};
#ifdef DEBUG_UART_MSG
static DebugPrintUart<true> debug_uart{uart0, 12};
#endif
static uint gpio_dma_chan;
static uint ctrl_dma_chan;
static constexpr uint pwm10_pin{16};
static constexpr uint pwm25_pin{18};
static uint pwm10_slice;
static uint pwm25_slice;
static const PIO _pio{pio0};
static uint _sm;
static uint _pio_offset;

static void *m_sample_buffer_addr{0};
static void **const ctrl_read_addr{&m_sample_buffer_addr};
static io_rw_32 *ctrl_write_addr;

static uint32_t dma_postrig_count{1};

static constexpr uint32_t pio_instr_count{3};

static volatile bool dma_irq_fired{false};
#ifdef DEBUG_UART_MSG
static volatile bool dma_irq_fired_debug{false};
#endif

void dma_irq_handler() {
    if (dma_channel_get_irq0_status(gpio_dma_chan)) {
        dma_irq_fired = true;
#ifdef DEBUG_UART_MSG
        dma_irq_fired_debug = true;
#endif
        dma_channel_set_irq0_enabled(gpio_dma_chan, false);
        dma_channel_acknowledge_irq0(gpio_dma_chan);
    }
}

static volatile bool pio_irq_fired{false};
#ifdef DEBUG_UART_MSG
static volatile bool pio_irq_fired_debug{false};
#endif
void pio_irq_handler() {
    if (pio_interrupt_get(_pio, 0)) {
        pio_irq_fired = true;
#ifdef DEBUG_UART_MSG
        pio_irq_fired_debug = true;
#endif
        pio_interrupt_clear(_pio, 0);
    }
}

static uint irq_gpio{0};
static uint irq_event{0};
static volatile uint32_t dma_trig_tx_count{SAMPLE_BUFFER_SIZE + 10};
static volatile bool gpio_irq_fired{false};
#ifdef DEBUG_UART_MSG
static volatile bool gpio_irq_fired_debug{false};
#endif
void gpio_irq_handler() {
    _pio->txf[_sm] = dma_postrig_count;
    dma_trig_tx_count = dma_channel_hw_addr(gpio_dma_chan)->transfer_count;
    // pio_sm_put(_pio, _sm, dma_postrig_count);
    gpio_acknowledge_irq(irq_gpio, irq_event);
    gpio_set_irq_enabled(irq_gpio, irq_event, false);
    gpio_irq_fired = true;
#ifdef DEBUG_UART_MSG
    gpio_irq_fired_debug = true;
#endif
}

void Hardware::init(void) {
    gpio_init(led_pin);
    gpio_set_dir(led_pin, true);
    usb_stream.init();

#ifdef DEBUG_UART_MSG
    debug_uart.init();
#endif

    constexpr uint gpio_in_pins[]{0, 1, 2, 3, 4, 5, 6, 7};

    for (const uint &pin : gpio_in_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, true);
    }

    const uint32_t sys_freq = clock_get_hz(clk_sys);

    const uint16_t pwm10_wrap = sys_freq/10000;
    gpio_set_function(pwm10_pin, GPIO_FUNC_PWM);
    pwm10_slice = pwm_gpio_to_slice_num(pwm10_pin);
    pwm_set_wrap(pwm10_slice, pwm10_wrap - 1);
    pwm_set_gpio_level(pwm10_pin, pwm10_wrap >> 1);

    const uint16_t pwm25_wrap = sys_freq/25000;;
    gpio_set_function(pwm25_pin, GPIO_FUNC_PWM);
    pwm25_slice = pwm_gpio_to_slice_num(pwm25_pin);
    pwm_set_wrap(pwm25_slice, pwm25_wrap - 1);
    pwm_set_gpio_level(pwm25_pin, pwm25_wrap >> 1);

    _pio_offset = pio_add_program(_pio, &pins_input_program);
    _sm = pio_claim_unused_sm(_pio, true);
    init_pio_pins_input(_pio, _sm, _pio_offset, 0);

    gpio_dma_chan = dma_claim_unused_channel(true);
    ctrl_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config dma_conf = dma_channel_get_default_config(gpio_dma_chan);
    // channel_config_set_irq_quiet(&dma_conf, true);
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_conf, false);
    channel_config_set_write_increment(&dma_conf, true);
    channel_config_set_dreq(&dma_conf, pio_get_dreq(_pio, _sm, false));
    channel_config_set_chain_to(&dma_conf, ctrl_dma_chan);
    dma_channel_configure(gpio_dma_chan, &dma_conf, &m_sample_buffer[0], &_pio->rxf[_sm], SAMPLE_BUFFER_SIZE, false);

    m_sample_buffer_addr = &m_sample_buffer[0];
    ctrl_write_addr = &(dma_channel_hw_addr(gpio_dma_chan)->al2_write_addr_trig);

    dma_conf = dma_channel_get_default_config(ctrl_dma_chan);
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_conf, false);
    channel_config_set_write_increment(&dma_conf, false);
    dma_channel_configure(ctrl_dma_chan, &dma_conf, ctrl_write_addr, ctrl_read_addr, 1, false);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_irq_handler);

    dma_channel_set_irq0_enabled(gpio_dma_chan, true);
    pio_set_irq0_source_enabled(_pio, pis_interrupt0, true);

    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_enabled(PIO0_IRQ_0, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    pwm_set_enabled(pwm10_slice, true);
    pwm_set_enabled(pwm25_slice, true);

    uint8_t temp{0};
    for (uint8_t &sample : m_sample_buffer) {
        sample = temp++;
    }
#ifdef DEBUG_UART_MSG
    send_debug("Hello there\nSysclk: ");
    send_debug(sys_freq);
    send_debug("\n");
#endif
};

void Hardware::send_debug(const char *string, size_t string_size) {
#ifdef DEBUG_UART_MSG
    debug_uart.write(string, string_size);
#endif
}

void Hardware::send_debug(const long num, int base) {
#ifdef DEBUG_UART_MSG
    debug_uart.print(num, base);
#endif
};

void Hardware::idle() {
    static int counter{0};
    constexpr int counter_wrap{500000};
    int rx_char;
    if (!m_rx_buffer.full()) {
        while ((rx_char = usb_stream.receive_timeout(0)) >= 0) {
            rx_buffer_push(static_cast<uint8_t>(rx_char));
            if (m_rx_buffer.full()) {
                break;
            }
        }
    }

    if (!counter) {
        toggle_LED();
    }
#ifdef DEBUG_UART_MSG
    if (dma_irq_fired_debug) {
        send_debug("DMA IRQ!\n");
        dma_irq_fired_debug = false;
    }

    if (gpio_irq_fired_debug) {
        send_debug("GPIO IRQ!\n");
        gpio_irq_fired_debug = false;
    }

    if (pio_irq_fired_debug) {
        send_debug("PIO IRQ!\n");
        pio_irq_fired_debug = false;
    }
#endif
    if (m_current_state == HW_STATE_BUSY_SAMPLING && pio_irq_fired) {
#ifdef DEBUG_UART_MSG
        send_debug("\nSampling done\n End Addr: ");
        send_debug(dma_channel_hw_addr(gpio_dma_chan)->write_addr, 16);
        send_debug("\n Tx count: ");
        send_debug(dma_channel_hw_addr(gpio_dma_chan)->transfer_count);
        send_debug("\n IRQ Tx count: ");
        send_debug(dma_trig_tx_count);
        send_debug("\n");
#endif
        end_sampling();
#ifdef DEBUG_UART_MSG
        send_debug("\nArray start: ");
        send_debug(m_sample_buffer_start);
        send_debug("\nTrig index: ");
        send_debug(m_sample_buffer_trig_index);
        send_debug("\nSample count: ");
        send_debug(m_sample_buffer_count);
        send_debug("\n");
#endif

        // busy_wait_ms(1);
        dma_irq_fired = false;
        pio_irq_fired = false;
        gpio_irq_fired = false;
    }

    counter = (counter + 1) % counter_wrap;
};

ela_status_t Hardware::send(const uint8_t *data, const size_t length) {
    usb_stream.send(data, length);
    // usb_stream.flush();
    return STATUS_OK;
};

void Hardware::comm_flush() {
    usb_stream.flush();
}

void Hardware::toggle_LED() {
    gpio_xor_mask(1 << led_pin);
}

void Hardware::error_handler() {
    while (true) {
        toggle_LED();
        busy_wait_ms(500);
    }
}

ela_status_t Hardware::start_sampling() {
    m_sample_buffer_start = 0;
    m_sample_buffer_trig_index = 0;
    m_sample_buffer_count = 0;
    dma_irq_fired = false;
    pio_irq_fired = false;

    const uint32_t sys_freq = clock_get_hz(clk_sys);
    const uint16_t pio_div = (sys_freq / pio_instr_count) / analyzer_agent.get_samplerate();

#ifdef DEBUG_UART_MSG
    send_debug("Start sampling\n Div: ");
    send_debug(pio_div);
#endif

    pio_sm_set_clkdiv_int_frac(_pio, _sm, pio_div, 0);
    dma_channel_set_write_addr(gpio_dma_chan, &m_sample_buffer[0], false);

#ifdef DEBUG_UART_MSG
    send_debug("\n Addr: ");
    send_debug(dma_channel_hw_addr(gpio_dma_chan)->write_addr, 16);
    send_debug("\n Count: ");
    send_debug(analyzer_agent.get_sample_count());
    send_debug("\n Ctrl: ");
    send_debug(dma_channel_hw_addr(gpio_dma_chan)->ctrl_trig, 16);
    send_debug("\n");
#endif

    for (size_t i{0}; i < NUM_OF_DIGITAL_PINS; i++) {
        Analyzer::pin_mode_t pin_trig_cond = analyzer_agent.get_pin_mode(i);
        if (pin_trig_cond >= Analyzer::pin_mode_t::PM_TRIGGER_BEGIN && pin_trig_cond <= Analyzer::pin_mode_t::PM_ENUM_END) {
            if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_RISING || pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_HIGH) {
                irq_event = static_cast<uint>(pin_trigger_event_t::RISING);
            } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_FALLING || pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_LOW) {
                irq_event = static_cast<uint>(pin_trigger_event_t::FALLING);
            } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_BOTH) {
                irq_event = static_cast<uint>(pin_trigger_event_t::FALLING) | static_cast<uint>(pin_trigger_event_t::RISING);
            }
            irq_gpio = i;
            if (analyzer_agent.get_postrig_count()) {
                dma_postrig_count = analyzer_agent.get_postrig_count();
            } else {
                dma_postrig_count = 1;
            }
#ifdef DEBUG_UART_MSG
            send_debug("Trigger:\n GPIO: ");
            send_debug(irq_gpio);
            send_debug("\n Event: ");
            send_debug(irq_event);
            send_debug("\n Postrig: ");
            send_debug(dma_postrig_count);
            send_debug("\n");
#endif
            break;
        }
    }

    pio_sm_exec(_pio, _sm, pio_encode_jmp(_pio_offset));
    pio_sm_clear_fifos(_pio, _sm);
    pio_sm_restart(_pio, _sm);
    if (!irq_event) {
        m_sample_buffer_addr = 0;
        pio_sm_put(_pio, _sm, analyzer_agent.get_sample_count());
    } else {
        dma_channel_acknowledge_irq0(gpio_dma_chan);
        dma_channel_set_irq0_enabled(gpio_dma_chan, true);
        m_sample_buffer_addr = &m_sample_buffer[0];
    }
    dma_channel_start(gpio_dma_chan);
    pio_sm_set_enabled(_pio, _sm, true);

    if (irq_event) {
        gpio_set_irq_enabled(irq_gpio, irq_event, true);
    }
    m_current_state = HW_STATE_BUSY_SAMPLING;
    return STATUS_OK;
}

void Hardware::abort_sampling(void) {
#ifdef DEBUG_UART_MSG
    send_debug("ABORT!\n");
#endif
    if (irq_event) {
        gpio_set_irq_enabled(irq_gpio, irq_event, false);
    }
    pio_sm_set_enabled(_pio, _sm, false);
    m_sample_buffer_addr = 0;
    dma_channel_abort(gpio_dma_chan);
    reset();
}

void Hardware::end_sampling(void) {
    const bool dma_cycled{dma_irq_fired};
    const bool trigger_occured{gpio_irq_fired};
    const uint32_t sample_buffer_end = SAMPLE_BUFFER_SIZE - dma_channel_hw_addr(gpio_dma_chan)->transfer_count;
    const uint irq_event_temp{irq_event};
    m_sample_buffer_addr = 0;
    dma_channel_abort(gpio_dma_chan);
    pio_sm_set_enabled(_pio, _sm, false);
    if (irq_event) {
        gpio_set_irq_enabled(irq_gpio, irq_event, false);
        irq_event = 0;
    }

    if (trigger_occured) {
        const uint32_t trig_index_calc = SAMPLE_BUFFER_SIZE - dma_trig_tx_count;
        const uint32_t trig_index_calc_corrected = find_late_trigger(trig_index_calc, &m_sample_buffer[0], irq_event_temp, irq_gpio);

#ifdef DEBUG_UART_MSG
        send_debug("Trig index: \n");
        send_debug(trig_index_calc);
        send_debug("\nTrig index corrected: \n");
        send_debug(trig_index_calc_corrected);
        send_debug("\n");
#endif

        m_sample_buffer_start = find_start_of_array(true,
                                                    trig_index_calc_corrected,
                                                    analyzer_agent.get_pretrig_count(),
                                                    SAMPLE_BUFFER_SIZE,
                                                    sample_buffer_end,
                                                    dma_cycled);

        m_sample_buffer_trig_index = calc_trig_sample(m_sample_buffer_start, trig_index_calc_corrected, SAMPLE_BUFFER_SIZE);
    } else {
        m_sample_buffer_start = find_start_of_array(false, 0, analyzer_agent.get_pretrig_count(), SAMPLE_BUFFER_SIZE, sample_buffer_end, dma_cycled);
        m_sample_buffer_trig_index = 0;
    }

    m_sample_buffer_count = find_sample_count(m_sample_buffer_start, analyzer_agent.get_sample_count(), SAMPLE_BUFFER_SIZE, sample_buffer_end, dma_cycled);

    if (m_sample_buffer_trig_index > m_sample_buffer_count) {
        m_sample_buffer_trig_index = 0;
    }

    hw_agent.m_current_state = Hardware::HW_STATE_SAMPLING_DONE;
}

void Hardware::reset() {
    if (irq_event) {
        gpio_set_irq_enabled(irq_gpio, irq_event, false);
        dma_channel_set_irq0_enabled(gpio_dma_chan, false);
        irq_event = 0;
    }
    irq_gpio = 0;
    pio_irq_fired = false;
    dma_irq_fired = false;
    gpio_irq_fired = false;
    m_sample_buffer_start = 0;
    m_sample_buffer_trig_index = 0;
    m_sample_buffer_count = 0;
    m_current_state = HW_STATE_IDLE;
}