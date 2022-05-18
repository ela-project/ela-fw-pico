#ifndef _HARDWARE_H
#define _HARDWARE_H

#include <stdint.h>
#include <sys/types.h>
#include <cstddef>
#include "analyzer.hpp"
#include "debug_print_uart.hpp"
#include "ela.h"
#include "etl/queue.h"

#define RX_BUFFER_SIZE 128

/* Class acts as a middle man between aplication and HAL */
class Hardware {
   public:
    Hardware() = default;

    enum hw_state_t : uint8_t {
        HW_STATE_IDLE = 0x00U,
        HW_STATE_BUSY_SAMPLING = 0x01U,
        HW_STATE_SAMPLING_DONE = 0x02U,
    };

    /* Public: Member variables */
    static inline uint8_t m_sample_buffer[SAMPLE_BUFFER_SIZE];
    static inline etl::queue<uint8_t, RX_BUFFER_SIZE, etl::memory_model::MEMORY_MODEL_MEDIUM> m_rx_buffer;
    static inline arrat_index_t m_sample_buffer_start{0};
    static inline arrat_index_t m_sample_buffer_end{0};
    static inline arrat_index_t m_sample_buffer_count{0};
    static inline arrat_index_t m_sample_buffer_trig_index{0};
    static inline hw_state_t m_current_state{HW_STATE_IDLE};
    static inline size_t m_sb_index{0};

    /* Public: Inline member functions */

    static uint8_t receive_byte() {  // Pop value out of RX serial buffer
        uint8_t rx_byte;
        m_rx_buffer.pop_into(rx_byte);
        // send_debug(rx_byte);
        return rx_byte;
    };

    template <size_t STRING_SIZE>
    static void send_debug(const char (&string)[STRING_SIZE]) {
        send_debug(string, STRING_SIZE - 1);
    };

    static void send_debug(const char *string, size_t string_size);
    static void send_debug(const long num, int base = 10);

    /* Public: Member functions */
    static int data_avalible() {
        return m_rx_buffer.size();
    };  // Data in RX serial buffer
    static void error_handler();
    static void init();
    static void idle();
    static void reset();
    static ela_status_t send(const uint8_t *data, const size_t length);

    static void comm_flush();

    static ela_status_t rx_buffer_push(const uint8_t &data) {
        if (!m_rx_buffer.full()) {
            m_rx_buffer.push(data);
            return STATUS_OK;
        }
        return STATUS_ERROR;
    };

    static void toggle_LED(void);

    static ela_status_t start_sampling();
    static void abort_sampling(void);
    static void end_sampling(void);
};  // END class Hardware

extern Hardware hw_agent;

#endif