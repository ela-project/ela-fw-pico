#pragma once

#include <stdint.h>

using uint = unsigned int;

inline constexpr uint trg_event_rising{1 << 3};
inline constexpr uint trg_event_falling{1 << 2};
inline constexpr uint trq_event_both{trg_event_rising | trg_event_falling};
enum class pin_trigger_event_t : uint { HIGH = 1 << 0, LOW = 1 << 1, FALLING = trg_event_falling, RISING = trg_event_rising };

inline bool pin_state(uint8_t sample, uint8_t pin_number) {
    return sample & (1 << pin_number);
}

inline bool trigger_occured(bool sample_before, bool sample_after, uint trig_event) {
    if (trig_event & trq_event_both) {
        return sample_before != sample_after;
    } else if (trig_event & trg_event_rising) {
        return sample_before == false && sample_after == true;
    } else if (trig_event & trg_event_falling) {
        return sample_before == true && sample_after == false;
    }
    return false;
}

inline uint32_t find_late_trigger(uint32_t trig_index, uint8_t *sample_array, uint trig_event, uint8_t pin_num) {
    bool sample_before;
    bool sample_after;
    for (uint32_t index{trig_index}; index > 0; --index) {
        sample_after = pin_state(sample_array[index], pin_num);
        sample_before = pin_state(sample_array[index - 1], pin_num);
        if (trigger_occured(sample_before, sample_after, trig_event)) {
            return index;
        }
    }
    return trig_index;
}

inline uint32_t find_start_of_array(bool trig_occured, uint32_t trigger_index, uint32_t pretrig_count, uint32_t array_size, uint32_t dma_end_index,
                                    bool dma_cycled) {
    uint32_t start_of_array = 0;

    if (trig_occured) {
        if (pretrig_count > trigger_index) {
            const uint32_t missing_samples = pretrig_count - trigger_index;
            if (!dma_cycled) {
                start_of_array = 0;
            } else {
                if (dma_end_index <= trigger_index) {
                    start_of_array = dma_end_index;
                } else {
                    const uint32_t avalible_from_last_cycle = array_size - dma_end_index;
                    const uint32_t samples_before_trigger = avalible_from_last_cycle + trigger_index;
                    if (samples_before_trigger > pretrig_count) {
                        start_of_array = array_size - missing_samples;
                    } else {
                        start_of_array = dma_end_index;
                    }
                }
            }
        } else {
            const uint32_t start_index = trigger_index - pretrig_count;
            if (start_index < dma_end_index) {
                start_of_array = start_index;
            } else {
                start_of_array = dma_end_index;
            }
        }
    } else {
        start_of_array = 0;
    }

    return start_of_array;
}

inline uint32_t find_sample_count(uint32_t start_of_array, uint32_t desired_sample_count, uint32_t array_size, uint32_t dma_end_index, bool dma_cycled) {
    uint32_t sample_count = 0;
    uint32_t avalible_samples{0};

    if (!dma_cycled) {
        avalible_samples = dma_end_index - start_of_array;
    } else {
        if (dma_end_index > start_of_array) {
            avalible_samples = dma_end_index - start_of_array;
        } else {
            avalible_samples = dma_end_index + (array_size - start_of_array);
        }
    }

    if (avalible_samples >= desired_sample_count) {
        sample_count = desired_sample_count;
    } else {
        sample_count = avalible_samples;
    }

    return sample_count;
}

inline uint32_t calc_trig_sample(uint32_t start_of_array, uint32_t trig_index, uint32_t array_size) {
    if (start_of_array < trig_index) {
        return trig_index - start_of_array;
    } else {
        return trig_index + (array_size - start_of_array);
    }
}