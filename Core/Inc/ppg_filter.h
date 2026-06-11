#ifndef __PPG_FILTER_H__
#define __PPG_FILTER_H__

#include "arm_math.h"

/**
 * @brief Initializes the PPG bandpass filter.
 */
void PPG_Filter_Init(void);

/**
 * @brief Filters a single PPG sample.
 * @param sample The raw input sample.
 * @return The filtered output sample.
 */
float32_t PPG_Filter_ProcessSample(float32_t sample);

#endif /* __PPG_FILTER_H__ */
