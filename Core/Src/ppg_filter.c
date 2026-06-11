#include "ppg_filter.h"

#define PPG_FILTER_STAGES 2

static float32_t ppg_filter_state[4 * PPG_FILTER_STAGES];
static arm_biquad_casd_df1_inst_f32 ppg_filter_inst;

// Stage 1: 2nd-order Butterworth LPF (fc = 5.0 Hz, fs = 100.0 Hz)
// Stage 2: 2nd-order Butterworth HPF (fc = 0.5 Hz, fs = 100.0 Hz)
static float32_t ppg_filter_coeffs[5 * PPG_FILTER_STAGES] = {
    // Stage 1 (LPF: 5 Hz)
    0.020083366f, 0.040166732f, 0.020083366f, 1.561018076f, -0.641351538f,
    // Stage 2 (HPF: 0.25 Hz)
    0.988954132f, -1.977908263f, 0.988954132f, 1.977786373f, -0.978029518f
};

/**
 * @brief Initializes the PPG bandpass filter.
 */
void PPG_Filter_Init(void) {
    arm_biquad_cascade_df1_init_f32(&ppg_filter_inst, PPG_FILTER_STAGES, ppg_filter_coeffs, ppg_filter_state);
}

/**
 * @brief Filters a single PPG sample.
 * @param sample The raw input sample.
 * @return The filtered output sample.
 */
float32_t PPG_Filter_ProcessSample(float32_t sample) {
    float32_t input = sample;
    float32_t output = 0.0f;
    arm_biquad_cascade_df1_f32(&ppg_filter_inst, &input, &output, 1);
    return output;
}
