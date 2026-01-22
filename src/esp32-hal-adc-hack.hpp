#pragma once

#include <cstdint>
#include <soc/soc_caps.h>

using SamplePtr = uint16_t *;
using adc_continuous_results_t = SamplePtr[SOC_ADC_MAX_CHANNEL_NUM];
bool analogContinuousReadSamples(adc_continuous_results_t &results, uint32_t timeout_ms);
