#pragma once

#include <cstdint>
#include <soc/soc_caps.h>

struct adc_continuous_sum_count_t {
  uint32_t sum_read_raw, count, invalids_count;
};

using adc_continuous_results_t = adc_continuous_sum_count_t[SOC_ADC_MAX_CHANNEL_NUM];
bool analogContinuousReadSumCount(adc_continuous_results_t &results, uint32_t timeout_ms);
