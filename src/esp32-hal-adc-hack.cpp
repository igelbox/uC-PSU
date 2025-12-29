#include "esp32-hal-adc-hack.hpp"

#include <esp32-hal-adc.h>
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_oneshot.h>

typedef struct {
  voidFuncPtr fn;
  void *arg;
} interrupt_config_t;

typedef struct {
  adc_oneshot_unit_handle_t _;
  adc_continuous_handle_t adc_continuous_handle;
  interrupt_config_t _;
  adc_cali_handle_t _;
  uint32_t _;
  uint32_t conversion_frame_size;
} adc_handle_t;

extern adc_handle_t adc_handle;

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

bool analogContinuousReadSumCount(adc_continuous_results_t &results, uint32_t timeout_ms) {
  if (!adc_handle.adc_continuous_handle) {
    log_e("ADC Continuous is not initialized!");
    return false;
  }
  uint32_t bytes_read = 0;
  uint8_t adc_read[adc_handle.conversion_frame_size];
  memset(adc_read, 0xcc, sizeof(adc_read));

  if (auto err = adc_continuous_read(adc_handle.adc_continuous_handle, adc_read, adc_handle.conversion_frame_size,
                                     &bytes_read, timeout_ms);
      err != ESP_OK) {
    log_e("Reading data failed with error: %X", err);
    return false;
  }

  memset(results, 0, sizeof(results));
  for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
    const auto p = (adc_digi_output_data_t *)&adc_read[i];
    const auto chan_num = ADC_GET_CHANNEL(p);
    const auto data = ADC_GET_DATA(p);

    if (chan_num >= SOC_ADC_CHANNEL_NUM(0)) {
      log_e("Invalid data [%d_%d]", chan_num, data);
      return false;
    }

    auto &result = results[chan_num];
    if (data >= (1 << SOC_ADC_DIGI_MAX_BITWIDTH)) {
      result.invalids_count += 1;
      log_e("Invalid data");
      continue;
    }

    result.sum_read_raw += data;
    result.count += 1;
  }
  return true;
}
