#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include ".config.hpp"
#include "assert.hpp"
#include "esp32-hal-adc-hack.hpp"
#include "interpoly.hpp"
#include "preferences.hpp"

Preferences preferences;

bool on = true, overcurrent = false;

auto mid_or_0(int16_t a, int16_t b) {
  const auto d = (b - a) / 2, m = a + d;
  return d && (m != b) ? m : 0;
}

inline static const std::string POLY_SUFFIX{"-poly"};
struct Pin {
  const uint8_t pin;
  const char *key;
  const int16_t threshold;
  const bool reversed;
  PolyXY poly;
  Pin(uint8_t pin, const char *key, int16_t threshold, bool reversed = false)
      : pin(pin), key(key), threshold(threshold), reversed(reversed) {}
  void begin(Preferences &preferences) {
    loadPoly(poly, (key + POLY_SUFFIX).c_str(), preferences);
    Serial.printf("LOADED: %s:", key);
    for (const auto &[x, y] : poly) {
      Serial.printf(" %d=%d", x, y);
    }
    Serial.println();
  }
  void calibrateDone() { CHECK_VA(storePoly(poly, (key + POLY_SUFFIX).c_str(), preferences), key); }
  void calibrateSet(uint16_t duty, int16_t value) {
    const auto &item = poly.emplace_back(duty, value);
    // Serial.printf("CALIBRATE: %s %d %d=%d\n", key, poly.size(), item.first, item.second);
    std::sort(poly.begin(), poly.end());
  }
  uint16_t calibrateNext() {
    switch (poly.size()) {
    case 0:
      return 512;
    case 1:
      return 0;
    case 2:
      return 1024;
    }
    struct {
      uint16_t v = 0xFFFF;
      float score = 0;
      void consider(float e, int16_t a, int16_t b) {
        const auto m = mid_or_0(a, b);
        if (!m)
          return;
        if (const auto s = e * abs(b - a); s > score) {
          score = s;
          v = m;
        }
      }
    } best;
    bool pf = false;
    float e;
    for (size_t i = 2; i < poly.size(); ++i) {
      const auto &c = poly[i], &p = poly[i - 1], &pp = poly[i - 2];
      const auto xv = lerpi(p.first, c, pp, F2S), v = p.second;
      e = abs(1.f - (float)v / xv);
      if (pf)
        best.consider(e, p.first, pp.first);
      pf = max(c.second, p.second) > threshold;
      if ((i == 2) && (max(p.second, pp.second) > threshold))
        best.consider(e, p.first, pp.first);
    }
    const auto &c = poly.back(), &p = *(poly.rbegin() + 1);
    if (pf and (max(c.second, p.second) > threshold))
      best.consider(e, c.first, p.first);
    return best.v;
  }
};
struct PinPwm : Pin {
  using Base = Pin;
  float value;
  uint16_t duty;
  PinPwm(uint8_t pin, const char *key, bool reversed, float threshold) : Base(pin, key, threshold * 1e3f, reversed) {}
  bool begin(Preferences &preferences) {
    Base::begin(preferences);
    if (!ledcAttach(pin, 39138, 10))
      return false;
    setValue(0);
    return true;
  }
  void setValue(float v) {
    value = v;
    const auto mv = on && !overcurrent ? (int16_t)(v * 1e3f) : 0;
    setDuty(interpoly(mv, poly, S2F, reversed ? DESC : ASC, DiscontinuityCheck::USE_LAST_CONTINUOUS));
  }
  void restoreValue() { setValue(value); }
  void setDuty(int16_t v) {
    duty = constrain(v, 0, 1024);
    Serial.printf("PWM: %s=%d\n", key, v);
    CHECK_VA(ledcWrite(pin, duty), "(%d,%d)", pin, duty);
  }
  void setDutyUnlimited() { setDuty(reversed ? 0 : 1024); }
} pwm_v(10, "pwm-mv", true, 1 /*1.25 is XL4015 lower reliable bound*/), pwm_a(20, "pwm-ma", false, 0);

struct PinAdc : Pin {
  static constexpr float K = 8;
  using Base = Pin;
  PinPwm &pwm;
  float avg;
  PinAdc(uint8_t pin, PinPwm &pwm, const char *key) : Base(pin, key, 0), pwm(pwm) {}
  float getValue() const {
    const auto mv = interpoly((int16_t)(avg * K), poly, F2S, ASC, DiscontinuityCheck::USE_LAST_CONTINUOUS);
    // Serial.printf("GV: %s %f -> %d\n", key, avg, mv);
    return mv / 1e3f;
  }
} adc_v(3, pwm_v, "adc-mv"), adc_a(4, pwm_a, "adc-ma");

#define PIN_CC 8
static bool ocp = true;
void IRAM_ATTR handleOvercurrent() {
  if (!ocp || overcurrent)
    return;
  overcurrent = true;
  pwm_v.restoreValue();
  pwm_a.restoreValue();
}
void checkOvercurrent() {
  if (ocp && digitalRead(PIN_CC))
    handleOvercurrent();
  else {
    overcurrent = false;
    pwm_v.restoreValue();
    pwm_a.restoreValue();
  }
}

struct : public AsyncServer {
  using AsyncServer::_port;
} server(5555);

static PinAdc *autocalibrate = nullptr;
static std::vector<float> adcv;
static float midv, mida;
void calibrate(PinAdc &adc, const std::string &rest) {
  auto &pwm = adc.pwm;
  if (rest == "RESTART") {
    ocp = overcurrent = false;
    adc.poly.clear();
    pwm.poly.clear();
  } else {
    midv = std::stof(rest);
    adcv.clear();
    autocalibrate = &adc;
  }
  if (const auto nd = pwm.calibrateNext(); nd != 0xFFFF)
    pwm.setDuty(nd);
}

std::string ff3n(float f) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.3f\n", f);
  return std::string(buffer);
}
std::initializer_list<std::pair<std::string, std::function<std::string(const std::string &suffix)>>> COMMANDS{
    {"*IDN?\n", [](const auto &) { return "igelbox,EPS1,0,0.1\n"; }},
    {"OUTPUT:CVCC? CH1\n", [](const auto &) { return digitalRead(PIN_CC) ? "CC\n" : "CV\n"; }},
    {"MEASURE:VOLTAGE? CH1\n", [](const auto &) { return ff3n(adc_v.getValue()); }},
    {"SOURCE1:VOLTAGE?\n", [](const auto &) { return ff3n(pwm_v.value); }},
    {"MEASURE:CURRENT? CH1\n", [](const auto &) { return ff3n(adc_a.getValue()); }},
    {"SOURCE1:CURRENT?\n", [](const auto &) { return ff3n(pwm_a.value); }},

    {"SOURCE1:CURRENT:PROTECTION:STATE?\n", [](const auto &) { return ocp ? "ON\n" : "OFF\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:STATE ",
     [](const auto &rest) {
       ocp = rest == "ON";
       checkOvercurrent();
       return "";
     }},
    {"SOURCE1:CURRENT:PROTECTION:LEVEL ",
     [](const auto &rest) {
       const auto level = std::stof(rest);
       if (level != pwm_a.value)
         Serial.printf("OCPL: %f != %f\n", level, pwm_a.value);
       return "";
     }},
    {"SOURCE1:CURRENT:PROTECTION:TRIPPED?\n", [](const auto &) { return overcurrent ? "YES\n" : "NO\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:CLEAR",
     [](const auto &) {
       checkOvercurrent();
       return "";
     }},

    {"OUTPUT? CH1\n", [](const auto &) { return on ? "ON\n" : "OFF\n"; }},
    {"OUTPUT CH1,",
     [](const auto &rest) {
       on = rest == "ON";
       pwm_v.restoreValue();
       pwm_a.restoreValue();
       return "";
     }},
    {"SOURCE1:VOLTAGE ",
     [](const auto &rest) {
       pwm_v.setValue(std::stof(rest));
       return "";
     }},
    {"SOURCE1:CURRENT ",
     [](const auto &rest) {
       pwm_a.setValue(std::stof(rest));
       return "";
     }},

    {"CALIBRATE:OUTPUT:VOLTAGE CH1,",
     [](const auto &rest) {
       pwm_a.setDutyUnlimited();
       calibrate(adc_v, rest);
       return "";
     }},
    {"CALIBRATE:OUTPUT:CURRENT CH1,",
     [](const auto &rest) {
       pwm_v.setDutyUnlimited();
       calibrate(adc_a, rest);
       return "";
     }},
};

void setup() {
  pinMode(pwm_v.pin, OUTPUT);
  digitalWrite(pwm_v.pin, HIGH);
  pinMode(PIN_CC, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_CC), handleOvercurrent, RISING);

  setCpuFrequencyMhz(80); // reduce heating a bit
  delay(500);             // give a time to connect monitor
  Serial.begin(115200);

  ASSERT(preferences.begin("epsu"));
  for (const auto pwm : {&pwm_v, &pwm_a})
    ASSERT_VA(pwm->begin(preferences), "(%d)", pwm->pin);
  for (const auto adc : {&adc_v, &adc_a})
    adc->begin(preferences);
  {
    analogContinuousSetAtten(ADC_0db);
    uint8_t pins[2] = {adc_v.pin, adc_a.pin};
    ASSERT(analogContinuous(pins, 2, 4092 / 2 / SOC_ADC_DIGI_RESULT_BYTES, 1024, nullptr));
    ASSERT(analogContinuousStart());
  }

  for (auto s = WiFi.begin(WIFI_SSID, WIFI_PASW); s != WL_CONNECTED; s = WiFi.status()) {
    Serial.print(s);
    delay(1000);
  }
  CHECK(WiFi.setTxPower(WIFI_POWER_2dBm));
  Serial.printf("TxPower: %d\n", WiFi.getTxPower());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  CHECK(MDNS.begin("epsu"));
  Serial.println("mDNS responder started");

  server.onClient(
      [](void *arg, AsyncClient *client) {
        Serial.printf("\nNew client connected: %p ", client);
        Serial.print(client->remoteIP());
        Serial.print(":");
        Serial.println(client->remotePort());

        using Buffer = std::string;
        const auto buffer = new Buffer();
        try {
          client->onDisconnect(
              [](void *arg, AsyncClient *client) {
                Serial.printf("\ndisconnected: %p\n", client);
                // Free up resources
                delete (Buffer *)arg;
                delete client;
              },
              (void *)buffer);
        } catch (...) {
          delete buffer;
          throw;
        }
        client->onData(
            [](void *arg, AsyncClient *client, void *data, size_t len) {
              // Serial.printf("\nData received from client %p %p [%d bytes]:", client, arg, len);
              const auto chars = (const char *)data;
              // for (size_t i = 0; i < len; i++) {
              //   Serial.print((char)chars[i]);
              // }
              // Serial.println();
              auto &buffer = *(Buffer *)arg;
              buffer.append(chars, len);

              if (const auto i = buffer.find('\n'); i != std::string::npos) {
                // Serial.printf("Command: '%s'", buffer.substr(0, i).c_str());
                bool found = false;
                for (const auto &[prefix, command] : COMMANDS) {
                  const auto &prefix_length = prefix.length();
                  if (prefix_length > i) {
                    if ((prefix_length == i + 1) && (prefix[i] == '\n')) {
                      // full command
                    } else
                      continue;
                  }
                  if (buffer.starts_with(prefix)) {
                    found = true;
                    // Serial.printf(" as %s\n", prefix.c_str());
                    const auto &response = command(buffer.substr(prefix_length, i - prefix_length));
                    if (!response.empty()) {
                      client->write(response.c_str(), response.length());
                    }
                  }
                }
                if (!found) {
                  Serial.printf("Unknown: %s\n", buffer.substr(0, i).c_str());
                }
                buffer.erase(0, i + 1);
              }

              if (!buffer.empty()) {
                Serial.printf("Buffer: '%s'\n", buffer.c_str());
              }
            },
            (void *)buffer);
        client->onError(
            [](void *arg, AsyncClient *client, int8_t error) {
              Serial.printf("\nConnection error for client %p. Error: %d\n", client, error);
            },
            nullptr);
      },
      nullptr);

  server.begin();
  Serial.printf("TCP Server started on port %d\n", server._port);
}

void processAutoCalibration(PinAdc &pin) {
  adcv.emplace_back(pin.avg);
  if (adcv.size() == 1) {
    return; // dirty reading
  }
  if (adcv.size() < (13 - (int)log2f(pin.avg))) {
    return;
  }
  float avg = 0.f;
  for (size_t i = 1; i < adcv.size(); ++i) {
    avg += adcv[i];
  }
  avg /= adcv.size() - 1;
  float v = midv;
  if (pin.poly.empty()) {
    mida = avg;
  } else {
    v *= avg / mida;
  }
  const auto a8 = (int16_t)(avg * PinAdc::K);
  const auto mv = (int16_t)(v * 1e3f);
  pin.calibrateSet(a8, mv);
  auto &pwm = pin.pwm;
  pwm.calibrateSet(pwm.duty, mv);
  Serial.printf("CA: %d %d %f\n", pwm.duty, a8, v);
  if (const auto nd = pwm.calibrateNext(); (nd != 0xFFFF) && (pwm.poly.size() < 16)) {
    pwm.setDuty(nd);
    adcv.clear();
    return;
  }
  Serial.printf("CALIBRATED: %s %d\n", pin.key, pin.poly.size());
  pin.calibrateDone();
  pwm.calibrateDone();
  pwm_a.restoreValue();
  pwm_v.restoreValue();
  autocalibrate = nullptr;
}
void processAdcResults(const adc_continuous_results_t &results) {
  for (const auto adc : {&adc_v, &adc_a}) {
    const auto &result = results[digitalPinToAnalogChannel(adc->pin)];
    // Serial.printf("%d\t%d\t%d\t%d\n", pin, digitalPinToAnalogChannel(pin), result.count, result.sum_read_raw);
    adc->avg = (float)result.sum_read_raw / (float)result.count;
  }
  Serial.printf("%d: u=%f\ti=%f\n", millis(), adc_v.avg, adc_a.avg);
  if (const auto pin = autocalibrate)
    processAutoCalibration(*pin);
}
void loop() {
  adc_continuous_results_t results;
  if (CHECK(analogContinuousReadSumCount(results, 1000)))
    processAdcResults(results);
}
