#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include ".config.hpp"
#include "assert.hpp"
#include "esp32-hal-adc-hack.hpp"

bool on = true;

auto pmap(double v, const std::initializer_list<double> &poly) {
  double x = 1.0, result = 0.0;
  for (auto it = std::rbegin(poly); it != std::rend(poly); ++it) {
    result += *it * x;
    x *= v;
  }
  return result;
}

void set(uint8_t pin, float v, const std::initializer_list<double> &poly) {
  const auto pwm = (int)constrain(pmap(v, poly), 0, 1024);
  Serial.printf("PWM: %d=%d(%f)\n", pin, pwm, v);
  CHECK_VA(ledcWrite(pin, pwm), "(%d,%d)", pin, pwm);
}

#define PIN_PWM_V 10
float set_v = 0;

void setVoltage(float v) {
  set_v = v;
  set(PIN_PWM_V, on ? v : 0.f, {-3.77264919e-02, 9.84725460e-01, -9.29032821e+01, 8.00313668e+02});
}

#define PIN_PWM_I 20
float set_i = 0;
void setCurrent(float v) {
  set_i = v;
  set(PIN_PWM_I, v, {268.5754725, -0.91707882});
}

#define PIN_ADC_I 4
float avg_i = 0;
#define PIN_ADC_V 3
float avg_v = 0;

struct : public AsyncServer {
  using AsyncServer::_port;
} server(5555);

std::string ff3n(float f) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.3f\n", f);
  return std::string(buffer);
}
std::initializer_list<std::pair<std::string, std::function<std::string(const std::string &suffix)>>> COMMANDS{
    {"*IDN?\n", [](const auto &) { return "igelbox,EPS1,0,0.1\n"; }},
    {"OUTPUT:CVCC? CH1\n", [](const auto &) { return "CV\n"; }}, // TODO
    {"MEASURE:VOLTAGE? CH1\n",
     [](const auto &) {
       return ff3n(pmap(avg_v, {1.26900834e-14, -8.12377947e-11, 1.81774135e-07, 4.32187213e-03, -6.37734292e-01}));
     }},
    {"SOURCE1:VOLTAGE?\n", [](const auto &) { return ff3n(set_v); }},
    {"MEASURE:CURRENT? CH1\n",
     [](const auto &) {
       return ff3n(pmap(avg_i, {1.53524393e-18, -4.16563790e-15, 4.28800185e-12, -2.12585388e-09, 5.39631775e-07,
                                -7.03940880e-05, 8.52150124e-03, -4.70666463e-02}));
     }},
    {"SOURCE1:CURRENT?\n", [](const auto &) { return ff3n(set_i); }},

    // TODO
    {"SOURCE1:CURRENT:PROTECTION:STATE?\n", [](const auto &) { return "OFF\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:TRIPPED?\n", [](const auto &) { return "NO\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:", [](const auto &) { return ""; }},

    {"OUTPUT? CH1\n", [](const auto &) { return on ? "ON\n" : "OFF\n"; }},
    {"OUTPUT CH1,",
     [](const auto &rest) {
       on = rest == "ON";
       setVoltage(set_v);
       return "";
     }},
    {"SOURCE1:VOLTAGE ",
     [](const auto &rest) {
       setVoltage(std::stof(rest));
       return "";
     }},
    {"SOURCE1:CURRENT ",
     [](const auto &rest) {
       setCurrent(std::stof(rest));
       return "";
     }},
};

void setup() {
  pinMode(PIN_PWM_V, OUTPUT);
  digitalWrite(PIN_PWM_V, HIGH);

  setCpuFrequencyMhz(80); // reduce heating a bit
  delay(500);             // give a time to connect monitor
  Serial.begin(115200);

  for (const auto pin : std::initializer_list<int>{PIN_PWM_V, PIN_PWM_I}) {
    ASSERT_VA(ledcAttach(pin, 39138, 10), "(%d)", pin);
    pin == PIN_PWM_V ? setVoltage(0) : setCurrent(0);
  }

  {
    analogContinuousSetAtten(ADC_0db);
    uint8_t pins[2] = {PIN_ADC_V, PIN_ADC_I};
    ASSERT(analogContinuous(pins, 2, 4092 / 2 / SOC_ADC_DIGI_RESULT_BYTES, 1024, nullptr));
    ASSERT(analogContinuousStart());
  }

  for (auto s = WiFi.begin(WIFI_SSID, WIFI_PASW); s != WL_CONNECTED; s = WiFi.status()) {
    Serial.print(s);
    delay(1000);
  }
  WiFi.setTxPower(WIFI_POWER_2dBm);
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

void loop() {
  adc_continuous_results_t results;
  if (CHECK(analogContinuousReadSumCount(results, 1000))) {
    for (const auto [pin, avg] : std::initializer_list<std::pair<uint8_t, float &>>{
             {PIN_ADC_V, avg_v},
             {PIN_ADC_I, avg_i},
         }) {
      const auto &result = results[digitalPinToAnalogChannel(pin)];
      // Serial.printf("%d\t%d\t%d\t%d\n", pin, digitalPinToAnalogChannel(pin), result.count, result.sum_read_raw);
      avg = (float)result.sum_read_raw / (float)result.count;
    }
    Serial.printf("%d: u=%f\ti=%f\n", millis(), avg_v, avg_i);
  }
}
