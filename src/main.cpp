#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include ".config.hpp"

bool on = true;

auto fmap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

void set(uint8_t pin, float v, float mn, float mx) {
  const auto pwm = (int)constrain(fmap(v, mn, mx, 0, 1024), 0, 1024);
  // Serial.printf("PWM: %d=%d(%f)\n", pin, pwm, v);
  if (!ledcWrite(pin, pwm)) {
    Serial.printf("!ledcWrite(%d,%d)\n", pin, pwm);
    Serial.flush();
    return;
  }
}

#define PIN_PWM_V 0
float set_v = 0;

void setVoltage(float v) {
  set_v = v;
  set(PIN_PWM_V, on ? v : 0.f, 12.08341336f, 0.8334513536f);
}

#define PIN_PWM_I 1
float set_i = 0;
void setCurrent(float v) {
  set_i = v;
  set(PIN_PWM_I, v, 0.007613493153f, 3.234381453f);
}

#define PIN_ADC_I 2
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
    {"MEASURE:VOLTAGE? CH1\n", [](const auto &) { return ff3n(fmap(avg_v, 0, 65536, 0.0511026945, 155.3751534)); }},
    {"SOURCE1:VOLTAGE?\n", [](const auto &) { return ff3n(set_v); }},
    {"MEASURE:CURRENT? CH1\n", [](const auto &) { return ff3n(fmap(avg_i, 64, 65536, 0.2526465391, 293.8658984)); }},
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
    if (!ledcAttach(pin, 39138, 10)) {
      Serial.printf("!ledcAttach: %d\n", pin);
      Serial.flush();
      for (;;)
        ;
    }
    pin == PIN_PWM_V ? setVoltage(0) : setCurrent(0);
  }

  {
    analogContinuousSetAtten(ADC_0db);
    uint8_t pins[2] = {PIN_ADC_V, PIN_ADC_I};
    if (!analogContinuous(pins, 2, 4092 / 2 / SOC_ADC_DIGI_RESULT_BYTES, 1024, nullptr)) {
      Serial.printf("!analogContinuous\n");
      Serial.flush();
      for (;;)
        ;
    }
    if (!analogContinuousStart()) {
      Serial.printf("!analogContinuousStart\n");
      Serial.flush();
      for (;;)
        ;
    }
  }

  for (auto s = WiFi.begin(WIFI_SSID, WIFI_PASW); s != WL_CONNECTED; s = WiFi.status()) {
    Serial.print(s);
    delay(1000);
  }
  WiFi.setTxPower(WIFI_POWER_2dBm);
  Serial.printf("TxPower: %d\n", WiFi.getTxPower());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  if (!MDNS.begin("epsu")) {
    Serial.println("!Error setting up mDNS responder");
  }
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
  adc_continuous_data_t *result = nullptr;
  if (analogContinuousRead(&result, 1000)) { // TODO: I use patched one to avoid integer division
    for (int i = 0; i < 2; i++) {
      if (result[i].pin == PIN_ADC_V)
        avg_v = result[i].avg_read_raw / 511.f;
      else
        avg_i = result[i].avg_read_raw / 511.f;
    }
    Serial.printf("%d, ADC PIN data: u=%f\ti=%f\n", millis(), avg_v, avg_i);
  } else {
    Serial.println("!analogContinuousRead");
  }
}
