#pragma once

#define CHECK(expr) CHECK_VA(expr, "")
#define CHECK_VA(expr, ...)                                                                                            \
  ((expr) || [&]() {                                                                                                   \
    Serial.printf("!CHECK: %s", #expr);                                                                                \
    Serial.printf(__VA_ARGS__);                                                                                        \
    Serial.print('\n');                                                                                                \
    return false;                                                                                                      \
  }())

#define ASSERT(expr) ASSERT_VA(expr, "")
#define ASSERT_VA(expr, ...)                                                                                           \
  if (!CHECK_VA(expr, __VA_ARGS__)) {                                                                                  \
    Serial.println("!HALT");                                                                                           \
    Serial.flush();                                                                                                    \
    for (;;)                                                                                                           \
      ;                                                                                                                \
  }
