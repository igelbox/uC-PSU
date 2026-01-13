#include <gtest/gtest.h>

#include "interpoly.hpp"

TEST(Calibrate, next) {
  struct C {
    int16_t key, value;
  };
  std::vector<C> cali{{0, 16}, {512, 1950}, {1024, 3980}};
  EXPECT_EQ(Calibrator(&C::key, &C::value, 0).next(cali), std::optional<int16_t>{256});
}
