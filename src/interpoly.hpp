#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

using XY = std::pair<int16_t, int16_t>;
using Direction = std::pair<int16_t XY::*, int16_t XY::*>;
inline static const Direction F2S{&XY::first, &XY::second}, S2F{&XY::second, &XY::first};
using PolyXY = std::vector<XY>;
using Order = std::function<bool(int16_t, int16_t)>;
inline static const Order ASC{std::less<int16_t>()}, DESC{std::greater<int16_t>()};

template <typename T> auto lerpi(int16_t x, const T &a, const T &b, const int16_t T::*from, const int16_t T::*to) {
  return static_cast<int16_t>( //
      static_cast<int32_t>(x - a.*from) * (b.*to - a.*to) / (b.*from - a.*from) + a.*to);
}

enum class DiscontinuityCheck { IGNORE, USE_LAST_CONTINUOUS };
auto interpoly(int16_t x, const PolyXY &poly, const Direction &direction, const Order &order,
               const DiscontinuityCheck discontinuity) {
  if (poly.empty())
    return x;

  const auto &[from, to] = direction;
  const auto &begin = poly.cbegin();
  auto it = begin;
  for (; it != poly.cend(); ++it) {
    if ((discontinuity == DiscontinuityCheck::USE_LAST_CONTINUOUS) && (it != begin) &&
        !order(*(it - 1).*from, *it.*from))
      break; // discountinuity
    if ((it == begin) || order(*it.*from, x))
      continue;
    return lerpi(x, *it, *(it - 1), from, to);
  }
  return it == (begin + 1) ? *begin.*to : lerpi(x, *(it - 2), *(it - 1), from, to);
}

template <typename T> struct Calibrator {
  const int16_t T::*key, T::*value;
  const int16_t threshold;
  Calibrator(int16_t T::*key, int16_t T::*value, int16_t threshold) : key(key), value(value), threshold(threshold) {}

  auto next(const std::vector<T> &poly) {
    struct {
      std::optional<int16_t> key = std::nullopt;
      float score = 0;
      void consider(float e, int16_t a, int16_t b) {
        const auto d = (b - a) / 2, m = a + d;
        if (!d || (m == b))
          return;
        if (const auto s = e * abs(b - a); s > score) {
          score = s;
          key = m;
        }
      }
    } best;
    bool pf = false;
    float e;
    for (size_t i = 2; i < poly.size(); ++i) {
      const auto &c = poly[i], &p = poly[i - 1], &pp = poly[i - 2];
      const auto xv = lerpi(p.*key, c, pp, key, value), v = p.*value;
      e = abs(1.f - (float)v / xv);
      if (pf)
        best.consider(e, p.*key, pp.*key);
      pf = std::max(c.*value, p.*value) > threshold;
      if ((i == 2) && (std::max(p.*value, pp.*value) > threshold))
        best.consider(e, p.*key, pp.*key);
    }
    const auto &c = poly.back(), &p = *(poly.rbegin() + 1);
    if (pf and (std::max(c.*value, p.*value) > threshold))
      best.consider(e, c.*key, p.*key);
    return best.key;
  }
};
