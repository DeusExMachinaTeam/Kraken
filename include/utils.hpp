#ifndef __KRAKEN_UTILS_HPP__
#define __KRAKEN_UTILS_HPP__

template <typename T>
inline T clamp(T v, T min, T max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
};

#endif