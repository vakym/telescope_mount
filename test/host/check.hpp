/**
 * @file check.hpp
 * @brief Минимальный фреймворк проверок без внешних зависимостей.
 */

#pragma once

#include <cmath>
#include <cstdio>

namespace check {

inline int &failures() noexcept
{
    static int value = 0;
    return value;
}

inline void fail(const char *expr, const char *file, int line)
{
    ++failures();
    std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
}

inline bool near(double a, double b, double tolerance) noexcept
{
    return std::fabs(a - b) <= tolerance;
}

}  // namespace check

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            ::check::fail(#cond, __FILE__, __LINE__);                     \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tolerance)                                        \
    do {                                                                   \
        const double check_a_ = static_cast<double>(a);                    \
        const double check_b_ = static_cast<double>(b);                    \
        if (!::check::near(check_a_, check_b_, static_cast<double>(tolerance))) { \
            ::check::fail("near(" #a ", " #b ")", __FILE__, __LINE__);     \
        }                                                                  \
    } while (0)

#define CHECK_GT(a, b)                                                     \
    do {                                                                   \
        const double check_a_ = static_cast<double>(a);                    \
        const double check_b_ = static_cast<double>(b);                    \
        if (!(check_a_ > check_b_)) {                                      \
            ::check::fail("(" #a " > " #b ")", __FILE__, __LINE__);        \
        }                                                                  \
    } while (0)

#define CHECK_GE(a, b)                                                     \
    do {                                                                   \
        const double check_a_ = static_cast<double>(a);                    \
        const double check_b_ = static_cast<double>(b);                    \
        if (!(check_a_ >= check_b_)) {                                     \
            ::check::fail("(" #a " >= " #b ")", __FILE__, __LINE__);       \
        }                                                                  \
    } while (0)

#define CHECK_LT(a, b)                                                     \
    do {                                                                   \
        const double check_a_ = static_cast<double>(a);                    \
        const double check_b_ = static_cast<double>(b);                    \
        if (!(check_a_ < check_b_)) {                                      \
            ::check::fail("(" #a " < " #b ")", __FILE__, __LINE__);        \
        }                                                                  \
    } while (0)

#define TEST_RESULT() (::check::failures() == 0 ? 0 : 1)
