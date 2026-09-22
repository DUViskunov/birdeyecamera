// Минимальный каркас тестов: код возврата и понятное сообщение об ошибке.
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

namespace wztest {

inline int& failures() {
    static int n = 0;
    return n;
}

inline void check(bool ok, const char* expr, const char* file, int line) {
    if (ok) return;
    ++failures();
    std::printf("  ПРОВАЛ %s:%d  %s\n", file, line, expr);
}

inline void checkNear(double a, double b, double tol, const char* expr, const char* file,
                      int line) {
    if (std::fabs(a - b) <= tol) return;
    ++failures();
    std::printf("  ПРОВАЛ %s:%d  %s  (%.9g против %.9g, допуск %.3g)\n", file, line, expr, a, b,
                tol);
}

inline int report(const char* suite) {
    if (failures() == 0) {
        std::printf("[OK]   %s\n", suite);
        return 0;
    }
    std::printf("[СБОЙ] %s: провалов %d\n", suite, failures());
    return 1;
}

}  // namespace wztest

#define CHECK(expr) wztest::check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) wztest::checkNear((a), (b), (tol), #a " ~ " #b, __FILE__, __LINE__)
