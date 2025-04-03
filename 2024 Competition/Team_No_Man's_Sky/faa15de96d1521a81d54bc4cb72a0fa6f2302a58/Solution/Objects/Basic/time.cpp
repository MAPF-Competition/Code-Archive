#include <Objects/Basic/time.hpp>

TimePoint get_now() {
    return std::chrono::steady_clock::now();
}

ETimer::ETimer() : start(get_now()) {
}

uint64_t ETimer::get_ms() const {
    return std::chrono::duration_cast<Milliseconds>(get_now() - start).count();
}

uint64_t ETimer::get_ns() const {
    return std::chrono::duration_cast<Nanoseconds>(get_now() - start).count();
}

void ETimer::reset() {
    start = get_now();
}

std::ostream &operator<<(std::ostream &output, const ETimer &time) {
    double t = time.get_ns() * 1e-9;
    if (t >= 1) {
        output << t << "s";
    } else if (t >= 1e-3) {
        output << t * 1e3 << "ms";
    } else if (t >= 1e-6) {
        output << t * 1e6 << "us";
    } else if (t >= 1e-9) {
        output << t * 1e9 << "ns";
    } else {
        output << "0s";
    }
    return output;
}
