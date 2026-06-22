//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/swmr.hpp>

using namespace uhd;

template <typename T>
swmr<T>::swmr() noexcept {
    write_count.store(0, std::memory_order_release);
}

template <typename T>
swmr<T>::swmr(T data) noexcept {
    // We can do a relaxed store here because we do a release during store
    write_count.store(0, std::memory_order_relaxed);
    store(&data);
}

template <typename T>
void swmr<T>::store(T data) {
    // Increment write counter to ensure the reader thread knows a write has started
    write_count.fetch_add(1, std::memory_order_relaxed);

    /**
     * Release fence to ensure write_count has been updated
     * A fence is required to prevent writes to _data from being reordered earlier,
     * which making the fetch_add memory_order_release alone would not do
     */
    std::atomic_thread_fence(std::memory_order_release);

    _data = data;

    // Fence to ensure _data is done being written to before write_count is updated
    std::atomic_thread_fence(std::memory_order_release);

    // Increment the write counter so the consumer knows we are done writting
    write_count.fetch_add(1, std::memory_order_release);
}

template <typename T>
void swmr<T>::load(T* dst) {
    int64_t intial_write_count = 0;
    T loaded_data;
    int64_t end_write_count = 0;

    do {
        // Get the value of write_count at the start of copying
        intial_write_count = write_count.load(std::memory_order_relaxed);

        // Fence to ensure reads of _data do not get reordered before reads for intial_write_count
        std::atomic_thread_fence(std::memory_order_acquire);

        memcpy(dst, &_data, sizeof(T));

        // Fence to ensure the read for end_write_count does not get reordered before copying _data
        std::atomic_thread_fence(std::memory_order_acquire);

        end_write_count= write_count.load(std::memory_order_relaxed);

    } while (
        // Repeat load if the class was updated while loading
        intial_write_count != end_write_count ||
        // Repeat load if the class is mid update (write_count is odd)
        (intial_write_count & 0x1) );

}
