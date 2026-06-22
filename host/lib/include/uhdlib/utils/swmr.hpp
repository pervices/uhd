//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <cstdint>
#include <atomic>

namespace uhd {

    /**
     * Fast single writer multiple reader interface for arbitary structs
     *
     * Currently the access is controlled through a sequence lock,
     * although there is no guarantee it will always be controlled through one
     */
    template <typename T>
    class swmr {

    private:

        // The data we are providing a swmr for
        T _data;

        /*
         * The number of times the class has been written to
         * Store operations increment the count by 1 at the start and end.
         * If it's odd a write is in progress
         */
        std::atomic<int64_t> write_count;


    public:

        /**
         * The default initialize to 0
         */
        explicit swmr() noexcept {
            write_count.store(0, std::memory_order_release);
        }

        /**
         * Initialize to store existing data
         */
        explicit swmr(T data) noexcept {
            // We can do a relaxed store here because we do a release during store
            write_count.store(0, std::memory_order_relaxed);
            store(data);
        }

        /**
         * NOTE: this is thread safe against loads, not against multiple writers
         */
        inline void store(T data) {
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

        inline void load(T* dst) {
            int64_t intial_write_count = 0;
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

        inline T load() {
            T r;
            load(&r);
            return r;
        }

        /**
         * Delete copy and move operators to avoid accidental non thread safe copies and moves
         */

        // Delete copy constructor
        swmr(const swmr&) = delete;
        // Delete copy assignment
        swmr& operator=(const swmr&) = delete;
        // Delete move constructor
        swmr(swmr&&) = delete;
        // Delete move assignment
        swmr& operator=(swmr&&) = delete;
    };

} // namepspace uhd
