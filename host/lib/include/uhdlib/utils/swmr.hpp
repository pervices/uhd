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

        // --- 1. LIFE-CYCLE CONSTRUCTORS ---

        /**
         * The default initialize to 0
         */
        explicit swmr() noexcept;

        /**
         * Initialize to store existing data
         */
        explicit swmr(T data) noexcept;

        /**
         * NOTE: this is thread safe against loads, not against multiple writers
         */
        void store(T dst);

        void load(T* dst);

        // --- 2. DELETED STRUCTURAL COPY/MOVE ---
        // Moving or copying multi-member structures results in data tearing
        swmr(const swmr&) = delete;
        swmr(swmr&&) = delete;

        // --- 3. DELETED COMPILER-GENERATED DEFAULT ASSIGNMENTS & CASTS ---
        // Prevents direct raw object assignments or dangerous structural downgrades
        swmr& operator=(const swmr&) = delete;

        // --- 4. DELETED MEMORY POINTER ALIASING ---
        // Direct addresses let threads modify individual fields bypassing internal sync
        swmr* operator&() = delete;
        const swmr* operator&() const = delete;
        swmr* operator->() = delete;
        swmr& operator*() = delete;

        // --- 5. DELETED READ-MODIFY-WRITE (RMW) OPERATORS ---
        // Math operations on a multi-type structure are inherently unaligned and unsafe
        swmr operator++(int) = delete;
        swmr operator--(int) = delete;
        swmr& operator++()    = delete;
        swmr& operator--()    = delete;

        swmr& operator+=(const swmr&) = delete;
        swmr& operator-=(const swmr&) = delete;

        // --- 6. DELETED CONDITIONAL EVALUATIONS ---
        // Block multi-step branches which cause race conditions
        bool operator==(const swmr&) const  = delete;
        bool operator!=(const swmr&) const  = delete;
        bool operator<(const swmr&) const   = delete;
        bool operator<=(const swmr&) const  = delete;
        bool operator>(const swmr&) const   = delete;
        bool operator>=(const swmr&) const  = delete;
    };

} // namepspace uhd
