//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

namespace uhd { namespace usrp {

/**
 * A recursive least squares implementation
 */
class rls {

private:
    // Slope of the prediction equation
    double m;
    // Offset of the prediction equation
    double b;
    /**
     * Forgetting factor
     * Keep between 0.95 and 0.995
     * A higher forgetting factor makes it less sensitive to noise but also slower
     * It is often represented by lamda (λ)
     */
    double _forgetting_factor;

public:
    rls(double initial_m_guess, double initial_b_guess, double initial_variance, double forgetting_factor);

    /**
     * Update rls with a new result pair
     * @param y The measured output value
     * @param x The input value used for said measured value
     */
    void update(double y, double x);

    /**
     * Predict the output for a given input
     * @param x The input value
     * @return The predicted output value
     */
    inline double predict(double x) {
        // TODO: implement
        return 0;
    }

};


}} // namespace uhd::usrp
