//
// Copyright 2026 Per Vices Corporation
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhdlib/utils/rls.hpp>


using namespace uhd;
using namespace uhd::usrp;

rls::rls(double initial_m_guess, double initial_b_guess, double initial_variance, double forgetting_factor)
:
m(initial_m_guess),
b(initial_b_guess),
_forgetting_factor(forgetting_factor),
p11(initial_variance), p12(initial_variance),
p21(initial_variance), p22(initial_variance)
{

}
