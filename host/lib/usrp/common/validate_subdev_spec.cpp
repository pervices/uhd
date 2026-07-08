//
// Copyright 2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhdlib/usrp/common/validate_subdev_spec.hpp>
#include <format>
#include <sstream>

using namespace uhd;
using namespace uhd::usrp;

void uhd::usrp::validate_subdev_spec(property_tree::sptr tree,
    const subdev_spec_t& spec,
    const std::string& type,
    const std::string& mb)
{
    const size_t num_dsps =
        tree->list(std::format("/mboards/{}/{}_dsps", mb, type)).size();

    // sanity checking on the length
    if (spec.empty())
        throw uhd::value_error(
            std::format("Empty {} subdevice specification is not supported.\n",
                type));
    if (spec.size() > num_dsps)
        throw uhd::value_error(
            std::format("The subdevice specification \"{}\" is too long.\n"
                              "The user specified {} channels, but there are only {} {} "
                              "dsps on mboard {}.\n",
                spec.to_string(), spec.size(), num_dsps, type, mb));

    // make a list of all possible specs
    subdev_spec_t all_specs;
    for (const std::string& db :
        tree->list(std::format("/mboards/{}/dboards", mb))) {
        for (const std::string& sd :
            tree->list(
                std::format("/mboards/{}/dboards/{}/{}_frontends", mb, db, type))) {
            all_specs.push_back(subdev_spec_pair_t(db, sd));
        }
    }

    // validate that the spec is possible
    for (const subdev_spec_pair_t& pair : spec) {
        uhd::assert_has(all_specs,
            pair,
            std::format("{} subdevice specification on mboard {}", type, mb));
    }

    // enable selected frontends, disable others
    for (const subdev_spec_pair_t& pair : all_specs) {
        const bool enb = uhd::has(spec, pair);
        tree->access<bool>(
                std::format("/mboards/{}/dboards/{}/{}_frontends/{}/enabled", mb,
                    pair.db_name, type, pair.sd_name))
            .set(enb);
    }
}
