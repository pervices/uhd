//
// Copyright 2010-2011,2015 Ettus Research LLC
// Copyright 2018-2020 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//


#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/thread.hpp>
#include <vector>

#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

bool uhd::set_thread_priority_safe(float priority, bool realtime)
{
    try {
        set_thread_priority(priority, realtime);
        return true;
    } catch (const uhd::access_error &e) {
        UHD_LOGGER_WARNING("UHD") << "Unable to set thread priority due to insufficient permission, performance will be negatively affected. Run the program with sudo or add the required permissions to this user";
        return false;
    } catch (const uhd::os_error& e) {
        UHD_LOGGER_WARNING("UHD")
            << "Unable to set the thread priority. Performance may be "
               "negatively affected.\n"
               "Please see the general application notes in the manual for "
               "instructions.\n"
            << e.what();
        return false;
    }
}

static void check_priority_range(float priority)
{
    if (priority > +1.0 or priority < -1.0) {
        throw uhd::value_error("priority out of range [-1.0, +1.0]");
    }
}

/***********************************************************************
 * Pthread API to set priority
 **********************************************************************/
#ifdef HAVE_PTHREAD_SETSCHEDPARAM
#    include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <string.h>
#include <errno.h>

namespace uhd {
    // From: https://man7.org/linux/man-pages/man2/sched_setattr.2.html
    struct sched_attr {
        uint32_t size;              /* Size of this structure */
        uint32_t sched_policy;      /* Policy (SCHED_*) */
        uint64_t sched_flags;       /* Flags */
        int32_t sched_nice;        /* Nice value (SCHED_OTHER,
                                    SCHED_BATCH) */
        uint32_t sched_priority;    /* Static priority (SCHED_FIFO,
                                    SCHED_RR) */
        /* Remaining fields are for SCHED_DEADLINE */
        uint64_t sched_runtime;
        uint64_t sched_deadline;
        uint64_t sched_period;

        /* Utilization hints */
        uint32_t sched_util_min;
        uint32_t sched_util_max;
    };
}

void uhd::set_thread_priority(float priority, bool realtime)
{
    check_priority_range(priority);

    // Set thread affinity because it would normally be a side effect of setting realtime priority and realtime priority was requested
    uint32_t current_core = 0;
    // getcpu wrapper is implemented in libc 2.29
    // Oracle 8 uses libc 2.28, so a direct syscall is required
    int r = syscall(SYS_getcpu, &current_core, NULL);
    if(r == 0) {
        std::vector<size_t> current_core_v(1, (size_t) current_core);
        set_thread_affinity(current_core_v);
    } else {
        UHD_LOG_WARNING("UHD", "Unable to get current cpu num while setting thread affinity. errno: " + std::string(strerror(errno)));
    }


    if(realtime) {
        set_thread_priority_realtime(priority);
    } else {
        set_thread_priority_non_realtime(priority);
    }
}

void uhd::set_thread_priority_realtime(float priority) {
    // Priority is no used in deadlines, the parameter is a legacy of previous schedueling
    (void) priority;

    struct sched_attr attr;
    attr.size = sizeof(attr);
    // SCHED_DEADLINE is used since SCHED_RR and SCHED_FIFO have limits on what % of the time they can run
    attr.sched_policy = SCHED_FIFO;
    //attr.sched_flags = 0 /*0x01*//*SCHED_FLAG_RESET_ON_FORK*/ | 0x20 /*SCHED_FLAG_UTIL_CLAMP_MIN*/ | 0x40 /*SCHED_FLAG_UTIL_CLAMP_MAX*/; // Documentation says to use SCHED_FLAG_RESET_ON_FORK, but it doesn't seem to be declared. Required to allow this thread to create child thread in deadline mode
    attr.sched_flags = 0
    // Nice is not used when using realtime threads
    attr.sched_nice = 0;
    // Priority is not used in deadline mode
    attr.sched_priority = 1;
    // Runtime, deadline, and priority should be equal so it uses 100% of time
    attr.sched_runtime = 0;
    attr.sched_deadline = 0;
    attr.sched_period = 0;
    attr.sched_util_min = 1024;
    // Allows for 100% usage of the core
    attr.sched_util_max = 1024;

    int ret = syscall(SYS_sched_setattr, getpid(), &attr, 0);

    if (ret != 0) {
        if(errno == EPERM) {
            throw uhd::access_error("error in pthread_setschedparam SCHED_DEADLINE: " + std::string(strerror(errno)));
        } else {
            throw uhd::os_error("error in pthread_setschedparam SCHED_DEADLINE: " + std::string(strerror(errno)));
        }
    }
}

void uhd::set_thread_priority_non_realtime(float priority) {

    int target_niceness = - ::round(priority * 20);
    // Nicenesss is in a range of -20 to 19, to keep priority 0 as neutral the value is mapped to -20 to 20 then capped
    if(target_niceness == 20) {
        target_niceness = 19;
    }

    int policy = SCHED_OTHER;

    // set the new priority and policy
    sched_param sp;

    // Only realtime scheduling has priority levels
    sp.sched_priority = 0;
    int ret = pthread_setschedparam(pthread_self(), policy, &sp);

    if (ret != 0) {
        throw uhd::os_error("error in pthread_setschedparam SCHED_OTHER: " + std::string(strerror(errno)));
    }
    int current_niceness = nice(0);

    int new_niceness(target_niceness - current_niceness);

    // Nice returns -1 to indicated an errors, however -1 is also a very reasonable return value
    // Workaround required to verify niceness can be set
    if(new_niceness == -1 ) {
        // Attempt to reduce niceness further (verifying it can be done)
        int test_niceness = nice(-1);
        // If niceness still -1 an error occured
        if(test_niceness == -1) {
            throw uhd::os_error("error in nice (thread priority): " + std::string(strerror(errno)));
        } else {
            // return niceness to intended value
            nice(1);
        }
    }
}
#endif /* HAVE_PTHREAD_SETSCHEDPARAM */

/***********************************************************************
 * Pthread API to set affinity
 **********************************************************************/
#ifdef HAVE_PTHREAD_SETAFFINITY_NP
#    include <pthread.h>
void uhd::set_thread_affinity(const std::vector<size_t>& cpu_affinity_list)
{
    if (cpu_affinity_list.empty()) {
        return;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    for (auto cpu_num : cpu_affinity_list) {
        if (cpu_num > CPU_SETSIZE) {
            UHD_LOG_WARNING(
                "UHD", "CPU index " << cpu_num << " in affinity list out of range");
        }
        CPU_SET(cpu_num, &cpu_set);
    }

    int status = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpu_set);
    if (status != 0) {
        UHD_LOG_WARNING("UHD", "Failed to set desired affinity for thread");
    }
}
#endif /* HAVE_PTHREAD_SETAFFINITYNP */

/***********************************************************************
 * Windows API to set priority
 **********************************************************************/
#ifdef HAVE_WIN_SETTHREADPRIORITY
#    include <windows.h>

void uhd::set_thread_priority(float priority, UHD_UNUSED(bool realtime))
{
    std::cout << "WIN sp.sched_priority: " << sp.sched_priority << std::endl;
    check_priority_range(priority);

    /*
     * Process wide priority is no longer set.
     * This is the responsibility of the application.
    //set the priority class on the process
    int pri_class = (realtime)? REALTIME_PRIORITY_CLASS : NORMAL_PRIORITY_CLASS;
    if (SetPriorityClass(GetCurrentProcess(), pri_class) == 0)
        throw uhd::os_error("error in SetPriorityClass");
     */

    // scale the priority value to the constants
    int priorities[] = {THREAD_PRIORITY_IDLE,
        THREAD_PRIORITY_LOWEST,
        THREAD_PRIORITY_BELOW_NORMAL,
        THREAD_PRIORITY_NORMAL,
        THREAD_PRIORITY_ABOVE_NORMAL,
        THREAD_PRIORITY_HIGHEST,
        THREAD_PRIORITY_TIME_CRITICAL};
    size_t pri_index = size_t((priority + 1.0) * 6 / 2.0); // -1 -> 0, +1 -> 6

    // set the thread priority on the thread
    if (SetThreadPriority(GetCurrentThread(), priorities[pri_index]) == 0)
        throw uhd::os_error("error in SetThreadPriority");
}
#endif /* HAVE_WIN_SETTHREADPRIORITY */

/***********************************************************************
 * Windows API to set affinity
 **********************************************************************/
#ifdef HAVE_WIN_SETTHREADAFFINITYMASK
#    include <windows.h>
void uhd::set_thread_affinity(const std::vector<size_t>& cpu_affinity_list)
{
    if (cpu_affinity_list.empty()) {
        return;
    }

    DWORD_PTR cpu_set{0};
    for (auto cpu_num : cpu_affinity_list) {
        if (cpu_num > 8 * sizeof(DWORD_PTR)) {
            UHD_LOG_WARNING(
                "UHD", "CPU index " << cpu_num << " in affinity list out of range");
        }
        cpu_set |= ((DWORD_PTR)1 << cpu_num);
    }

    DWORD_PTR status = SetThreadAffinityMask(GetCurrentThread(), cpu_set);
    if (status == 0) {
        UHD_LOG_WARNING("UHD", "Failed to set desired affinity for thread");
    }
}
#endif /* HAVE_WIN_SETTHREADAFFINITYMASK */

/***********************************************************************
 * Unimplemented API to set priority
 **********************************************************************/
#ifdef HAVE_THREAD_PRIO_DUMMY
void uhd::set_thread_priority(float, bool)
{
    std::cout << "DUMMY sp.sched_priority: " << sp.sched_priority << std::endl;
    UHD_LOG_DEBUG("UHD", "Setting thread priority is not implemented");
}

#endif /* HAVE_THREAD_PRIO_DUMMY */

/***********************************************************************
 * Unimplemented API to set affinity
 **********************************************************************/
#ifdef HAVE_THREAD_SETAFFINITY_DUMMY
void uhd::set_thread_affinity(const std::vector<size_t>& cpu_affinity_list)
{
    UHD_LOG_DEBUG("UHD", "Setting thread affinity is not implemented");
}
#endif /* HAVE_THREAD_SETAFFINITY_DUMMY */

void uhd::set_thread_name(boost::thread* thrd, const std::string& name)
{
#ifdef HAVE_PTHREAD_SETNAME
    pthread_setname_np(thrd->native_handle(), name.substr(0, 16).c_str());
#endif /* HAVE_PTHREAD_SETNAME */
#ifdef HAVE_THREAD_SETNAME_DUMMY
    // Then we can't set the thread name. This function may get called
    // before the logger starts, and thus can't log any error messages.
    // Note that CMake will also tell the user about not being able to set
    // thread names.
#endif /* HAVE_THREAD_SETNAME_DUMMY */
}

void uhd::set_thread_name(std::thread* thrd, const std::string& name)
{
#ifdef HAVE_PTHREAD_SETNAME
    pthread_setname_np(thrd->native_handle(), name.substr(0, 16).c_str());
#endif /* HAVE_PTHREAD_SETNAME */
#ifdef HAVE_THREAD_SETNAME_DUMMY
    // Then we can't set the thread name. This function may get called
    // before the logger starts, and thus can't log any error messages.
    // Note that CMake will also tell the user about not being able to set
    // thread names.
#endif /* HAVE_THREAD_SETNAME_DUMMY */
}
