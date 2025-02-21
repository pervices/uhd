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
    };
}

void uhd::set_thread_priority(float priority, bool realtime)
{

    if(realtime) {
        // Realtime threading has been disabled
        // SCHED_DEADLINE prevents setting thread affinity, which is more important
        // SCHED_FIFO and SCHED_RR result in worse performance, even after setting /proc/sys/kernel/sched_rt_runtime_us to -1
        set_thread_priority_realtime(priority);

        // To achieve the effect desired by realtime threading without actually using realtime threading:
        //     Adjusting priority range:
        //         If realtime threading is requested shift priority from range -1..1 to 0.5..1.
        //         In non realtime mode shift 0..1 to 0..0.5 and don't affect -1..0
        //         This achieves the effect of always having a higher priority than non realtime threads, while not affecting negative priority of realtime threads
    } else if (realtime) {
        priority = ((priority + 1) * 0.25) + 0.5;
        check_priority_range(priority);

        set_thread_priority_non_realtime(priority);
    }else {
        // Shift priority range so pseudo realtime threading can always be higher
        if(priority >0) {
            priority = (priority * 0.5);
        }
        check_priority_range(priority);

        set_thread_priority_non_realtime(priority);
    }
}

void uhd::set_thread_priority_realtime(float priority) {
    // Priority is no used in deadlines, the parameter is a legacy of previous schedueling
    (void) priority;

    struct sched_attr attr;
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_flags = 0x01/*SCHED_FLAG_RESET_ON_FORK*/; // Documentation says to use SCHED_FLAG_RESET_ON_FORK, but it doesn't seem to be declared. Required to allow this thread to create child thread in deadline mode
    // Nice is not used when using realtime threads
    attr.sched_nice = 0;
    // Priority is not used in deadline mode
    attr.sched_priority = 0;
    // Runtime, deadline, and priority should be equal so it uses 100% of time
    attr.sched_runtime = 120000000000;
    attr.sched_deadline = 120000000000;
    attr.sched_period = 120000000000;

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


void uhd::set_thread_affinity_active_core() {
    int cpu = get_cpu();
    if(cpu < 0) {
        UHD_LOG_ERROR("UHD", "Unable to set affinity to current core due to being unable to check the current core");
    }
    std::vector<size_t> target_cpu(1, (size_t) cpu);
    uhd::set_thread_affinity(target_cpu);
}


int uhd::get_cpu() {
    unsigned int cpu;
    // Syscall used because getcpu is does not exist on Oracle
    int r = syscall(SYS_getcpu, &cpu, nullptr);
    if(r) {
        UHD_LOG_ERROR("UHD", "Failed to get current core");
        return -1;
    } else {
        return (int) cpu;
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
