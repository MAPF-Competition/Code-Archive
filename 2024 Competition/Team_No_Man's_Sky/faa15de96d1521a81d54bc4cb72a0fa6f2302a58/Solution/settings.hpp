#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>

// поиск по [KEK], чтобы найти всякие моменты

//#define ENABLE_DEFAULT_PLANNER

//#define ENABLE_DEFAULT_SCHEDULER

//#define ENABLE_TRIVIAL_SCHEDULER

//#define ENABLE_SMART_PLANNER

//#define ENABLE_ASSERT

#define ENABLE_PIBTS

//#define ENABLE_PMPS

//#define ENABLE_DHM

//#define ENABLE_DHMR

#define ENABLE_GG

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
//#define BUILD_META_INFO

#define PRINT(args...) // do { args } while(0)

#define ENABLE_PIBTS_ANNEALING

#define ENABLE_PIBTS_TRICK

//#define ENABLE_PHANTOM_SCHEDULE

//#define ENABLE_SCHEDULER_TRICK

#define ENABLE_SCHEDULER_CHANGE_TASK

//#define ENABLE_DEFAULT_SCHEDULER_TRICK

//#define ENABLE_GG_SOLVER

#define DISABLE_AGENTS

//#define DISABLE_LATE_AGENTS

//#define ENABLE_GUIDANCE_PATH_PLANNER

static constexpr uint32_t MAX_CONST = 10'000'000;

static constexpr uint32_t THREADS = 32;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = -1;

static constexpr uint32_t DHM_REBUILD_TIMELIMIT = 0;

static constexpr uint32_t DHM_REBUILD_COUNT = 0;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = 350;

static constexpr uint32_t SCHEDULER_TRIV_SOLVE_TIME = 150;

static constexpr uint32_t SCHEDULER_TRICK_TIME = 350;

static constexpr uint32_t SCHEDULER_LNS_TIME = 0;

static constexpr uint32_t INVALID_DIST = 0;

static constexpr uint32_t PHANTOM_AGENT_AVAILABLE_DIST = 30;

static constexpr uint32_t MAX_AGENTS_NUM = 10'000;

uint32_t &get_unique_id();

#define ENABLE_FILEPRINT

struct Printer {
    [[nodiscard]] std::ofstream &get() const;
};

template<typename T>
Printer operator<<(Printer printer, const T &value) {
#ifdef ENABLE_FILEPRINT
    printer.get() << value;
    std::cout << value;
#else
    std::cout << value;
#endif
    return printer;
}
