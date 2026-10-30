// Minimal assertion helpers so the tests need no external framework.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

#define CHECK_STR(actual, expected)                                                                        \
  do {                                                                                                     \
    const std::string actual_ = (actual);                                                                  \
    const std::string expected_ = (expected);                                                              \
    if (actual_ != expected_) {                                                                            \
      std::fprintf(stderr, "%s:%d: CHECK_STR failed\n  actual:   %s\n  expected: %s\n", __FILE__, __LINE__, \
                   actual_.c_str(), expected_.c_str());                                                    \
      std::exit(1);                                                                                        \
    }                                                                                                      \
  } while (false)

#define RUN(test)                          \
  do {                                     \
    test();                                \
    std::printf("PASS %s\n", #test);       \
  } while (false)
