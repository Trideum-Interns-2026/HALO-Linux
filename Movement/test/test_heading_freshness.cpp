// Logic tests for the heading staleness guard (no ROS, no MAVSDK).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "movement/heading_freshness.hpp"

using movement::HeadingFreshness;
using movement::sanitizeHeadingTimeout;

namespace {
constexpr double kMaxAge = 1.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

TEST(HeadingFreshness, FirstSampleIsFresh) {
    HeadingFreshness f(kMaxAge);
    const auto h = f.update(123.0, 1000, 50.0);
    ASSERT_TRUE(h.has_value());
    EXPECT_DOUBLE_EQ(*h, 123.0);
}

TEST(HeadingFreshness, AdvancingTimestampsStayFreshForever) {
    HeadingFreshness f(kMaxAge);
    for (int i = 0; i < 100; ++i) {
        const auto h = f.update(10.0 + i, 1000 + 100000ULL * i, 5.0 + 0.9 * i);
        ASSERT_TRUE(h.has_value()) << i;
        EXPECT_DOUBLE_EQ(*h, 10.0 + i);
    }
}

TEST(HeadingFreshness, ExactlyMaxAgeIsStillFresh) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(90.0, 1000, 10.0).has_value());
    EXPECT_TRUE(f.update(90.0, 1000, 11.0).has_value());
}

TEST(HeadingFreshness, JustOverMaxAgeIsStale) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(90.0, 1000, 10.0).has_value());
    EXPECT_FALSE(f.update(90.0, 1000, 11.001).has_value());
}

TEST(HeadingFreshness, StaysStaleUntilTheTimestampAdvances) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(90.0, 1000, 10.0).has_value());
    EXPECT_FALSE(f.update(90.0, 1000, 12.0).has_value());
    EXPECT_FALSE(f.update(90.0, 1000, 20.0).has_value());
    const auto h = f.update(95.0, 2000, 20.1);  // recovery
    ASSERT_TRUE(h.has_value());
    EXPECT_DOUBLE_EQ(*h, 95.0);
    EXPECT_TRUE(f.update(95.0, 2000, 21.1).has_value());   // the age counts from the recovery
    EXPECT_FALSE(f.update(95.0, 2000, 21.2).has_value());
}

TEST(HeadingFreshness, NonFiniteHeadingIsNeverReturned) {
    HeadingFreshness f(kMaxAge);
    EXPECT_FALSE(f.update(kNaN, 1000, 1.0).has_value());
    EXPECT_FALSE(f.update(kInf, 2000, 1.1).has_value());
    EXPECT_FALSE(f.update(-kInf, 3000, 1.2).has_value());
    // A non-finite sample is not recorded: a good sample afterwards is still a first sample.
    EXPECT_TRUE(f.update(45.0, 3000, 1.3).has_value());
}

TEST(HeadingFreshness, NonFiniteHeadingDoesNotKeepAStaleStreamAlive) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(45.0, 1000, 1.0).has_value());
    EXPECT_FALSE(f.update(kNaN, 2000, 1.5).has_value());
    EXPECT_FALSE(f.update(45.0, 1000, 2.5).has_value());  // timestamp never advanced with a value
}

TEST(HeadingFreshness, ResetForgetsTheStreamAndTheNextSampleIsFirst) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(90.0, 1000, 10.0).has_value());
    ASSERT_FALSE(f.update(90.0, 1000, 20.0).has_value());
    f.reset();
    // Same timestamp as before the reset, but it is a new first sample.
    const auto h = f.update(90.0, 1000, 30.0);
    ASSERT_TRUE(h.has_value());
    EXPECT_DOUBLE_EQ(*h, 90.0);
    EXPECT_FALSE(f.update(90.0, 1000, 31.5).has_value());
}

TEST(HeadingFreshness, ResetBeforeAnySampleIsHarmless) {
    HeadingFreshness f(kMaxAge);
    f.reset();
    EXPECT_TRUE(f.update(1.0, 5, 0.0).has_value());
}

TEST(HeadingFreshness, BackwardsTimestampCountsAsANewSample) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(10.0, 5000000, 10.0).has_value());
    EXPECT_FALSE(f.update(10.0, 5000000, 12.0).has_value());  // stale
    const auto h = f.update(20.0, 1000, 12.1);  // flight controller rebooted: clock restarted
    ASSERT_TRUE(h.has_value());
    EXPECT_DOUBLE_EQ(*h, 20.0);
}

TEST(HeadingFreshness, ZeroTimestampStreamThatNeverAdvancesGoesStale) {
    HeadingFreshness f(kMaxAge);
    ASSERT_TRUE(f.update(0.0, 0, 1.0).has_value());
    EXPECT_FALSE(f.update(0.0, 0, 2.5).has_value());
}

TEST(HeadingFreshness, SmallAndLargeLimitsAreHonoured) {
    HeadingFreshness tight(movement::kMinHeadingTimeoutS);
    ASSERT_TRUE(tight.update(1.0, 1, 0.0).has_value());
    EXPECT_TRUE(tight.update(1.0, 1, 0.1).has_value());
    EXPECT_FALSE(tight.update(1.0, 1, 0.2).has_value());

    HeadingFreshness loose(movement::kMaxHeadingTimeoutS);
    ASSERT_TRUE(loose.update(1.0, 1, 0.0).has_value());
    EXPECT_TRUE(loose.update(1.0, 1, 10.0).has_value());
    EXPECT_FALSE(loose.update(1.0, 1, 10.5).has_value());
}

TEST(HeadingFreshness, NonFiniteLimitFailsSafeAsStale) {
    HeadingFreshness f(kNaN);
    ASSERT_TRUE(f.update(1.0, 1, 0.0).has_value());  // a new sample is always fresh
    EXPECT_FALSE(f.update(1.0, 1, 0.0).has_value());
}

// ---------- heading_timeout_s sanitising (main.cpp reads the parameter) ----------

TEST(HeadingTimeoutSanitize, DefaultIsOneSecondWithinTheLimits) {
    EXPECT_DOUBLE_EQ(movement::kDefaultHeadingTimeoutS, 1.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(movement::kDefaultHeadingTimeoutS), 1.0);
}

TEST(HeadingTimeoutSanitize, ClampsToTheRangeAndKeepsTheEdges) {
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(0.1), 0.1);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(10.0), 10.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(0.0), 0.1);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(-5.0), 0.1);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(0.0999), 0.1);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(10.001), 10.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(1e9), 10.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(2.5), 2.5);
}

TEST(HeadingTimeoutSanitize, NonFiniteFallsBackToTheDefault) {
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(kNaN), 1.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(kInf), 1.0);
    EXPECT_DOUBLE_EQ(sanitizeHeadingTimeout(-kInf), 1.0);
}
