// Logic tests for movement/target_geometry.hpp: no ROS, no MAVSDK, no sim.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "movement/follow_logic.hpp"
#include "movement/target_geometry.hpp"

using namespace movement;

namespace {

constexpr double kDegToRad = kPi / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Only the fields the geometry uses, so this compiles whatever else CameraGeometry holds.
CameraGeometry camera(double center_x = 960.0, double width = 1920.0, double focal_x = 960.0) {
    CameraGeometry c;
    c.center_x_px = center_x;
    c.width_px = width;
    c.focal_x_px = focal_x;
    return c;
}

// Test-side WGS84 radii, written out separately from the header.
double meridianRadius(double lat_deg) {
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;
    const double s = std::sin(lat_deg * kDegToRad);
    return a * (1.0 - e2) / std::pow(1.0 - e2 * s * s, 1.5);
}
double primeVerticalRadius(double lat_deg) {
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;
    const double s = std::sin(lat_deg * kDegToRad);
    return a / std::sqrt(1.0 - e2 * s * s);
}

// Person `north_m` / `east_m` from `drone` (local flat-earth inverse, fine for tens of metres).
GeoPoint offsetFrom(const GeoPoint& drone, double north_m, double east_m) {
    GeoPoint p = drone;
    p.latitude_deg += north_m / meridianRadius(drone.latitude_deg) / kDegToRad;
    p.longitude_deg += east_m /
                       (primeVerticalRadius(drone.latitude_deg) *
                        std::cos(drone.latitude_deg * kDegToRad)) /
                       kDegToRad;
    return p;
}

// Person at `range_m` on compass bearing `bearing_deg` from the drone.
GeoPoint personAt(const GeoPoint& drone, double bearing_deg, double range_m) {
    return offsetFrom(drone, range_m * std::cos(bearing_deg * kDegToRad),
                      range_m * std::sin(bearing_deg * kDegToRad));
}

double haversineM(double lat1, double lon1, double lat2, double lon2, double radius_m) {
    const double dlat = (lat2 - lat1) * kDegToRad;
    const double dlon = (lon2 - lon1) * kDegToRad;
    const double h = std::sin(dlat / 2) * std::sin(dlat / 2) +
                     std::cos(lat1 * kDegToRad) * std::cos(lat2 * kDegToRad) *
                         std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2.0 * radius_m * std::asin(std::sqrt(h));
}

const GeoPoint kHome{34.7, -86.6, 200.0};

}  // namespace

// ---------------------------------------------------------------------------------------------
// LocalOffset

class LocalOffsetLat : public ::testing::TestWithParam<double> {};

TEST_P(LocalOffsetLat, KnownNorthDistance) {
    const double lat = GetParam();
    const GeoPoint drone{lat, 10.0, 0.0};
    const GeoPoint person = offsetFrom(drone, 100.0, 0.0);
    const auto o = localOffsetNorthEast(drone, person);
    ASSERT_TRUE(o.has_value());
    EXPECT_NEAR(o->first, 100.0, 1e-3);
    EXPECT_NEAR(o->second, 0.0, 1e-6);
}

TEST_P(LocalOffsetLat, KnownEastDistance) {
    const double lat = GetParam();
    const GeoPoint drone{lat, 10.0, 0.0};
    const GeoPoint person = offsetFrom(drone, 0.0, 100.0);
    const auto o = localOffsetNorthEast(drone, person);
    ASSERT_TRUE(o.has_value());
    EXPECT_NEAR(o->first, 0.0, 1e-6);
    EXPECT_NEAR(o->second, 100.0, 1e-3);
}

TEST_P(LocalOffsetLat, DegreeLengthsMatchPublishedTables) {
    // Published WGS84 metres per degree; 0.01 deg offsets scaled by 100.
    const double lat = GetParam();
    const GeoPoint drone{lat, 10.0, 0.0};
    const auto n = localOffsetNorthEast(drone, GeoPoint{lat + 0.01, 10.0, 0.0});
    const auto e = localOffsetNorthEast(drone, GeoPoint{lat, 10.01, 0.0});
    ASSERT_TRUE(n.has_value());
    ASSERT_TRUE(e.has_value());
    if (lat == 0.0) {
        EXPECT_NEAR(n->first * 100.0, 110574.27, 1.0);
        EXPECT_NEAR(e->second * 100.0, 111319.49, 1.0);
    } else if (lat == 60.0) {
        EXPECT_NEAR(n->first * 100.0, 111412.84, 1.0);
        EXPECT_NEAR(e->second * 100.0, 55799.97, 1.0);
    }
    // Any latitude: matches the independent radius formulas.
    EXPECT_NEAR(n->first * 100.0, meridianRadius(lat + 0.005) * kDegToRad, 1.0);
    EXPECT_NEAR(e->second * 100.0,
                primeVerticalRadius(lat) * std::cos(lat * kDegToRad) * kDegToRad, 1.0);
}

TEST_P(LocalOffsetLat, SwappingThePointsNegatesTheOffset) {
    const double lat = GetParam();
    const GeoPoint a{lat, 20.0, 5.0};
    const GeoPoint b{lat + 0.0003, 20.0004, 50.0};
    const auto ab = localOffsetNorthEast(a, b);
    const auto ba = localOffsetNorthEast(b, a);
    ASSERT_TRUE(ab.has_value());
    ASSERT_TRUE(ba.has_value());
    EXPECT_NEAR(ab->first, -ba->first, 1e-9);
    EXPECT_NEAR(ab->second, -ba->second, 1e-9);
}

TEST_P(LocalOffsetLat, AgreesWithHaversineWithin1cmAt100m) {
    // Haversine on a sphere with the local ellipsoid radius of the direction travelled.
    const double lat = GetParam();
    const GeoPoint drone{lat, 10.0, 0.0};
    const GeoPoint north = offsetFrom(drone, 100.0, 0.0);
    const GeoPoint east = offsetFrom(drone, 0.0, 100.0);
    const auto on = localOffsetNorthEast(drone, north);
    const auto oe = localOffsetNorthEast(drone, east);
    ASSERT_TRUE(on.has_value());
    ASSERT_TRUE(oe.has_value());
    const double hn = haversineM(drone.latitude_deg, drone.longitude_deg, north.latitude_deg,
                                 north.longitude_deg, meridianRadius(lat + 0.0005));
    const double he = haversineM(drone.latitude_deg, drone.longitude_deg, east.latitude_deg,
                                 east.longitude_deg, primeVerticalRadius(lat));
    EXPECT_NEAR(std::hypot(on->first, on->second), hn, 0.01);
    EXPECT_NEAR(std::hypot(oe->first, oe->second), he, 0.01);
    // And against the plain spherical haversine (R = 6371 km) to 0.6 %: the sphere is only that good.
    const double sphere = haversineM(drone.latitude_deg, drone.longitude_deg, east.latitude_deg,
                                     east.longitude_deg, 6371000.0);
    EXPECT_NEAR(std::hypot(oe->first, oe->second), sphere, 0.6);
}

INSTANTIATE_TEST_SUITE_P(Latitudes, LocalOffsetLat, ::testing::Values(0.0, 34.7, 60.0));

TEST(LocalOffset, SamePositionIsZero) {
    const auto o = localOffsetNorthEast(kHome, kHome);
    ASSERT_TRUE(o.has_value());
    EXPECT_DOUBLE_EQ(o->first, 0.0);
    EXPECT_DOUBLE_EQ(o->second, 0.0);
}

TEST(LocalOffset, EastShrinksWithCosineOfLatitude) {
    const auto eq = localOffsetNorthEast(GeoPoint{0.0, 0.0, 0.0}, GeoPoint{0.0, 0.001, 0.0});
    const auto hi = localOffsetNorthEast(GeoPoint{60.0, 0.0, 0.0}, GeoPoint{60.0, 0.001, 0.0});
    ASSERT_TRUE(eq.has_value());
    ASSERT_TRUE(hi.has_value());
    EXPECT_NEAR(hi->second / eq->second, 0.5, 0.5 * 0.003);
}

TEST(LocalOffset, AltitudeDoesNotChangeTheOffset) {
    GeoPoint high = kHome;
    high.altitude_m += 500.0;
    const GeoPoint person = offsetFrom(kHome, 30.0, -40.0);
    const auto a = localOffsetNorthEast(kHome, person);
    const auto b = localOffsetNorthEast(high, person);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_DOUBLE_EQ(a->first, b->first);
    EXPECT_DOUBLE_EQ(a->second, b->second);
}

TEST(LocalOffset, AntimeridianGoesTheShortWay) {
    const GeoPoint west_of_line{10.0, 179.9999, 0.0};
    const GeoPoint east_of_line{10.0, -179.9999, 0.0};
    const auto o = localOffsetNorthEast(west_of_line, east_of_line);
    ASSERT_TRUE(o.has_value());
    // 0.0002 deg of longitude eastward, about 22 m, not 360 degrees westward.
    EXPECT_NEAR(o->first, 0.0, 1e-6);
    EXPECT_GT(o->second, 20.0);
    EXPECT_LT(o->second, 24.0);
    const auto back = localOffsetNorthEast(east_of_line, west_of_line);
    ASSERT_TRUE(back.has_value());
    EXPECT_NEAR(back->second, -o->second, 1e-9);
}

TEST(LocalOffset, NearThePoleStaysFinite) {
    const auto close = localOffsetNorthEast(GeoPoint{89.9999, 0.0, 0.0},
                                            GeoPoint{89.9999, 90.0, 0.0});
    ASSERT_TRUE(close.has_value());
    EXPECT_TRUE(std::isfinite(close->first));
    EXPECT_TRUE(std::isfinite(close->second));
    const auto at_pole = localOffsetNorthEast(GeoPoint{90.0, 0.0, 0.0},
                                              GeoPoint{90.0, 180.0, 0.0});
    ASSERT_TRUE(at_pole.has_value());
    EXPECT_TRUE(std::isfinite(at_pole->first));
    EXPECT_TRUE(std::isfinite(at_pole->second));
    const auto south = localOffsetNorthEast(GeoPoint{-90.0, 0.0, 0.0},
                                            GeoPoint{-89.9999, 45.0, 0.0});
    ASSERT_TRUE(south.has_value());
    EXPECT_TRUE(std::isfinite(south->first));
    EXPECT_TRUE(std::isfinite(south->second));
}

TEST(LocalOffset, NonFiniteOrOutOfRangeGivesNothing) {
    const std::vector<GeoPoint> bad = {
        {kNaN, 0.0, 0.0},   {0.0, kNaN, 0.0},   {0.0, 0.0, kNaN},   {kInf, 0.0, 0.0},
        {0.0, -kInf, 0.0},  {0.0, 0.0, kInf},   {90.001, 0.0, 0.0}, {-90.001, 0.0, 0.0},
        {0.0, 180.001, 0.0}, {0.0, -180.001, 0.0}};
    for (const GeoPoint& b : bad) {
        EXPECT_FALSE(localOffsetNorthEast(kHome, b).has_value())
            << b.latitude_deg << " " << b.longitude_deg << " " << b.altitude_m;
        EXPECT_FALSE(localOffsetNorthEast(b, kHome).has_value())
            << b.latitude_deg << " " << b.longitude_deg << " " << b.altitude_m;
    }
}

TEST(LocalOffset, ExactLimitsAreAccepted) {
    EXPECT_TRUE(localOffsetNorthEast(GeoPoint{90.0, 180.0, 0.0}, GeoPoint{-90.0, -180.0, 0.0})
                    .has_value());
}

// ---------------------------------------------------------------------------------------------
// Bearing

TEST(Bearing, CardinalDirections) {
    EXPECT_NEAR(worldBearingDeg(1.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(worldBearingDeg(0.0, 1.0), 90.0, 1e-12);
    EXPECT_NEAR(worldBearingDeg(-1.0, 0.0), 180.0, 1e-12);
    EXPECT_NEAR(worldBearingDeg(0.0, -1.0), -90.0, 1e-12);
    EXPECT_NEAR(worldBearingDeg(1.0, 1.0), 45.0, 1e-12);
    EXPECT_NEAR(worldBearingDeg(-1.0, -1.0), -135.0, 1e-12);
}

TEST(Bearing, Wrap180Range) {
    EXPECT_DOUBLE_EQ(wrap180(180.0), 180.0);
    EXPECT_DOUBLE_EQ(wrap180(-180.0), 180.0);
    EXPECT_DOUBLE_EQ(wrap180(181.0), -179.0);
    EXPECT_DOUBLE_EQ(wrap180(-181.0), 179.0);
    EXPECT_DOUBLE_EQ(wrap180(360.0), 0.0);
    EXPECT_DOUBLE_EQ(wrap180(720.0 + 10.0), 10.0);
    EXPECT_DOUBLE_EQ(wrap180(0.0), 0.0);
    EXPECT_TRUE(std::isnan(wrap180(kNaN)));
}

TEST(Bearing, RelativeBearingWrapsAcrossNorth) {
    EXPECT_NEAR(relativeBearingDeg(10.0, 350.0), 20.0, 1e-12);
    EXPECT_NEAR(relativeBearingDeg(350.0, 10.0), -20.0, 1e-12);
    EXPECT_NEAR(relativeBearingDeg(-10.0, 10.0), -20.0, 1e-12);
    EXPECT_NEAR(relativeBearingDeg(0.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(relativeBearingDeg(90.0, 90.0), 0.0, 1e-12);
}

TEST(Bearing, RelativeBearingExactlyOppositeIsPlus180) {
    EXPECT_DOUBLE_EQ(relativeBearingDeg(180.0, 0.0), 180.0);
    EXPECT_DOUBLE_EQ(relativeBearingDeg(0.0, 180.0), 180.0);
    EXPECT_DOUBLE_EQ(relativeBearingDeg(-90.0, 90.0), 180.0);
    EXPECT_DOUBLE_EQ(relativeBearingDeg(270.0, 90.0), 180.0);
    EXPECT_DOUBLE_EQ(relativeBearingDeg(0.0, 0.0 + 180.0 + 360.0), 180.0);
}

TEST(Bearing, RelativeBearingStaysInRange) {
    for (double world = -360.0; world <= 360.0; world += 7.5) {
        for (double heading = 0.0; heading < 360.0; heading += 22.5) {
            const double rel = relativeBearingDeg(world, heading);
            EXPECT_GT(rel, -180.0);
            EXPECT_LE(rel, 180.0);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// PixelX

TEST(PixelX, CentredIsThePrincipalPoint) {
    EXPECT_DOUBLE_EQ(targetPixelX(0.0, camera()), 960.0);
    // Off-centre principal point.
    EXPECT_DOUBLE_EQ(targetPixelX(0.0, camera(951.3, 1920.0, 1000.0)), 951.3);
}

TEST(PixelX, MatchesThePinholeModel) {
    const CameraGeometry c = camera(960.0, 1920.0, 1000.0);
    for (double rel : {-80.0, -45.0, -10.0, 5.0, 30.0, 60.0}) {
        EXPECT_NEAR(targetPixelX(rel, c), 960.0 + 1000.0 * std::tan(rel * kDegToRad), 1e-9);
    }
    EXPECT_NEAR(targetPixelX(45.0, c), 1960.0, 1e-9);
    EXPECT_NEAR(targetPixelX(-45.0, c), -40.0, 1e-9);
    EXPECT_GT(targetPixelX(10.0, c), targetPixelX(5.0, c));  // right bearing -> right of centre
}

TEST(PixelX, RoundTripsBackToTheBearing) {
    const CameraGeometry c = camera(955.0, 1920.0, 1380.0);
    for (double rel = -88.0; rel <= 88.0; rel += 4.0) {
        const double x = targetPixelX(rel, c);
        const double back = std::atan((x - c.center_x_px) / c.focal_x_px) / kDegToRad;
        EXPECT_NEAR(back, rel, 1e-9) << rel;
    }
}

TEST(PixelX, ClampedAt89DegreesAndFinite) {
    const CameraGeometry c = camera();
    const double at89 = targetPixelX(89.0, c);
    EXPECT_TRUE(std::isfinite(at89));
    EXPECT_DOUBLE_EQ(targetPixelX(90.0, c), at89);
    EXPECT_DOUBLE_EQ(targetPixelX(135.0, c), at89);
    EXPECT_DOUBLE_EQ(targetPixelX(180.0, c), at89);
    const double at_m89 = targetPixelX(-89.0, c);
    EXPECT_TRUE(std::isfinite(at_m89));
    EXPECT_DOUBLE_EQ(targetPixelX(-90.0, c), at_m89);
    EXPECT_DOUBLE_EQ(targetPixelX(-180.0, c), at_m89);
    EXPECT_NEAR(at89, 960.0 + 960.0 * std::tan(89.0 * kDegToRad), 1e-6);
}

TEST(PixelX, TargetBehindLandsFarOutsideTheImage) {
    const CameraGeometry c = camera();
    EXPECT_GT(targetPixelX(180.0, c), c.width_px);
    EXPECT_LT(targetPixelX(-180.0 + 1e-9, c), 0.0);
    EXPECT_GT(targetPixelX(120.0, c), c.width_px);
    EXPECT_LT(targetPixelX(-120.0, c), 0.0);
}

// ---------------------------------------------------------------------------------------------
// Synthesize

TEST(Synthesize, AheadAtRangeIsCentredWithThatRange) {
    const auto s = synthesizeTarget(kHome, personAt(kHome, 0.0, 6.0), 0.0, camera());
    ASSERT_TRUE(s.has_value());
    EXPECT_NEAR(s->x_px, 960.0, 1e-3);
    ASSERT_TRUE(s->distance_m.has_value());
    EXPECT_NEAR(*s->distance_m, 6.0, 1e-3);
}

TEST(Synthesize, HeadingTurnsTheTargetInTheImage) {
    const GeoPoint east = personAt(kHome, 90.0, 8.0);
    const auto facing_north = synthesizeTarget(kHome, east, 0.0, camera());
    const auto facing_east = synthesizeTarget(kHome, east, 90.0, camera());
    const auto facing_south = synthesizeTarget(kHome, east, 180.0, camera());
    ASSERT_TRUE(facing_north.has_value());
    ASSERT_TRUE(facing_east.has_value());
    ASSERT_TRUE(facing_south.has_value());
    // Person due east, nose north: 90 deg right, clamped to 89 deg, off the right edge: no range.
    EXPECT_GT(facing_north->x_px, 1920.0);
    EXPECT_FALSE(facing_north->distance_m.has_value());
    // Nose east: dead ahead.
    EXPECT_NEAR(facing_east->x_px, 960.0, 1e-3);
    ASSERT_TRUE(facing_east->distance_m.has_value());
    EXPECT_NEAR(*facing_east->distance_m, 8.0, 1e-3);
    // Nose south: person to the left, off the image.
    EXPECT_LT(facing_south->x_px, 0.0);
    EXPECT_FALSE(facing_south->distance_m.has_value());
}

TEST(Synthesize, HeadingWrapsAcrossNorth) {
    // Person 20 deg right of the nose whether the nose says 350 or -10.
    const GeoPoint person = personAt(kHome, 10.0, 10.0);
    const auto a = synthesizeTarget(kHome, person, 350.0, camera());
    const auto b = synthesizeTarget(kHome, person, -10.0, camera());
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NEAR(a->x_px, 960.0 + 960.0 * std::tan(20.0 * kDegToRad), 1e-2);
    EXPECT_NEAR(a->x_px, b->x_px, 1e-6);
}

TEST(Synthesize, RangeIsHorizontalAndIgnoresAltitude) {
    GeoPoint person = personAt(kHome, 0.0, 6.0);
    person.altitude_m = kHome.altitude_m + 60.0;  // far above
    const auto above = synthesizeTarget(kHome, person, 0.0, camera());
    person.altitude_m = kHome.altitude_m - 150.0;  // far below
    const auto below = synthesizeTarget(kHome, person, 0.0, camera());
    ASSERT_TRUE(above.has_value());
    ASSERT_TRUE(below.has_value());
    ASSERT_TRUE(above->distance_m.has_value());
    ASSERT_TRUE(below->distance_m.has_value());
    EXPECT_NEAR(*above->distance_m, 6.0, 1e-3);
    EXPECT_NEAR(*below->distance_m, 6.0, 1e-3);
    EXPECT_NEAR(above->x_px, below->x_px, 1e-9);
}

TEST(Synthesize, AtTheImageEdgeStillGivesARange) {
    // fx = 960 = half the width, so +-45 deg is the image edge.
    for (double rel : {44.99, -44.99}) {
        const auto s = synthesizeTarget(kHome, personAt(kHome, rel, 7.0), 0.0, camera());
        ASSERT_TRUE(s.has_value()) << rel;
        EXPECT_GE(s->x_px, 0.0);
        EXPECT_LE(s->x_px, 1920.0);
        ASSERT_TRUE(s->distance_m.has_value()) << rel;
        EXPECT_NEAR(*s->distance_m, 7.0, 1e-3);
    }
}

TEST(Synthesize, JustInsideTheColumnLimitKeepsTheRange) {
    // focal = cx = width / 2, so a 45 deg bearing is the right image edge; stay 1e-6 deg inside
    // it (tan() rounding makes the exact edge unreliable).
    const auto s = synthesizeTarget(kHome, personAt(kHome, 45.0 - 1e-6, 7.0), 0.0, camera());
    ASSERT_TRUE(s.has_value());
    EXPECT_TRUE(s->distance_m.has_value());
}

TEST(Synthesize, JustOutsideTheViewGivesNoRange) {
    for (double rel : {45.5, -45.5, 60.0, -60.0, 89.0, -89.0}) {
        const auto s = synthesizeTarget(kHome, personAt(kHome, rel, 7.0), 0.0, camera());
        ASSERT_TRUE(s.has_value()) << rel;
        EXPECT_FALSE(s->distance_m.has_value()) << rel;
        EXPECT_TRUE(s->x_px < 0.0 || s->x_px > 1920.0) << rel;
    }
}

TEST(Synthesize, BehindTheDroneHasNoRangeAndStaysFinite) {
    for (double bearing : {180.0, 170.0, -170.0, 135.0, -135.0}) {
        const auto s = synthesizeTarget(kHome, personAt(kHome, bearing, 12.0), 0.0, camera());
        ASSERT_TRUE(s.has_value()) << bearing;
        EXPECT_TRUE(std::isfinite(s->x_px)) << bearing;
        EXPECT_FALSE(s->distance_m.has_value()) << bearing;
    }
}

TEST(Synthesize, TooCloseHoldsCentredWithoutRange) {
    for (double range : {0.0, 0.1, 0.4, 0.49}) {
        const auto s = synthesizeTarget(kHome, personAt(kHome, 100.0, range), 0.0, camera());
        ASSERT_TRUE(s.has_value()) << range;
        EXPECT_DOUBLE_EQ(s->x_px, 960.0) << range;
        EXPECT_FALSE(s->distance_m.has_value()) << range;
    }
    const auto same = synthesizeTarget(kHome, kHome, 123.0, camera());
    ASSERT_TRUE(same.has_value());
    EXPECT_DOUBLE_EQ(same->x_px, 960.0);
    EXPECT_FALSE(same->distance_m.has_value());
}

TEST(Synthesize, JustFarEnoughUsesTheBearing) {
    const auto s = synthesizeTarget(kHome, personAt(kHome, 20.0, 0.6), 0.0, camera());
    ASSERT_TRUE(s.has_value());
    EXPECT_NEAR(s->x_px, 960.0 + 960.0 * std::tan(20.0 * kDegToRad), 1e-1);
    ASSERT_TRUE(s->distance_m.has_value());
    EXPECT_NEAR(*s->distance_m, 0.6, 1e-3);
}

TEST(Synthesize, MissingOrNonFiniteHeadingGivesNothing) {
    const GeoPoint person = personAt(kHome, 0.0, 6.0);
    EXPECT_FALSE(synthesizeTarget(kHome, person, std::nullopt, camera()).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, kNaN, camera()).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, kInf, camera()).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, -kInf, camera()).has_value());
}

TEST(Synthesize, UnusableCameraGivesNothing) {
    const GeoPoint person = personAt(kHome, 0.0, 6.0);
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(960.0, 1920.0, 0.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(960.0, 1920.0, -5.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(960.0, 1920.0, kNaN)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(960.0, 0.0, 960.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(960.0, -1.0, 960.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(kNaN, 1920.0, 960.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, camera(kInf, 1920.0, 960.0)).has_value());
    EXPECT_FALSE(synthesizeTarget(kHome, person, 0.0, CameraGeometry{}).has_value());
}

TEST(Synthesize, InvalidPositionsGiveNothing) {
    EXPECT_FALSE(
        synthesizeTarget(kHome, GeoPoint{kNaN, 0.0, 0.0}, 0.0, camera()).has_value());
    EXPECT_FALSE(
        synthesizeTarget(kHome, GeoPoint{95.0, 0.0, 0.0}, 0.0, camera()).has_value());
    EXPECT_FALSE(
        synthesizeTarget(GeoPoint{0.0, 200.0, 0.0}, kHome, 0.0, camera()).has_value());
    EXPECT_FALSE(
        synthesizeTarget(kHome, GeoPoint{34.7, -86.6, kNaN}, 0.0, camera()).has_value());
}

// Sweep: every target direction (0-360 deg, 15 deg steps) and range (0.5-50 m) combined with several
// headings gives a command inside every limit, for follow and for orbit, with the controller
// parameters sanitised the way the node does it, and with wild parameters clamped to the ceilings.
TEST(Synthesize, FollowAndOrbitCommandsStayWithinEveryLimitForASweepOfTargets) {
    FollowParams defaults;
    FollowParams wild;
    wild.max_forward_m_s = 1e9;
    wild.max_strafe_m_s = 1e9;
    wild.max_yaw_rate_deg_s = 1e9;
    wild.forward_gain = 1e9;
    wild.lateral_gain = 1e9;
    wild.yaw_gain = 1e9;
    wild.orbit_yaw_ki = 1e9;
    wild.min_distance_m = -1e9;
    wild.max_distance_m = 1e9;
    wild.safe_distance_m = 1e9;
    OrbitParams orbit_ccw;
    orbit_ccw.enabled = true;
    OrbitParams orbit_cw = orbit_ccw;
    orbit_cw.clockwise = true;
    OrbitParams orbit_wild = orbit_ccw;
    orbit_wild.period_s = 1e-9;
    orbit_wild.max_speed_m_s = 1e9;

    const CameraGeometry cam = camera();
    std::size_t checked = 0;
    for (const FollowParams& raw : {defaults, wild}) {
        const FollowParams p = sanitize(raw);
        for (const OrbitParams& raw_orbit : {orbit_ccw, orbit_cw, orbit_wild}) {
            const OrbitParams o = sanitize(raw_orbit);
            for (double heading : {0.0, 90.0, 200.0, 359.0}) {
                for (double bearing = 0.0; bearing < 360.0; bearing += 15.0) {
                    for (double range : {0.5, 1.0, 3.0, 6.0, 10.0, 25.0, 50.0}) {
                        const auto s = synthesizeTarget(
                            kHome, personAt(kHome, bearing, range), heading, cam);
                        ASSERT_TRUE(s.has_value());
                        ASSERT_TRUE(std::isfinite(s->x_px));

                        const VelocityCommand f =
                            computeFollowCommand(s->x_px, s->distance_m, cam, p);
                        EXPECT_TRUE(std::isfinite(f.forward_m_s));
                        EXPECT_LE(std::fabs(f.forward_m_s), p.max_forward_m_s + 1e-12);
                        EXPECT_LE(std::fabs(f.right_m_s), p.max_strafe_m_s + 1e-12);
                        EXPECT_LE(std::fabs(f.yawspeed_deg_s), p.max_yaw_rate_deg_s + 1e-12);
                        EXPECT_LE(std::fabs(f.forward_m_s), kForwardSpeedCeilingMps);
                        EXPECT_LE(std::fabs(f.right_m_s), kStrafeSpeedCeilingMps);
                        EXPECT_LE(std::fabs(f.yawspeed_deg_s), kYawRateCeilingDegS);
                        if (!s->distance_m) {
                            // No range: never any forward motion.
                            EXPECT_DOUBLE_EQ(f.forward_m_s, 0.0);
                        }

                        OrbitState state;
                        for (int step = 0; step < 20; ++step) {
                            const VelocityCommand c = computeOrbitCommand(
                                s->x_px, s->distance_m, cam, p, o, state, 0.1);
                            EXPECT_TRUE(std::isfinite(c.forward_m_s));
                            EXPECT_TRUE(std::isfinite(c.right_m_s));
                            EXPECT_TRUE(std::isfinite(c.yawspeed_deg_s));
                            EXPECT_LE(std::fabs(c.forward_m_s), p.max_forward_m_s + 1e-12);
                            EXPECT_LE(std::fabs(c.right_m_s),
                                      std::min(o.max_speed_m_s, kOrbitSpeedCeilingMps) + 1e-12);
                            EXPECT_LE(std::fabs(c.yawspeed_deg_s),
                                      p.max_yaw_rate_deg_s + 1e-12);
                            EXPECT_LE(std::fabs(c.yawspeed_deg_s), kYawRateCeilingDegS);
                            if (!s->distance_m) {
                                EXPECT_DOUBLE_EQ(c.forward_m_s, 0.0);
                                EXPECT_DOUBLE_EQ(c.right_m_s, 0.0);
                            }
                        }
                        ++checked;
                    }
                }
            }
        }
    }
    EXPECT_EQ(checked, 2u * 3u * 4u * 24u * 7u);
}

TEST(Synthesize, DirectlyAheadAtTheSafeDistanceCommandsNothing) {
    const FollowParams p = sanitize(FollowParams{});
    const auto s = synthesizeTarget(kHome, personAt(kHome, 0.0, p.safe_distance_m), 0.0, camera());
    ASSERT_TRUE(s.has_value());
    const VelocityCommand c = computeFollowCommand(s->x_px, s->distance_m, camera(), p);
    EXPECT_NEAR(c.forward_m_s, 0.0, 1e-3);
    EXPECT_NEAR(c.right_m_s, 0.0, 1e-3);
    EXPECT_NEAR(c.yawspeed_deg_s, 0.0, 1e-1);
}

// ---------------------------------------------------------------------------------------------
// ErrorGate

TEST(ErrorGate, OffWhenZero) {
    EXPECT_FALSE(rejectTargetError(0.0, 0.0, 0.0, 0.0));
    EXPECT_FALSE(rejectTargetError(1.0, 2.0, 3.0, 0.0));
    EXPECT_FALSE(rejectTargetError(1e9, 1e9, 1e9, 0.0));
}

TEST(ErrorGate, ExactlyAtTheLimitIsAccepted) {
    EXPECT_FALSE(rejectTargetError(3.0, 4.0, 0.0, 5.0));    // norm = 5
    EXPECT_FALSE(rejectTargetError(0.0, 0.0, 7.0, 7.0));
    EXPECT_FALSE(rejectTargetError(2.0, 3.0, 6.0, 7.0));    // norm = 7
}

TEST(ErrorGate, AboveTheLimitIsRejected) {
    EXPECT_TRUE(rejectTargetError(3.0, 4.0, 0.0, 4.999));
    EXPECT_TRUE(rejectTargetError(0.0, 0.0, 7.001, 7.0));
    EXPECT_TRUE(rejectTargetError(2.0, 3.0, 6.0, 6.99));
    EXPECT_TRUE(rejectTargetError(100.0, 0.0, 0.0, 10.0));
}

TEST(ErrorGate, BelowTheLimitIsAccepted) {
    EXPECT_FALSE(rejectTargetError(1.0, 1.0, 1.0, 5.0));
    EXPECT_FALSE(rejectTargetError(0.0, 0.0, 0.0, 0.001));
}

TEST(ErrorGate, UsesTheNormNotTheLargestAxis) {
    // Each axis alone is within 5, the combined norm (about 6.9) is not.
    EXPECT_TRUE(rejectTargetError(4.0, 4.0, 4.0, 5.0));
}

TEST(ErrorGate, NonFiniteErrorIsAlwaysRejected) {
    for (double max_m : {0.0, 5.0, 1e9}) {
        EXPECT_TRUE(rejectTargetError(kNaN, 0.0, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, kNaN, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, 0.0, kNaN, max_m));
        EXPECT_TRUE(rejectTargetError(kInf, 0.0, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, kInf, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, 0.0, -kInf, max_m));
    }
}

TEST(ErrorGate, NegativeErrorIsAlwaysRejected) {
    for (double max_m : {0.0, 5.0, 1e9}) {
        EXPECT_TRUE(rejectTargetError(-0.001, 0.0, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, -1.0, 0.0, max_m));
        EXPECT_TRUE(rejectTargetError(0.0, 0.0, -1.0, max_m));
        EXPECT_TRUE(rejectTargetError(-1.0, -1.0, -1.0, max_m));
    }
}

TEST(ErrorGate, NonFiniteOrNegativeLimitIsRejected) {
    EXPECT_TRUE(rejectTargetError(1.0, 1.0, 1.0, kNaN));
    EXPECT_TRUE(rejectTargetError(0.0, 0.0, 0.0, kNaN));
    EXPECT_TRUE(rejectTargetError(1.0, 1.0, 1.0, kInf));
    EXPECT_TRUE(rejectTargetError(1.0, 1.0, 1.0, -kInf));
    EXPECT_TRUE(rejectTargetError(1.0, 1.0, 1.0, -1.0));
    EXPECT_TRUE(rejectTargetError(0.0, 0.0, 0.0, -1.0));
}
