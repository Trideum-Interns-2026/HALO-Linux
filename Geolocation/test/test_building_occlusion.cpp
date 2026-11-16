#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "geolocation/building_occlusion.hpp"

namespace {

geolocation::BuildingFootprint buildingAt(
    double latitude_deg,
    double longitude_deg,
    double radius_m,
    double base_alt_m = 100.0,
    double height_m = 20.0) {
    return {"test", latitude_deg, longitude_deg, radius_m, base_alt_m, height_m};
}

}  // namespace

TEST(BuildingOcclusion, RotatesEcefUncertaintyToHorizontalEnu) {
    const geolocation::GeodeticPosition equator{0.0, 0.0, 100.0};
    EXPECT_NEAR(
        geolocation::horizontalSigmaM(equator, {3.0, 0.0, 0.0}), 0.0, 1e-12);
    EXPECT_NEAR(
        geolocation::horizontalSigmaM(equator, {0.0, 3.0, 3.0}), 1.0, 1e-12);
}

TEST(BuildingOcclusion, PositionInsideDiscHasUnitRiskAndZeroDistance) {
    const auto estimate = geolocation::estimateBuildingRisk(
        {34.7, -86.6, 100.0}, {3.0, 3.0, 3.0},
        {buildingAt(34.7, -86.6, 10.0)}, 200.0);
    EXPECT_DOUBLE_EQ(estimate.probability, 1.0);
    EXPECT_DOUBLE_EQ(estimate.building_distance_m, 0.0);
    EXPECT_DOUBLE_EQ(estimate.building_height_m, 120.0);
}

TEST(BuildingOcclusion, RiskFallsOffWithDistanceFromDiscEdge) {
    const geolocation::GeodeticPosition position{0.0, 0.0, 100.0};
    const auto building = buildingAt(0.0, 0.00002694946, 1.0);
    const auto estimate = geolocation::estimateBuildingRisk(
        position, {0.0, 3.0, 3.0}, {building}, 10.0);
    EXPECT_NEAR(estimate.building_distance_m, 2.0, 0.02);
    EXPECT_NEAR(estimate.probability, std::exp(-0.5 * 4.0), 0.01);
    EXPECT_NEAR(estimate.building_bearing_deg, 90.0, 1e-5);
}

TEST(BuildingOcclusion, BearingIsClockwiseFromNorthAndNormalized) {
    const geolocation::GeodeticPosition position{0.0, 0.0, 100.0};
    const std::vector<geolocation::BuildingFootprint> buildings{
        buildingAt(-0.0001, 0.0, 1.0)};
    const auto estimate = geolocation::estimateBuildingRisk(
        position, {3.0, 3.0, 3.0}, buildings, 20.0);
    EXPECT_NEAR(estimate.building_bearing_deg, 180.0, 1e-5);
}

TEST(BuildingOcclusion, ExcludesBuildingsClearedByLowestThreeSigmaAltitude) {
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {34.7, -86.6, 130.0}, {3.0, 3.0, 3.0},
            {buildingAt(34.7, -86.6, 10.0, 100.0, 20.0)}, 200.0),
        std::out_of_range);
}

TEST(BuildingOcclusion, RoofAtLowestThreeSigmaBoundIsStillConsidered) {
    const auto estimate = geolocation::estimateBuildingRisk(
        {34.7, -86.6, 123.0}, {3.0, 3.0, 3.0},
        {buildingAt(34.7, -86.6, 10.0, 100.0, 20.0)}, 200.0);
    EXPECT_DOUBLE_EQ(estimate.probability, 1.0);
}

TEST(BuildingOcclusion, MeasuresRangeToNearestDiscEdge) {
    const geolocation::GeodeticPosition position{0.0, 0.0, 100.0};
    const auto building = buildingAt(0.0, 0.0001347473, 10.0);
    const auto estimate = geolocation::estimateBuildingRisk(
        position, {3.0, 3.0, 3.0}, {building}, 5.1);
    EXPECT_NEAR(estimate.building_distance_m, 5.0, 0.03);
}

TEST(BuildingOcclusion, DoesNotPublishWhenBuildingIsOutOfRange) {
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {0.0, 0.0, 100.0}, {3.0, 3.0, 3.0},
            {buildingAt(0.0, 0.001, 10.0)}, 20.0),
        std::out_of_range);
}

TEST(BuildingOcclusion, SelectsTheBuildingWithHighestOverlapRisk) {
    const geolocation::GeodeticPosition position{0.0, 0.0, 100.0};
    const auto nearer = buildingAt(0.0, 0.0001, 3.0);
    const auto inside = buildingAt(0.0, 0.0, 1.0);
    const auto estimate = geolocation::estimateBuildingRisk(
        position, {3.0, 3.0, 3.0}, {nearer, inside}, 100.0);
    EXPECT_DOUBLE_EQ(estimate.probability, 1.0);
    EXPECT_DOUBLE_EQ(estimate.building_distance_m, 0.0);
}

TEST(BuildingOcclusion, RejectsInvalidPositionUncertaintyAndRange) {
    const std::vector<geolocation::BuildingFootprint> buildings{
        buildingAt(0.0, 0.0, 1.0)};
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {91.0, 0.0, 100.0}, {3.0, 3.0, 3.0}, buildings, 10.0),
        std::invalid_argument);
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {0.0, 0.0, 100.0}, {-1.0, 3.0, 3.0}, buildings, 10.0),
        std::invalid_argument);
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {0.0, 0.0, 100.0}, {3.0, 3.0, 3.0}, buildings, -1.0),
        std::invalid_argument);
}

TEST(BuildingOcclusion, SkipsInvalidFootprints) {
    auto invalid = buildingAt(91.0, 0.0, 1.0);
    EXPECT_THROW(
        geolocation::estimateBuildingRisk(
            {0.0, 0.0, 100.0}, {3.0, 3.0, 3.0}, {invalid}, 100.0),
        std::out_of_range);
}

TEST(BuildingFootprintLoader, LoadsValidRowsAndReportsSkippedRows) {
    const std::string path = "/tmp/geolocation_building_loader_test.csv";
    {
        std::ofstream output(path);
        output << "id,latitude_deg,longitude_deg,radius_m,base_alt_m,height_m\n"
               << "valid,34.7,-86.6,10,190,20\n"
               << "bad,not-a-number,-86.6,10,190,20\n"
               << "wrong_columns,34.7,-86.6\n";
    }
    const auto result = geolocation::loadBuildingFootprints(path);
    std::remove(path.c_str());
    ASSERT_TRUE(result.file_opened);
    ASSERT_TRUE(result.header_valid);
    ASSERT_EQ(result.footprints.size(), 1U);
    EXPECT_EQ(result.footprints.front().id, "valid");
    EXPECT_EQ(result.skipped_rows, 2U);
}

TEST(BuildingFootprintLoader, ReportsMissingFilesAndInvalidHeaders) {
    const auto missing = geolocation::loadBuildingFootprints(
        "/tmp/geolocation_building_file_that_does_not_exist.csv");
    EXPECT_FALSE(missing.file_opened);
    EXPECT_TRUE(missing.footprints.empty());

    const std::string path = "/tmp/geolocation_building_bad_header_test.csv";
    {
        std::ofstream output(path);
        output << "name,lat,lon,radius,base,height\n";
    }
    const auto invalid_header = geolocation::loadBuildingFootprints(path);
    std::remove(path.c_str());
    EXPECT_TRUE(invalid_header.file_opened);
    EXPECT_FALSE(invalid_header.header_valid);
    EXPECT_TRUE(invalid_header.footprints.empty());
}
