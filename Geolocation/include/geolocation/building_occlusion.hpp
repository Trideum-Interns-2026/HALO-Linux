#ifndef GEOLOCATION_BUILDING_OCCLUSION_HPP
#define GEOLOCATION_BUILDING_OCCLUSION_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geolocation/position_estimator.hpp"

namespace geolocation {

struct BuildingFootprint {
    std::string id;
    double latitude_deg{};
    double longitude_deg{};
    double radius_m{};
    double base_alt_m{};
    double height_m{};
};

struct BuildingRiskEstimate {
    double probability{};
    double building_bearing_deg{};
    double building_distance_m{};
    double building_height_m{};
};

struct BuildingFootprintLoadResult {
    std::vector<BuildingFootprint> footprints;
    bool file_opened{false};
    bool header_valid{false};
    std::size_t skipped_rows{};
};

inline std::string trimCsvField(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

inline std::vector<std::string> splitCsvRow(const std::string& row) {
    std::vector<std::string> fields;
    std::stringstream stream(row);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trimCsvField(field));
    }
    if (!row.empty() && row.back() == ',') {
        fields.emplace_back();
    }
    return fields;
}

inline double parseCsvNumber(const std::string& field) {
    std::size_t consumed = 0;
    const double value = std::stod(field, &consumed);
    if (consumed != field.size() || !std::isfinite(value)) {
        throw std::invalid_argument("CSV value must be finite numeric text");
    }
    return value;
}

inline bool isValidFootprint(const BuildingFootprint& footprint) {
    try {
        validatePosition(
            {footprint.latitude_deg, footprint.longitude_deg, footprint.base_alt_m});
    } catch (const std::invalid_argument&) {
        return false;
    }
    return !footprint.id.empty() && std::isfinite(footprint.radius_m) &&
        footprint.radius_m > 0.0 && std::isfinite(footprint.height_m) &&
        footprint.height_m >= 0.0;
}

inline BuildingFootprintLoadResult loadBuildingFootprints(const std::string& path) {
    BuildingFootprintLoadResult result;
    std::ifstream input(path);
    if (!input) {
        return result;
    }
    result.file_opened = true;

    constexpr std::array<const char*, 6> kHeader{
        "id", "latitude_deg", "longitude_deg", "radius_m", "base_alt_m", "height_m"};
    std::string row;
    if (!std::getline(input, row)) {
        return result;
    }
    const auto header = splitCsvRow(row);
    result.header_valid = header.size() == kHeader.size();
    for (std::size_t i = 0; result.header_valid && i < kHeader.size(); ++i) {
        result.header_valid = header[i] == kHeader[i];
    }
    if (!result.header_valid) {
        ++result.skipped_rows;
        return result;
    }

    while (std::getline(input, row)) {
        if (trimCsvField(row).empty()) {
            continue;
        }
        try {
            const auto fields = splitCsvRow(row);
            if (fields.size() != kHeader.size()) {
                ++result.skipped_rows;
                continue;
            }
            BuildingFootprint footprint{
                fields[0],
                parseCsvNumber(fields[1]),
                parseCsvNumber(fields[2]),
                parseCsvNumber(fields[3]),
                parseCsvNumber(fields[4]),
                parseCsvNumber(fields[5])};
            if (!isValidFootprint(footprint)) {
                ++result.skipped_rows;
                continue;
            }
            result.footprints.push_back(std::move(footprint));
        } catch (const std::invalid_argument&) {
            ++result.skipped_rows;
        } catch (const std::out_of_range&) {
            ++result.skipped_rows;
        }
    }
    return result;
}

inline std::array<double, 2> horizontalEnuOffset(
    const GeodeticPosition& origin,
    const GeodeticPosition& position) {
    const auto origin_ecef = geodeticToEcef(origin);
    const auto position_ecef = geodeticToEcef(position);
    const std::array<double, 3> delta{
        position_ecef[0] - origin_ecef[0],
        position_ecef[1] - origin_ecef[1],
        position_ecef[2] - origin_ecef[2]};
    const double latitude = degreesToRadians(origin.latitude_deg);
    const double longitude = degreesToRadians(origin.longitude_deg);
    const double east = -std::sin(longitude) * delta[0] +
        std::cos(longitude) * delta[1];
    const double north = -std::sin(latitude) * std::cos(longitude) * delta[0] -
        std::sin(latitude) * std::sin(longitude) * delta[1] +
        std::cos(latitude) * delta[2];
    return {east, north};
}

inline double horizontalSigmaM(
    const GeodeticPosition& position,
    const std::array<double, 3>& error_ecef_3sigma_m) {
    for (const double bound : error_ecef_3sigma_m) {
        if (!std::isfinite(bound) || bound < 0.0) {
            throw std::invalid_argument("position error bounds must be finite and non-negative");
        }
    }

    const double latitude = degreesToRadians(position.latitude_deg);
    const double longitude = degreesToRadians(position.longitude_deg);
    const std::array<double, 3> sigma_ecef{
        error_ecef_3sigma_m[0] / 3.0,
        error_ecef_3sigma_m[1] / 3.0,
        error_ecef_3sigma_m[2] / 3.0};
    const std::array<double, 3> east_basis{
        -std::sin(longitude), std::cos(longitude), 0.0};
    const std::array<double, 3> north_basis{
        -std::sin(latitude) * std::cos(longitude),
        -std::sin(latitude) * std::sin(longitude),
        std::cos(latitude)};

    double east_variance = 0.0;
    double north_variance = 0.0;
    for (std::size_t axis = 0; axis < sigma_ecef.size(); ++axis) {
        east_variance += east_basis[axis] * east_basis[axis] *
            sigma_ecef[axis] * sigma_ecef[axis];
        north_variance += north_basis[axis] * north_basis[axis] *
            sigma_ecef[axis] * sigma_ecef[axis];
    }
    const double sigma_m = std::sqrt((east_variance + north_variance) / 2.0);
    if (!std::isfinite(sigma_m)) {
        throw std::invalid_argument("horizontal position uncertainty is too large");
    }
    return sigma_m;
}

inline BuildingRiskEstimate estimateBuildingRisk(
    const GeodeticPosition& position,
    const std::array<double, 3>& error_ecef_3sigma_m,
    const std::vector<BuildingFootprint>& buildings,
    double max_range_m) {
    validatePosition(position);
    if (!std::isfinite(max_range_m) || max_range_m < 0.0) {
        throw std::invalid_argument("maximum building range must be finite and non-negative");
    }
    if (!std::isfinite(error_ecef_3sigma_m[2]) || error_ecef_3sigma_m[2] < 0.0) {
        throw std::invalid_argument("vertical position error must be finite and non-negative");
    }
    const double sigma_m = horizontalSigmaM(position, error_ecef_3sigma_m);

    bool found = false;
    BuildingRiskEstimate riskiest;
    for (const auto& building : buildings) {
        if (!isValidFootprint(building)) {
            continue;
        }
        const double roof_altitude_m = building.base_alt_m + building.height_m;
        if (!std::isfinite(roof_altitude_m) ||
            position.altitude_m - error_ecef_3sigma_m[2] > roof_altitude_m) {
            continue;
        }

        const auto offset = horizontalEnuOffset(
            position, {building.latitude_deg, building.longitude_deg, building.base_alt_m});
        const double center_distance_m = std::hypot(offset[0], offset[1]);
        const double edge_distance_m = std::max(0.0, center_distance_m - building.radius_m);
        if (edge_distance_m > max_range_m) {
            continue;
        }

        const double probability = edge_distance_m == 0.0 ? 1.0 :
            (sigma_m > 0.0 ?
                std::exp(-0.5 * (edge_distance_m / sigma_m) *
                    (edge_distance_m / sigma_m)) : 0.0);
        if (!found || probability > riskiest.probability) {
            double bearing_deg = std::atan2(offset[0], offset[1]) * 180.0 / kPi;
            if (bearing_deg < 0.0) {
                bearing_deg += 360.0;
            }
            riskiest = {
                probability,
                bearing_deg,
                edge_distance_m,
                roof_altitude_m};
            found = true;
        }
    }
    if (!found) {
        throw std::out_of_range(
            "no building is in range below the drone's lowest 3-sigma altitude");
    }
    return riskiest;
}

}  // namespace geolocation

#endif
