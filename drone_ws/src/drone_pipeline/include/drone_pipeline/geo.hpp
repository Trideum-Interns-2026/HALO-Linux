#pragma once
// WGS84 helpers: LLA <-> ECEF <-> local ENU. Header-only, no ROS dependency.
#include <array>
#include <cmath>

namespace geo {

constexpr double kA = 6378137.0;                 // semi-major axis (m)
constexpr double kF = 1.0 / 298.257223563;       // flattening
constexpr double kE2 = kF * (2.0 - kF);          // eccentricity squared
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

using Vec3 = std::array<double, 3>;
struct Lla { double lat, lon, alt; };            // degrees, degrees, meters

inline Vec3 lla_to_ecef(const Lla & p) {
  const double lat = p.lat * kDeg2Rad, lon = p.lon * kDeg2Rad;
  const double s = std::sin(lat), c = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * s * s);
  return {(n + p.alt) * c * std::cos(lon),
          (n + p.alt) * c * std::sin(lon),
          (n * (1.0 - kE2) + p.alt) * s};
}

inline Lla ecef_to_lla(const Vec3 & e) {
  const double lon = std::atan2(e[1], e[0]);
  const double p = std::hypot(e[0], e[1]);
  double lat = std::atan2(e[2], p * (1.0 - kE2));
  double alt = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double s = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * s * s);
    alt = p / std::cos(lat) - n;
    lat = std::atan2(e[2], p * (1.0 - kE2 * n / (n + alt)));
  }
  return {lat * kRad2Deg, lon * kRad2Deg, alt};
}

// Rows are the East, North, Up unit vectors expressed in ECEF coordinates.
using Basis = std::array<Vec3, 3>;

inline Basis enu_basis(double lat_deg, double lon_deg) {
  const double lat = lat_deg * kDeg2Rad, lon = lon_deg * kDeg2Rad;
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double so = std::sin(lon), co = std::cos(lon);
  return Basis{{{{-so, co, 0.0}},
                {{-sl * co, -sl * so, cl}},
                {{cl * co, cl * so, sl}}}};
}

// Point reached by moving (east, north, up) meters from ref.
inline Lla offset_lla(const Lla & ref, double east, double north, double up) {
  const Vec3 e0 = lla_to_ecef(ref);
  const Basis b = enu_basis(ref.lat, ref.lon);
  const double enu[3] = {east, north, up};
  Vec3 e = e0;
  for (int k = 0; k < 3; ++k)
    for (int i = 0; i < 3; ++i) e[i] += b[k][i] * enu[k];
  return ecef_to_lla(e);
}

// Position of p relative to ref, as (east, north, up) meters.
inline Vec3 lla_to_enu(const Lla & ref, const Lla & p) {
  const Vec3 a = lla_to_ecef(ref), c = lla_to_ecef(p);
  const Vec3 d = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  const Basis b = enu_basis(ref.lat, ref.lon);
  Vec3 out{};
  for (int k = 0; k < 3; ++k)
    out[k] = b[k][0] * d[0] + b[k][1] * d[1] + b[k][2] * d[2];
  return out;
}

inline double distance_m(const Vec3 & a, const Vec3 & b) {
  return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

// Axis-aligned 1-sigma in ENU -> per-axis 1-sigma in ECEF (diagonal of the rotated covariance).
inline Vec3 enu_sigma_to_ecef_sigma(const Lla & at, const Vec3 & sig_enu) {
  const Basis b = enu_basis(at.lat, at.lon);
  Vec3 out{};
  for (int i = 0; i < 3; ++i) {
    double var = 0.0;
    for (int k = 0; k < 3; ++k) var += b[k][i] * b[k][i] * sig_enu[k] * sig_enu[k];
    out[i] = std::sqrt(var);
  }
  return out;
}

}  // namespace geo
