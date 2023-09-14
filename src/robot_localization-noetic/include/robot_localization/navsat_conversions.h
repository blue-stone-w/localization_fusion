

#ifndef ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H
#define ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H

/**  @file

     @brief Universal Transverse Mercator transforms.
     Functions to convert (spherical) latitude and longitude to and
     from (Euclidean) UTM coordinates.
     @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <string>

#include <stdio.h>
#include <stdlib.h>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace RobotLocalization
{
namespace NavsatConversions
{

const double RADIANS_PER_DEGREE = M_PI / 180.0;
const double DEGREES_PER_RADIAN = 180.0 / M_PI;

// Grid granularity for rounding UTM coordinates to generate MapXY.
const double grid_size = 100000.0; // 100 km grid

// WGS84 Parameters
#define WGS84_A 6378137.0 // major axis
#define WGS84_B 6356752.31424518 // minor axis
#define WGS84_F 0.0033528107 // ellipsoid flattening
#define WGS84_E 0.0818191908 // first eccentricity
#define WGS84_EP 0.0820944379 // second eccentricity

// UTM Parameters
#define UTM_K0 0.9996 // scale factor
#define UTM_FE 500000.0 // false easting
#define UTM_FN_N 0.0 // false northing, northern hemisphere
#define UTM_FN_S 10000000.0 // false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E) // e^2
#define UTM_E4 (UTM_E2 * UTM_E2) // e^4
#define UTM_E6 (UTM_E4 * UTM_E2) // e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2)) // e'^2

/**
 * Utility function to convert geodetic to UTM position
 *
 * Units in are floating point degrees (sign for east/west)
 *
 * Units out are meters
 *
 * @todo deprecate this interface in favor of LLtoUTM()
 */
static inline void UTM(double lat, double lon, double *x, double *y)
{
  // constants
  static const double m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
  static const double m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
  static const double m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
  static const double m3 = -(35 * UTM_E6 / 3072);

  // compute the central meridian
  int cm = ((lon >= 0.0) ? (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 + 3) : (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 - 3));

  // convert degrees into radians
  double rlat  = lat * RADIANS_PER_DEGREE;
  double rlon  = lon * RADIANS_PER_DEGREE;
  double rlon0 = cm * RADIANS_PER_DEGREE;

  // compute trigonometric functions
  double slat = sin(rlat);
  double clat = cos(rlat);
  double tlat = tan(rlat);

  // decide the false northing at origin
  double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

  double T = tlat * tlat;
  double C = UTM_EP2 * clat * clat;
  double A = (rlon - rlon0) * clat;
  double M = WGS84_A * (m0 * rlat + m1 * sin(2 * rlat) + m2 * sin(4 * rlat) + m3 * sin(6 * rlat));
  double V = WGS84_A / sqrt(1 - UTM_E2 * slat * slat);

  // compute the easting-northing coordinates
  *x = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * pow(A, 5) / 120);
  *y = fn + UTM_K0 * (M + V * tlat * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2) * pow(A, 6) / 720)));

  return;
}

/**
 * Convert lat/long to UTM coords.
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * @param[out] gamma meridian convergence at point (degrees).
 */
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone, double &gamma)
{
  int zone;
  bool northp;
  double k_unused;
  GeographicLib::UTMUPS::Forward(Lat, Long, zone, northp, UTMEasting, UTMNorthing, gamma,
                                 k_unused, GeographicLib::UTMUPS::zonespec::MATCH);
  GeographicLib::MGRS::Forward(zone, northp, UTMEasting, UTMNorthing, -1, UTMZone);
}

/**
 * Convert lat/long to UTM coords.
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 */
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone)
{
  double gamma = 0.0;
  LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, UTMZone, gamma);
}

/**
 * Converts UTM coords to lat/long.
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 *
 * @param[out] gamma meridian convergence at point (degrees).
 */
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const std::string &UTMZone, double &Lat, double &Long,
                           double & /*gamma*/)
{
  int zone;
  bool northp;
  double x_unused;
  double y_unused;
  int prec_unused;
  GeographicLib::MGRS::Reverse(UTMZone, zone, northp, x_unused, y_unused, prec_unused, true);
  GeographicLib::UTMUPS::Reverse(zone, northp, UTMEasting, UTMNorthing, Lat, Long);
}

/**
 * Converts UTM coords to lat/long.
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 */
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const std::string &UTMZone, double &Lat, double &Long)
{
  double gamma;
  UTMtoLL(UTMNorthing, UTMEasting, UTMZone, Lat, Long, gamma);
}

} // namespace NavsatConversions
} // namespace RobotLocalization

#endif // ROBOT_LOCALIZATION_NAVSAT_CONVERSIONS_H
