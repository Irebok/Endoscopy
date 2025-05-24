/**
 * @file FusionCalibration.h
 * @author Seb Madgwick
 * @brief Gyroscope, accelerometer, and magnetometer calibration models.
 */

#ifndef FUSION_CALIBRATION_H
#define FUSION_CALIBRATION_H

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.h"

#ifdef _WIN32
    #ifdef FUSION_EXPORTS
    #define FUSION_API __declspec(dllexport)
    #else
    #define FUSION_API __declspec(dllimport)
    #endif
#else
    #define FUSION_API
#endif

//------------------------------------------------------------------------------
// Inline functions

/**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param misalignment Misalignment matrix.
 * @param sensitivity Sensitivity.
 * @param offset Offset.
 * @return Calibrated measurement.
 */
FUSION_API FusionVector FusionCalibrationInertial(const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset);

/**
 * @brief Magnetometer calibration model.
 * @param uncalibrated Uncalibrated measurement.
 * @param softIronMatrix Soft-iron matrix.
 * @param hardIronOffset Hard-iron offset.
 * @return Calibrated measurement.
 */
FUSION_API FusionVector FusionCalibrationMagnetic(const FusionVector uncalibrated, const FusionMatrix softIronMatrix, const FusionVector hardIronOffset);

#endif

//------------------------------------------------------------------------------
// End of file
