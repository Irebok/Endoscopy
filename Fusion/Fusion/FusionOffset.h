/**
 * @file FusionOffset.h
 * @author Seb Madgwick
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

#ifndef FUSION_OFFSET_H
#define FUSION_OFFSET_H

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
// Definitions

/**
 * @brief Gyroscope offset algorithm structure. Structure members are used
 * internally and must not be accessed by the application.
 */
typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    FusionVector gyroscopeOffset;
} FusionOffset;

//------------------------------------------------------------------------------
// Function declarations

FUSION_API void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);

FUSION_API FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);

#endif

//------------------------------------------------------------------------------
// End of file
