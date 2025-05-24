#include "FusionCalibration.h"



FusionVector FusionCalibrationInertial(const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset) {
    return FusionMatrixMultiplyVector(misalignment, FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
}

FusionVector FusionCalibrationMagnetic(const FusionVector uncalibrated, const FusionMatrix softIronMatrix, const FusionVector hardIronOffset) {
    return FusionMatrixMultiplyVector(softIronMatrix, FusionVectorSubtract(uncalibrated, hardIronOffset));
}