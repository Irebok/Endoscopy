#include "FusionMath.h"



// static inline float FusionDegreesToRadians(const float degrees) {
//     return degrees * ((float) M_PI / 180.0f);
// }


// static inline float FusionRadiansToDegrees(const float radians) {
//     return radians * (180.0f / (float) M_PI);
// }


// static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
//     const FusionVector result = {.axis = {
//             .x = vectorA.axis.x + vectorB.axis.x,
//             .y = vectorA.axis.y + vectorB.axis.y,
//             .z = vectorA.axis.z + vectorB.axis.z,
//     }};
//     return result;
// }


// static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
//     const FusionVector result = {.axis = {
//             .x = vectorA.axis.x - vectorB.axis.x,
//             .y = vectorA.axis.y - vectorB.axis.y,
//             .z = vectorA.axis.z - vectorB.axis.z,
//     }};
//     return result;
// }

// static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
//     #ifdef FUSION_USE_NORMAL_SQRT
//         const float magnitudeReciprocal = 1.0f / sqrtf(FusionVectorMagnitudeSquared(vector));
//     #else
//         const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
//     #endif
//         return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
// }

FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
    #define Q quaternion.element
        const float halfMinusQySquared = 0.5f - Q.y * Q.y; // calculate common terms to avoid repeated operations
        const FusionEuler euler = {.angle = {
                .roll = FusionRadiansToDegrees(atan2f(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x)),
                .pitch = FusionRadiansToDegrees(FusionAsin(2.0f * (Q.w * Q.y - Q.z * Q.x))),
                .yaw = FusionRadiansToDegrees(atan2f(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z)),
        }};
        return euler;
    #undef Q
}

int FusionTestFunction(void) {
    return 42;
}