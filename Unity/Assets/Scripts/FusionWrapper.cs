using System;
using System.Runtime.InteropServices;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


public static class FusionWrapper
{
    public const int SAMPLE_RATE = 40;
    // public const int SAMPLE_RATE = 97;

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionVector {
        public float x, y, z;

        public FusionVector(float x, float y, float z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public FusionVector(Vector3 vector) {
            x = vector.x;
            y = vector.y;
            z = vector.z;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionMatrix {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public FusionMatrix(float m00, float m01, float m02,
                            float m10, float m11, float m12,
                            float m20, float m21, float m22) {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        public FusionMatrix(Matrix4x4 matrix) {
            m00 = matrix.m00; m01 = matrix.m01; m02 = matrix.m02;
            m10 = matrix.m10; m11 = matrix.m11; m12 = matrix.m12;
            m20 = matrix.m20; m21 = matrix.m21; m22 = matrix.m22;
        }

        public FusionMatrix(Matrix<double> mathNetMatrix)
        {
            if (mathNetMatrix.RowCount != 3 || mathNetMatrix.ColumnCount != 3)
            {
                throw new ArgumentException("Matrix must be 3x3 to be converted to FusionMatrix.");
            }

            m00 = (float)mathNetMatrix[0, 0];
            m01 = (float)mathNetMatrix[0, 1];
            m02 = (float)mathNetMatrix[0, 2];

            m10 = (float)mathNetMatrix[1, 0];
            m11 = (float)mathNetMatrix[1, 1];
            m12 = (float)mathNetMatrix[1, 2];

            m20 = (float)mathNetMatrix[2, 0];
            m21 = (float)mathNetMatrix[2, 1];
            m22 = (float)mathNetMatrix[2, 2];
        }
    }



    [StructLayout(LayoutKind.Sequential)]
    public struct FusionEuler {
        public float roll;
        public float pitch;
        public float yaw;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionQuaternion {
        public float w;
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionAhrsSettings {
        public int convention;
        public float gain;
        public float gyroscopeRange;
        public float accelerationRejection;
        public float magneticRejection;
        public uint recoveryTriggerPeriod;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionOffset {
        public float filterCoefficient;
        public uint timeout;
        public uint timer;
        public FusionVector gyroscopeOffset;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionAhrs {
        public FusionAhrsSettings settings;
        public FusionQuaternion quaternion;
        public FusionVector accelerometer;
        [MarshalAs(UnmanagedType.I1)] public bool initialising;
        public float rampedGain;
        public float rampedGainStep;
        [MarshalAs(UnmanagedType.I1)] public bool angularRateRecovery;
        public FusionVector halfAccelerometerFeedback;
        public FusionVector halfMagnetometerFeedback;
        [MarshalAs(UnmanagedType.I1)] public bool accelerometerIgnored;
        public int accelerationRecoveryTrigger;
        public int accelerationRecoveryTimeout;
        [MarshalAs(UnmanagedType.I1)] public bool magnetometerIgnored;
        public int magneticRecoveryTrigger;
        public int magneticRecoveryTimeout;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionAhrsFlags {
        [MarshalAs(UnmanagedType.I1)] public bool initialising;
        [MarshalAs(UnmanagedType.I1)] public bool angularRateRecovery;
        [MarshalAs(UnmanagedType.I1)] public bool accelerationRecovery;
        [MarshalAs(UnmanagedType.I1)] public bool magneticRecovery;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FusionAhrsInternalStates {
        public float accelerationError;
        [MarshalAs(UnmanagedType.I1)] public bool accelerometerIgnored;
        public float accelerationRecoveryTrigger;
        public float magneticError;
        [MarshalAs(UnmanagedType.I1)] public bool magnetometerIgnored;
        public float magneticRecoveryTrigger;
    }

    // Funciones
    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionOffsetInitialise(ref FusionOffset offset, uint sampleRate);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionVector FusionOffsetUpdate(ref FusionOffset offset, FusionVector gyroscope);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionVector FusionCalibrationInertial(
        FusionVector vector, FusionMatrix misalignment, FusionVector sensitivity, FusionVector offset);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionVector FusionCalibrationMagnetic(
        FusionVector vector, FusionMatrix softIronMatrix, FusionVector hardIronOffset);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionAhrsInitialise(ref FusionAhrs ahrs);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionAhrsReset(ref FusionAhrs ahrs);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionAhrsSetSettings(ref FusionAhrs ahrs, ref FusionAhrsSettings settings);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionAhrsUpdate(ref FusionAhrs ahrs, FusionVector gyro, FusionVector accel, FusionVector mag, float deltaTime);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern void FusionAhrsUpdateNoMagnetometer(ref FusionAhrs ahrs, FusionVector gyro, FusionVector accel, float deltaTime);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionQuaternion FusionAhrsGetQuaternion(ref FusionAhrs ahrs);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionVector FusionAhrsGetEarthAcceleration(ref FusionAhrs ahrs);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionEuler FusionQuaternionToEuler(FusionQuaternion quaternion);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern int FusionTestFunction();
        
    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionAhrsFlags FusionAhrsGetFlags(ref FusionAhrs ahrs);

    [DllImport("Fusion", CallingConvention = CallingConvention.Cdecl)]
    public static extern FusionAhrsInternalStates FusionAhrsGetInternalStates(ref FusionAhrs ahrs);
}
