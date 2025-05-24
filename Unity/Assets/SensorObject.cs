using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;
using System.Collections;
using static FusionWrapper;


public class OrientationUpdater : MonoBehaviour
{
    public Verifier provider; 
    int i = 0;

    void Update()
    {
        if (provider != null)
        {   
            i++;
            if (i == 10)
            {
                // Debug.Log($"comenzando calibracion desde objeto externo");
                // provider.startCalibrationAcel(Verifier.SensorType.MidTube, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.MidTube, 1000);
                // provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)44.67, (float)20.69, (float)-39.62), Matrix4x4.identity);
                // provider.startCalibrationAcel(Verifier.SensorType.EndTube, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.EndTube, 1000);
                // provider.startCalibrationMag(Verifier.SensorType.EndTube, 3000);
                // provider.startCalibrationAcel(Verifier.SensorType.MidTube, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.MidTube, 1000);
                // provider.startCalibrationMag(Verifier.SensorType.MidTube, 3000);
                provider.setAcelBias(Verifier.SensorType.MidTube, new Vector3((float)0.06350683, (float)0.03050875, (float)-0.008543432));  // 0,06350683 0,03050875 -0,008543432
                provider.setGyroBias(Verifier.SensorType.MidTube, new Vector3((float)5.723135, (float)-3.25192, (float)2.541067));  // 5,723135 -3,25192 2,541067
                Matrix4x4 m4x4End = Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)18.67, (float)13.37, (float)-2.08), m4x4End);  // 18.67, 13.37, -2.08

                provider.setAcelBias(Verifier.SensorType.EndTube, new Vector3((float)0.0, (float)0.01, (float)0.01));  //  0,00, 0,01, 0,01
                provider.setGyroBias(Verifier.SensorType.EndTube, new Vector3((float)-2.52, (float)0.65, (float)-0.58));  //  -2,52, 0,65, -0,58
                Matrix4x4 m4x4Mid = Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                provider.setMagBias(Verifier.SensorType.EndTube, new Vector3((float)15.08, (float)13.95, (float)5.58), m4x4Mid);  // (15.08, 13.95, 5.58)
                // provider.setAcelBias(Verifier.SensorType.EndTube, new Vector3((float)0.06350683, (float)0.03050875, (float)-0.008543432));  // 0,06350683 0,03050875 -0,008543432
                // provider.setGyroBias(Verifier.SensorType.EndTube, new Vector3((float)5.723135, (float)-3.25192, (float)2.541067));  // 5,723135 -3,25192 2,541067
                // Matrix4x4 m4x4End =  Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                // provider.setMagBias(Verifier.SensorType.EndTube, new Vector3((float)18.67, (float)13.37, (float)-2.08), m4x4End);  // 18.67, 13.37, -2.08

                // provider.setAcelBias(Verifier.SensorType.MidTube, new Vector3((float)0.0, (float)0.01, (float)0.01));  //  0,00, 0,01, 0,01
                // provider.setGyroBias(Verifier.SensorType.MidTube, new Vector3((float)-2.52, (float)0.65, (float)-0.58));  //  -2,52, 0,65, -0,58
                // Matrix4x4 m4x4Mid =  Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                // provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)15.08, (float)13.95, (float)5.58), m4x4Mid);  // (15.08, 13.95, 5.58)

                provider.setAcelBias(Verifier.SensorType.Internal, new Vector3((float)0.0, (float)-0.02, (float)-0.01));
                provider.setGyroBias(Verifier.SensorType.Internal, new Vector3((float)0.87, (float)-2.65, (float)0.52));
                provider.setMagBias(Verifier.SensorType.Internal, new Vector3((float)44.67, (float)20.69, (float)-39.62), Matrix4x4.identity);
                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.EndTube, 500);
                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.MidTube, 500);
                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.Internal, 500);

            }

            if (provider.isCalibrationAcel(Verifier.SensorType.MidTube) || provider.isCalibrationGyro(Verifier.SensorType.MidTube) || provider.isCalibrationMag(Verifier.SensorType.MidTube))
            {
                return; // Salir de Update si aún estamos calibrando
            }

            Vector3 acelbias= provider.getAcelBias(Verifier.SensorType.MidTube);
            Vector3 gyrobias= provider.getGyroBias(Verifier.SensorType.MidTube);
            // Debug.Log($"Leyendo calibracion acel {acelbias.x} {acelbias.y} {acelbias.z}"); 
            // Debug.Log($"Leyendo calibracion gyro {gyrobias.x} {gyrobias.y} {gyrobias.z}");
            // provider.GetMagCalibrationReport(Verifier.SensorType.MidTube);

            // if (provider.isCalibrationAcel(Verifier.SensorType.EndTube) || provider.isCalibrationGyro(Verifier.SensorType.EndTube) || provider.isCalibrationMag(Verifier.SensorType.EndTube))
            // {
            //     Debug.Log($"Calibring endtube sensor");
            //     return; // Salir de Update si aún estamos calibrando
            // }
            // Debug.Log($"valor i: {i}");


            // if (i==10)
            // {
            //     Debug.Log($"Comenzando calibracion desde objeto externo");
            //     provider.startCalibrationAcel(Verifier.SensorType.MidTube, 1000); 
            //     provider.startCalibrationGyro(Verifier.SensorType.MidTube, 1000);
            // }
            // if (provider.isCalibrationAcel(Verifier.SensorType.MidTube) || provider.isCalibrationGyro(Verifier.SensorType.MidTube))
            // {
            //     // Debug.Log($"Calibring MidTube sensor");
            //     return; // Salir de Update si aún estamos calibrando
            // }

            // if (i==10)
            // {
            //     Debug.Log($"Comenzando calibracion desde objeto externo");
            //     provider.startCalibrationAcel(Verifier.SensorType.EndTube, 1000); 
            //     provider.startCalibrationGyro(Verifier.SensorType.EndTube, 1000);
            // }
            // if (provider.isCalibrationAcel(Verifier.SensorType.EndTube) || provider.isCalibrationGyro(Verifier.SensorType.EndTube))
            // {
            //     Debug.Log($"Calibring EndTube sensor");
            //     return; // Salir de Update si aún estamos calibrando
            // }


            // (float roll, float pitch, float yaw) = provider.GetOrientation(Verifier.SensorType.Internal);
            // (float roll, float pitch, float yaw) = provider.GetOrientation(Verifier.SensorType.MidTube);
            (float roll, float pitch, float yaw) = provider.GetOrientation(Verifier.SensorType.Internal);
            // transform.rotation = Quaternion.Euler(pitch, yaw, roll);
            // transform.position = new Vector3(provider.GetPosition(),0,0);
            // Debug.Log($"SensorObjekct Roll: {roll:F1}°, Pitch: {pitch:F1}°, Yaw: {yaw:F1}°");


            // FusionAhrsFlags flags = provider.GetFlags(Verifier.SensorType.Internal);
            // FusionAhrsFlags flags = provider.GetFlags(Verifier.SensorType.MidTube);
            FusionAhrsFlags flags = provider.GetFlags(Verifier.SensorType.Internal);
            if (flags.initialising || flags.angularRateRecovery || flags.accelerationRecovery || flags.magneticRecovery)
                Debug.Log($"SensorObject Roll: {roll:F1}°, Pitch: {pitch:F1}°, Yaw: {yaw:F1}°, initialising: {flags.initialising}, angularRateRecovery: {flags.angularRateRecovery}, accelerationRecovery: {flags.accelerationRecovery}, magneticRecovery: {flags.magneticRecovery}");
            
            FusionAhrsInternalStates intStates = provider.GetInternalStates(Verifier.SensorType.Internal);
            // if (intStates.accelerationError > 0 || intStates.accelerationRecoveryTrigger > 0 || intStates.accelerometerIgnored || intStates.magneticError > 0 || intStates.magneticRecoveryTrigger > 0 || intStates.magnetometerIgnored) {
            //     Debug.Log(
            //         $"[AHRS] AccErr: {intStates.accelerationError:F4}, AccIgnored: {intStates.accelerometerIgnored}, " +
            //         $"AccRecTrig: {intStates.accelerationRecoveryTrigger:F4}, MagErr: {intStates.magneticError:F4}, " +
            //         $"MagIgnored: {intStates.magnetometerIgnored}, MagRecTrig: {intStates.magneticRecoveryTrigger:F4}"
            //     );
            // }

            // if (i < 100) 
            // {
            //     i++;
            // }
            // if (i==100)
            // {
            //     Matrix4x4 manualMatrix = new Matrix4x4();
            //     manualMatrix.m00 = 0.938f;  manualMatrix.m01 = 0.000f;  manualMatrix.m02 = 0.000f;  manualMatrix.m03 = 0.0f;
            //     manualMatrix.m10 = 0.000f;  manualMatrix.m11 = 1.033f;  manualMatrix.m12 = 0.000f;  manualMatrix.m13 = 0.0f;
            //     manualMatrix.m20 = 0.000f;  manualMatrix.m21 = 0.000f;  manualMatrix.m22 = 1.0329f;  manualMatrix.m23 = 0.0f;
            //     manualMatrix.m30 = 0.0f;    manualMatrix.m31 = 0.0f;    manualMatrix.m32 = 0.0f;    manualMatrix.m33 = 1.0f;

            //     provider.setMagBias(Verifier.SensorType.EndTube, new Vector3((float)8.16, (float)10.23, (float)193.74), manualMatrix);
            // }

            // if (i == 1) 
            // {
            //     Matrix4x4 manualMatrix = new Matrix4x4();
            //     manualMatrix.m00 = 1.040f;  manualMatrix.m01 = 0.000f;  manualMatrix.m02 = 0.000f;  manualMatrix.m03 = 0.0f;
            //     manualMatrix.m10 = 0.000f;  manualMatrix.m11 = 0.936f;  manualMatrix.m12 = 0.000f;  manualMatrix.m13 = 0.0f;
            //     manualMatrix.m20 = 0.000f;  manualMatrix.m21 = 0.000f;  manualMatrix.m22 = 1.024f;  manualMatrix.m23 = 0.0f;
            //     manualMatrix.m30 = 0.0f;    manualMatrix.m31 = 0.0f;    manualMatrix.m32 = 0.0f;    manualMatrix.m33 = 1.0f;

            //     provider.setMagBias(new Vector3((float)-9.85, (float)-31.49, (float)195.84), manualMatrix);
            // }
            // if(i < 100) 
            // {
            //     i++;
            //     Debug.Log($"SensorObject Roll: {roll:F1}°, Pitch: {pitch:F1}°, Yaw: {yaw:F1}°");
            // }
            
            // if(i==100) 
            // {
            //     // provider.startCalibrationMag(10000);
            //     Debug.Log($"SensorObject starting magnetometer calibration");
            //     StartCoroutine(CalibrateMagnetometer());
            //     i++;
            // }
        }
        // Debug.Log($"Provider erroneo");

    }

    // private IEnumerator CalibrateMagnetometer()
    // {
    //     while (provider.isCalibrationMag(Verifier.SensorType.Internal))
    //     {
    //         yield return null; 
    //     }

    //     Vector3 bias = provider.getAcelBias(Verifier.SensorType.MidTube);
    //         Debug.Log($"SensorObject Acel: {bias.x:F1}°, {bias.y:F1}°, {bias.z:F1}°");

    //     bias = provider.getGyroBias(Verifier.SensorType.Internal);
    //         Debug.Log($"SensorObject Gyro:  {bias.x:F1}°, {bias.y:F1}°, {bias.z:F1}°");
    //     // bias = provider.getMagBias();
    //     //     Debug.Log($"SensorObject Mag: {bias.x:F1}°, {bias.y:F1}°, {bias.z:F1}°");
    //     provider.GetMagCalibrationReport(Verifier.SensorType.Internal);
    // }
}