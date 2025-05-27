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

    bool offsetCalibrated = false;

    void Update()
    {
        if (provider != null)
        {   
            i++;
            if (i == 10)
            {
                // Debug.Log($"comenzando calibracion desde objeto externo");
                // provider.startCalibrationAcel(Verifier.SensorType.Internal, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.Internal, 1000);
                // provider.startCalibrationMag(Verifier.SensorType.Internal, 4000);
                // provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)44.67, (float)20.69, (float)-39.62), Matrix4x4.identity);
                // provider.startCalibrationAcel(Verifier.SensorType.EndTube, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.EndTube, 1000);
                // provider.startCalibrationMag(Verifier.SensorType.EndTube, 2000);
                // provider.startCalibrationAcel(Verifier.SensorType.MidTube, 1000); 
                // provider.startCalibrationGyro(Verifier.SensorType.MidTube, 1000);
                // provider.startCalibrationMag(Verifier.SensorType.MidTube, 2000);


                provider.setAcelBias(Verifier.SensorType.MidTube, new Vector3((float)0.05044835, (float)-0.0003466018, (float)-0.001710713));  // 
                provider.setGyroBias(Verifier.SensorType.MidTube, new Vector3((float)-3.165749, (float)-5.542736, (float)2.636271));  // 
                Matrix4x4 m4x4End = Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                // provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)62.96, (float)311.49, (float)37.38), m4x4End);  // 
                provider.setMagBias(Verifier.SensorType.MidTube, new Vector3((float)19.01, (float)12.45, (float)2.22), m4x4End);  // (19.01, 12.45, 2.22)      IMU sin endoscopio


                provider.setAcelBias(Verifier.SensorType.EndTube, new Vector3((float)0.01189658, (float)-0.01439686, (float)0.01217377));  //  
                provider.setGyroBias(Verifier.SensorType.EndTube, new Vector3((float)0.4471607, (float)2.758421, (float)-0.5646203));  //  
                Matrix4x4 m4x4Mid = Matrix4x4.identity; // m4x4.m00 = (float)1.354;  m4x4.m11 = (float)0.851; m4x4.m22 = (float)0.921; 
                // provider.setMagBias(Verifier.SensorType.EndTube, new Vector3((float)12.87, (float)15.92, (float)-0.39), m4x4Mid);  // 
                provider.setMagBias(Verifier.SensorType.EndTube, new Vector3((float)17.40, (float)10.51, (float)-11.43), m4x4Mid);  // (17.40, 10.51, -11.43)    IMU sin endoscopio


                provider.setAcelBias(Verifier.SensorType.Internal, new Vector3((float)0.0, (float)-0.02, (float)-0.01)); // x e y cambiados
                provider.setGyroBias(Verifier.SensorType.Internal, new Vector3((float)0.87, (float)-2.65, (float)0.52));
                provider.setMagBias(Verifier.SensorType.Internal, new Vector3((float)44.67, (float)20.69, (float)+39.62), Matrix4x4.identity);

                provider.setAcelBias(Verifier.SensorType.Internal, new Vector3((float)0.03, (float)0.03, (float)-0.01)); // x e y cambiados
                provider.setGyroBias(Verifier.SensorType.Internal, new Vector3((float)-2.74, (float)0.87, (float)0.59));
                provider.setMagBias(Verifier.SensorType.Internal, new Vector3((float)42.66, (float)23.17, (float)38.91), Matrix4x4.identity);   //prueba

                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.EndTube, 500);
                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.MidTube, 500);
                provider.StartDynamicDeltaTimeCalibration(Verifier.SensorType.Internal, 500);

            }

            if (provider.isCalibrationAcel(Verifier.SensorType.MidTube) || provider.isCalibrationGyro(Verifier.SensorType.MidTube) || provider.isCalibrationMag(Verifier.SensorType.MidTube)  || provider.isCalibrationMag(Verifier.SensorType.EndTube) || provider.isCalibrationAcel(Verifier.SensorType.Internal) || provider.isCalibrationGyro(Verifier.SensorType.Internal) || provider.isCalibrationMag(Verifier.SensorType.Internal))
            {
                Debug.Log("Calibring sensors... ");   
                return; // Salir de Update si aún estamos calibrando
            }

            // if (!offsetCalibrated)
            // {
            //     provider.CalibrateRelativeRotation();
            //     offsetCalibrated = true;
            // }
            // provider.GetMagCalibrationReport(Verifier.SensorType.MidTube);
                // provider.GetMagCalibrationReport(Verifier.SensorType.EndTube);



                Verifier.SensorType sensor = Verifier.SensorType.EndTube;
            (float roll, float pitch, float yaw) = provider.GetOrientation(sensor);
            // Debug.Log($"SensorObject Roll: {roll:F1}°, Pitch: {pitch:F1}°, Yaw: {yaw:F1}°");


            // FusionAhrsFlags flags = provider.GetFlags(Verifier.SensorType.Internal);
            // FusionAhrsFlags flags = provider.GetFlags(Verifier.SensorType.MidTube);
            FusionAhrsFlags flags = provider.GetFlags(sensor);
            FusionAhrsInternalStates intStates = provider.GetInternalStates(sensor);
            
            Vector3 acelbias= provider.getAcelBias(sensor);
            Vector3 gyrobias= provider.getGyroBias(sensor);

            // Debug.Log($"Leyendo calibracion acel {acelbias.x} {acelbias.y} {acelbias.z}"); 
            // Debug.Log($"Leyendo calibracion gyro {gyrobias.x} {gyrobias.y} {gyrobias.z}");

            // if (flags.initialising || flags.angularRateRecovery || flags.accelerationRecovery || flags.magneticRecovery)
                // Debug.Log($"SensorObject Roll: {roll:F1}°, Pitch: {pitch:F1}°, Yaw: {yaw:F1}°, initialising: {flags.initialising}, angularRateRecovery: {flags.angularRateRecovery}, accelerationRecovery: {flags.accelerationRecovery}, magneticRecovery: {flags.magneticRecovery}");
    
            // if (intStates.accelerationError > 0 || intStates.accelerationRecoveryTrigger > 0 || intStates.accelerometerIgnored || intStates.magneticError > 0 || intStates.magneticRecoveryTrigger > 0 || intStates.magnetometerIgnored || flags.initialising || flags.angularRateRecovery || flags.accelerationRecovery || flags.magneticRecovery)
            // {
            //     Debug.Log(
            //         $"[AHRS] AccErr: {intStates.accelerationError:F4}, AccIgnored: {intStates.accelerometerIgnored}, " +
            //         $"AccRecTrig: {intStates.accelerationRecoveryTrigger:F4}, MagErr: {intStates.magneticError:F4}, " +
            //         $"MagIgnored: {intStates.magnetometerIgnored}, MagRecTrig: {intStates.magneticRecoveryTrigger:F4} " +
            //         $"initialising: {flags.initialising}, angularRateRecovery: {flags.angularRateRecovery}, " +
            //         $"accelerationRecovery: {flags.accelerationRecovery}, magneticRecovery: {flags.magneticRecovery}"
            //     );
            // }
        }

    }
}