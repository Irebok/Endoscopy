using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;
using static FusionWrapper;
using Kalmanspace;
using System.IO;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Text.RegularExpressions;
using Unity.Collections;
using UnityEditor.VersionControl;
using UnityEngine.UI;


public class Verifier : MonoBehaviour
{
    private ENDOSCOPY_CONTROLLER_Device device;

    public Transform CubeMando;
    public Transform CubeMidTube;
    public Transform CubeEndTube;
    public Transform CubePositionTube;
    public Transform CubeButton1;
    public Transform CubeButton2;
    public Transform CubeButton3;
    public Quaternion referenceRelativeRotation;

    private struct SensorData
    {
        public int id;
        public Vector3 accelerometer;
        public Vector3 gyroscope;
        public Vector3 magnetometer;
        public uint dtAccel;
        public uint dtGyro;
        public uint dtMag;
        public uint accelEnabled;
        public uint gyroEnabled;
        public uint magEnabled;
        public bool accelReceived;
        public bool gyroReceived;
        public bool magReceived;
        public uint timestamp;
        public float roll;
        public float pitch;
        public float yaw;
        public Quaternion quaternion;
        public KalmanVector3 kalmanIMU;
        public FusionOffset offset;
        public FusionAhrs ahrs;
        public Vector3 acelBias;
        public Vector3 gyroBias;
        public Vector3 magBias;
        public float deltaTimeSum;
        public Matrix4x4 softIronMatrix;
        public Matrix4x4 inverseSoftIronMatrix;
        public Vector3[] magCalibrationSamples;
        public int calibrationSamplesCollectedAcel;
        public int calibrationSamplesCollectedGyro;
        public int calibrationSamplesCollectedMag;
        public int deltaTimeSamplesCollected;
        public Vector3 calibrationSumAcel;
        public Vector3 calibrationSumGyro;
        public Vector3 calibrationMinMag;
        public Vector3 calibrationMaxMag;
        public Vector3 magSum;
        public bool isCalibratingAcel;
        public int acelCalibratingSamples;
        public bool isCalibratingGyro;
        public int gyroCalibratingSamples;
        public bool isCalibratingMag;
        public int magCalibratingSamples;
        public bool isCalibratingDeltaTime;
        public int deltaTimeCalibratingSamples;
        public bool isCalibratingRelatOffset;
        public int magSampleIndex;
        public float lastTimestamp;
        public float deltaTime;
        
    }

    FusionAhrsSettings settings;
    private struct Encoder
    {
        public Transform Cube;
        public int id;
        public long value;
        public long offset;
    }
    private struct Buttons
    {
        public Transform Button1;
        public Transform Button2;
        public Transform Button3;
        public int id;
        public int value1;
        public int value2;
        public int value3;
    }

    public enum SensorType
    {
        Internal = 1,
        MidTube = 2,
        EndTube = 3,
        PositionTube = 4,
    }

    private SensorData internalSensorData;
    private SensorData tubeSensorData;
    private SensorData endSensorData;
    private Encoder encoderSensorData;
    private Buttons buttonsSensorData;
    private ref SensorData getSensor(SensorType type)
    {
        switch (type)
        {
            case SensorType.Internal:
                return ref internalSensorData;
            case SensorType.MidTube:
                return ref tubeSensorData;
            case SensorType.EndTube:
                return ref endSensorData;
            default:
                return ref internalSensorData;
        }
    }

    private string logFilePath;
    private StreamWriter logWriter;

    readonly FusionMatrix gyroscopeMisalignment = new FusionMatrix(
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
    );
    readonly FusionVector gyroscopeSensitivity = new FusionVector(1.0f, 1.0f, 1.0f);
    // Acelerómetro
    readonly FusionMatrix accelerometerMisalignment = new FusionMatrix(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    );

    readonly FusionVector accelerometerSensitivity = new FusionVector(1.0f, 1.0f, 1.0f);

    // private FusionAhrs ahrs;
    // private float lastTimestamp;
    // private float lastTimestampMag;
    private int incompleteFrames = 0;
    private bool trackingTime = false;
    private float startTime = 0f;
    private byte lastTrackedTimestamp = 0;


    // Calibración (se puede actualizar con tus datos reales luego)
    private readonly FusionMatrix identityMatrix = new FusionMatrix {
        m00 = 1, m11 = 1, m22 = 1
    };
    private readonly FusionVector zeroVector = new FusionVector { x = 0, y = 0, z = 0 };
    private readonly FusionVector oneVector = new FusionVector { x = 1, y = 1, z = 1 };

    void Start() 
    {

    }

    private void OnDisable()
    {
        InputSystem.onEvent -= OnInputEvent;
        if (logWriter != null)
            logWriter.Close();
    }

    void initializeSensors(ref SensorData sensor, ref FusionAhrsSettings settings, int id)
    {
        FusionWrapper.FusionOffsetInitialise(ref sensor.offset, FusionWrapper.SAMPLE_RATE);
        FusionWrapper.FusionAhrsInitialise(ref sensor.ahrs);
        FusionWrapper.FusionAhrsSetSettings(ref sensor.ahrs, ref settings);

        sensor.id = id;
        sensor.kalmanIMU = new KalmanVector3(new Vector3(0f, 0f, 0f));
        sensor.roll = 0;
        sensor.pitch = 0;
        sensor.yaw = 0;

        sensor.acelBias = new Vector3();
        sensor.calibrationSamplesCollectedAcel = 0;
        sensor.calibrationSumAcel = Vector3.zero;
        sensor.isCalibratingAcel = false;
        sensor.acelCalibratingSamples = 1000;

        sensor.gyroBias = new Vector3();
        sensor.calibrationSamplesCollectedGyro = 0;
        sensor.calibrationSumGyro = Vector3.zero;
        sensor.isCalibratingGyro = false;
        sensor.gyroCalibratingSamples = 1000;

        sensor.magBias = new Vector3();
        sensor.calibrationSamplesCollectedMag = 0;
        sensor.calibrationMinMag = Vector3.zero;
        sensor.calibrationMaxMag = Vector3.zero;
        sensor.magSum = Vector3.zero;
        sensor.isCalibratingMag = false;
        sensor.magCalibratingSamples = 1000;

        // sensor.magCalibrationSamples = new Vector3[magCalibratingSamples];

        sensor.softIronMatrix = Matrix4x4.identity;
        sensor.inverseSoftIronMatrix = Matrix4x4.identity;
        sensor.lastTimestamp = Time.realtimeSinceStartup;

    }

    void initializeEncoder(ref Encoder sensor, int id)
    {
        sensor.id = 4;
        sensor.value = 0;
        sensor.offset = 0;
    }
    
    void initializeButtons(ref Buttons sensor, int id)
    {
        sensor.id = 5;
        sensor.value1 = 0;
        sensor.value2 = 0;
        sensor.value3 = 0;
    }

    private void OnEnable()
    {
        InputSystem.onEvent += OnInputEvent;

        // Inicializar la fusión
        // FusionWrapper.FusionOffsetInitialise(ref offset, FusionWrapper.SAMPLE_RATE);
        settings = new FusionAhrsSettings
        {
            // convention = 0, // NWU
            // gain = 0.2f,
            // gyroscopeRange = 2000.0f,
            // accelerationRejection = 70f,
            // magneticRejection = 70f,
            // recoveryTriggerPeriod = SAMPLE_RATE*1;
            convention = 0, // NWU
            gain = 0.15f,
            gyroscopeRange = 2000.0f,
            accelerationRejection = 15f,
            magneticRejection = 15f,
            recoveryTriggerPeriod = SAMPLE_RATE * 5
        };

        initializeSensors(ref internalSensorData, ref settings, 1);
        initializeSensors(ref tubeSensorData, ref settings, 2);
        initializeSensors(ref endSensorData, ref settings, 3);
        initializeEncoder(ref encoderSensorData, 4);
        initializeButtons(ref buttonsSensorData, 5);

        referenceRelativeRotation = new Quaternion(0, 0, 0, 0);

        logFilePath = Application.persistentDataPath + "/sensor_log.csv";
        logWriter = new StreamWriter(logFilePath, false); // false = sobrescribe cada ejecución
        logWriter.AutoFlush = true;
    }

    bool IsMagnetometerEnabled(uint flags) => (flags & 0b100) != 0;

    
    private void OnInputEvent(InputEventPtr eventPtr, InputDevice inputDevice)
    {
        // Check if the event is a valid StateEvent
        if (!(eventPtr.IsA<StateEvent>()))
            return;

        // Ensure inputDevice is the correct type
        if (!(inputDevice is ENDOSCOPY_CONTROLLER_Device ctrl))
        {
            return;
        }

        device = ctrl;  // Initialize the device

        if (device == null)
        {
            Debug.LogError("ENDOSCOPY_CONTROLLER_Device initialization failed.");
            return;
        }

        float now = Time.realtimeSinceStartup;


        var id = ctrl.GetID();
        var (ax, ay, az) = ctrl.GetAxisAcelStates();
        var (gx, gy, gz) = ctrl.GetAxisGyroStates();
        var (mx, my, mz) = ctrl.GetAxisMagStates();
        uint enable_data = ctrl.GetEnableData();
        Debug.Log($"Received id: {id} {enable_data}");

        if (id == 4)
        {
            var (mayor_peso, menor_peso) = ctrl.GetPositionState();
            encoderSensorData.value = ((mayor_peso << 16) | menor_peso) - encoderSensorData.offset;
            string line = string.Format("{0:F4};{1:F4};{2:F4};{3:F4};{4:F4};{5:F4};{6:F4};{7:F4};{8:F4};{9:F4};{10:F4};{11:F4};{12:F4};{13:F4};{14:F4};{15:F4};{16:F4};{17:F4};{18:F4};{19:F2};{20:F2};{21:F2};{22:F1}",
            Time.realtimeSinceStartup,
            encoderSensorData.value, encoderSensorData.offset, mayor_peso,
            menor_peso, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            encoderSensorData.id);
            logWriter.WriteLine(line);
            if (CubePositionTube != null)
            {
                CubePositionTube.localPosition = new Vector3(
                    ((float)encoderSensorData.value) / 50,
                    CubePositionTube.localPosition.y,
                    CubePositionTube.localPosition.z
                );
            }
        }
        else if (id == 5)
        {
            buttonsSensorData.value1 = ((enable_data & 0b00000001) > 1) ? buttonsSensorData.value1 = 1 : buttonsSensorData.value1 = 0;
            buttonsSensorData.value2 = ((enable_data & 0b00000010) > 1) ? buttonsSensorData.value2 = 1 : buttonsSensorData.value2 = 0;
            buttonsSensorData.value3 = ((enable_data & 0b00000100) > 1) ? buttonsSensorData.value3 = 1 : buttonsSensorData.value3 = 0;

            UpdateButtonColor(ref CubeButton1, buttonsSensorData.value1);
            UpdateButtonColor(ref CubeButton2, buttonsSensorData.value2);
            UpdateButtonColor(ref CubeButton3, buttonsSensorData.value3);
            Debug.Log($"Buttons value: {buttonsSensorData.value1} {buttonsSensorData.value2} {buttonsSensorData.value3}");
        }
        else
        {
            SensorType type = (SensorType)id;
            ref SensorData currentSensorData = ref getSensor(type);

            currentSensorData.deltaTime = now - currentSensorData.lastTimestamp;
            currentSensorData.lastTimestamp = now;


            currentSensorData.accelerometer = new Vector3(-ay, -ax, az);
            currentSensorData.accelReceived = true;
            currentSensorData.accelEnabled = enable_data;

            currentSensorData.gyroscope = new Vector3(-gy, -gx, gz);
            currentSensorData.gyroReceived = true;
            currentSensorData.gyroEnabled = enable_data;

            if ((enable_data & 0b100) != 0) currentSensorData.kalmanIMU.Step(currentSensorData.deltaTime, new Vector3(-mx, -my, mz));
            else currentSensorData.kalmanIMU.Step(currentSensorData.deltaTime);

            currentSensorData.magnetometer = currentSensorData.kalmanIMU.GetEstimate();
            currentSensorData.magReceived = true;
            currentSensorData.magEnabled = enable_data;

            if (enable_data >= 3) ProcessSensorData(ref currentSensorData);

            if (currentSensorData.isCalibratingAcel || currentSensorData.isCalibratingGyro || currentSensorData.isCalibratingMag || currentSensorData.isCalibratingDeltaTime)
            {
                if (currentSensorData.isCalibratingAcel) acelCalibration(ref currentSensorData, currentSensorData.accelerometer);
                if (currentSensorData.isCalibratingGyro) gyroCalibration(ref currentSensorData, currentSensorData.gyroscope);
                if (currentSensorData.isCalibratingMag) magCalibration(ref currentSensorData, currentSensorData.magnetometer);
                if (currentSensorData.isCalibratingDeltaTime) deltaTimeCalibration(ref currentSensorData);
            }
        }
    }

    
    void UpdateButtonColor(ref Transform buttonTransform, int value)
    {
        
        if (buttonTransform == null) {
            return;
        }
        
        var renderer = buttonTransform.GetComponent<Renderer>();
        if (renderer == null) {
            return;
        }
        renderer.material.SetColor("_BaseColor", (value == 1) ? Color.red : Color.blue);
    }
    private void ProcessSensorData(ref SensorData data)
    {    
        var acel = new FusionVector { x = data.accelerometer.x, y = data.accelerometer.y, z = data.accelerometer.z};
        var gyro = new FusionVector { x = data.gyroscope.x, y = data.gyroscope.y, z = data.gyroscope.z}; // convert to deg/s
        var mag = new FusionVector { x = data.magnetometer.x, y = data.magnetometer.y, z = data.magnetometer.z};


        // Calibración
        var gyro_cal = FusionCalibrationInertial(gyro, gyroscopeMisalignment, gyroscopeSensitivity, new FusionVector(data.gyroBias));
        var acel_cal = FusionCalibrationInertial(acel, accelerometerMisalignment, accelerometerSensitivity, new FusionVector(data.acelBias));
        var mag_cal = FusionCalibrationMagnetic(mag, new FusionMatrix(data.softIronMatrix), new FusionVector(data.magBias));

        var gyro_cal_offset = FusionOffsetUpdate(ref data.offset, gyro_cal);
        // if(data.id == (int)SensorType.EndTube)
        //     Debug.Log($"Mag: {mag.x} {mag.y} {mag.z}  Mag_cal: {mag_cal.x} {mag_cal.y} {mag_cal.z}  Gyro_cal_offset: {gyro_cal_offset.x} {gyro_cal_offset.y} {gyro_cal_offset.z}  Delta: {data.deltaTime}");



        // Ejecutamos AHRS
        FusionAhrsUpdate(ref data.ahrs, gyro_cal_offset, acel_cal, mag_cal, data.deltaTime);


        // Obtenemos resultados
        var quatPtr = FusionAhrsGetQuaternion(ref data.ahrs);
        // Quaternion rotation = ToUnityQuaternion(quatPtr);
        var euler = FusionQuaternionToEuler(quatPtr);
        Quaternion rotation = Quaternion.Euler(data.pitch, data.yaw, data.roll);
        data.roll = euler.roll;
        data.pitch = euler.pitch;
        data.yaw = euler.yaw;
        data.quaternion = rotation;

        if (data.id == (int)SensorType.Internal && CubeMando != null)
        {
            CubeMando.rotation = rotation;
        }
        else if (data.id == (int)SensorType.MidTube && CubeMidTube != null)
        {
            CubeMidTube.rotation = rotation;
        }
        else if (data.id == (int)SensorType.EndTube && CubeEndTube != null)
        {
            CubeEndTube.rotation = rotation;
        }

        // Debug: Mostrar resultados AHRS
        // Debug.Log($"[n] Roll: {euler.roll:F1}°, Pitch: {euler.pitch:F1}°, Yaw: {euler.yaw:F1}°");
        // Debug.Log($"[Earth AcFusiocel] X: {earthAccel.x:F2}, Y: {earthAccel.y:F2}, Z: {earthAccel.z:F2}");
        // if(data.id == 3)
        //     Debug.Log($"Data received: Δt={data.deltaTime:F8}, " +
        //         $"accel=({acel.x:F4}|{acel.y:F4}|{acel.z:F4}), " +
        //         $"gyro=({gyro.x:F4}|{gyro.y:F4}|{gyro.z:F4}), " +
        //         $"mag=({mag.x:F4}|{mag.y:F4}|{mag.z:F4}), " +
        //         $"accel_c=({acel_cal.x:F4}|{acel_cal.y:F4}|{acel_cal.z:F4})" +
        //         $"gyro_c=({gyro_cal.x:F4}|{gyro_cal.y:F4}|{gyro_cal.z:F4})" +
        //         $"mag_c=({mag_cal.x:F4}|{mag_cal.y:F4}|{mag_cal.z:F4})");
        string line = string.Format("{0:F4};{1:F4};{2:F4};{3:F4};{4:F4};{5:F4};{6:F4};{7:F4};{8:F4};{9:F4};{10:F4};{11:F4};{12:F4};{13:F4};{14:F4};{15:F4};{16:F4};{17:F4};{18:F4};{19:F2};{20:F2};{21:F2};{22:F1}",
            Time.realtimeSinceStartup,
            acel.x, acel.y, acel.z,
            gyro.x, gyro.y, gyro.z,
            mag.x, mag.y, mag.z,
            acel_cal.x, acel_cal.y, acel_cal.z,
            gyro_cal_offset.x, gyro_cal_offset.y, gyro_cal_offset.z,
            mag_cal.x, mag_cal.y, mag_cal.z,
            data.roll, data.pitch, data.yaw,
            data.id);
            logWriter.WriteLine(line);
        // }
    }


    private void acelCalibration(ref SensorData currentSensorData, Vector3 calValue)//
    {       
        // Debug.Log($"Calibring acel {currentSensorData.id}");
        currentSensorData.calibrationSumAcel += calValue;
        currentSensorData.calibrationSamplesCollectedAcel++;

        if (currentSensorData.calibrationSamplesCollectedAcel >= currentSensorData.acelCalibratingSamples) {
            currentSensorData.acelBias = (currentSensorData.calibrationSumAcel/currentSensorData.acelCalibratingSamples) - new Vector3(0,0,1);
            currentSensorData.isCalibratingAcel = false;
            Debug.Log($"Acelerometer calibration completed {currentSensorData.id}. Bias: {currentSensorData.acelBias.x:F2}, {currentSensorData.acelBias.y:F2}, {currentSensorData.acelBias.z:F2}");
        }
        return;
    }

    private void gyroCalibration(ref SensorData currentSensorData, Vector3 calValue)
    {
            currentSensorData.calibrationSumGyro += calValue;
            currentSensorData.calibrationSamplesCollectedGyro++;

            if (currentSensorData.calibrationSamplesCollectedGyro >= currentSensorData.gyroCalibratingSamples) {
                currentSensorData.gyroBias = currentSensorData.calibrationSumGyro / currentSensorData.gyroCalibratingSamples;
                currentSensorData.isCalibratingGyro = false;
                Debug.Log($"Gyroscope calibration completed {currentSensorData.id}. Bias: {currentSensorData.gyroBias.x:F2}, {currentSensorData.gyroBias.y:F2}, {currentSensorData.gyroBias.z:F2}");
            }
        return;
    }


    private void magCalibration(ref SensorData currentSensorData, Vector3 calValue)
    {
        if (currentSensorData.magSampleIndex < currentSensorData.magCalibratingSamples)
        {
            currentSensorData.magCalibrationSamples[currentSensorData.magSampleIndex++] = calValue;
        }

        if (currentSensorData.magSampleIndex >= currentSensorData.magCalibratingSamples)
        {
            // Hard iron correction
            Vector3 min = currentSensorData.magCalibrationSamples[0];
            Vector3 max = currentSensorData.magCalibrationSamples[0];

            for (int i = 1; i < currentSensorData.magCalibratingSamples; i++)
            {
                min = Vector3.Min(min, currentSensorData.magCalibrationSamples[i]);
                max = Vector3.Max(max, currentSensorData.magCalibrationSamples[i]);
            }
            Debug.Log($"Magnetometer calibration completed.\nMin: {min}\nMax:{max}");

            currentSensorData.magBias = (max + min) / 2.0f;
            Vector3 scale = (max - min) / 2.0f;

            float avg = (scale.x + scale.y + scale.z) / 3.0f;
            currentSensorData.softIronMatrix = Matrix4x4.identity;
            currentSensorData.softIronMatrix.m00 = avg / scale.x;
            currentSensorData.softIronMatrix.m11 = avg / scale.y;
            currentSensorData.softIronMatrix.m22 = avg / scale.z;
            currentSensorData.inverseSoftIronMatrix = currentSensorData.softIronMatrix.inverse;

            currentSensorData.isCalibratingMag = false;
            Debug.Log($"Magnetometer calibration completed.\nBias: {currentSensorData.magBias}\nSoft iron matrix:\n{currentSensorData.softIronMatrix}");
        }
    }

    private void deltaTimeCalibration(ref SensorData currentSensorData)
    {
        currentSensorData.deltaTimeSum += currentSensorData.deltaTime;
        currentSensorData.deltaTimeSamplesCollected++;

        if (currentSensorData.deltaTimeSamplesCollected >= currentSensorData.deltaTimeCalibratingSamples)
        {
            float avgDelta = currentSensorData.deltaTimeSum / currentSensorData.deltaTimeSamplesCollected;
            uint sampleRate = (uint)(1 / avgDelta);
            settings.recoveryTriggerPeriod = sampleRate; // adaptado a ms


            FusionAhrsSetSettings(ref currentSensorData.ahrs, ref settings);
            currentSensorData.offset.timeout = sampleRate * 5;
            currentSensorData.isCalibratingDeltaTime = false;

            Debug.Log($"[✅] Calibración de deltaTime completada. Periodo promedio: {currentSensorData.id} {sampleRate:F6}s");
        }
    }

    private void StartGenericCalibration()
    {
        internalSensorData.isCalibratingAcel = true;
        internalSensorData.isCalibratingGyro = true;
        tubeSensorData.isCalibratingAcel = true;
        tubeSensorData.isCalibratingGyro = true;
        endSensorData.isCalibratingAcel = true;
        endSensorData.isCalibratingGyro = true;
        Debug.Log("Starting acelerometer and gyroscope calibration...");
    }

    public (float, float, float) GetOrientation(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return (sensor.roll, sensor.pitch, sensor.yaw);       
    }

    public Quaternion getSensorQuaternion(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.quaternion;
    } 

    public long GetPosition()
    {
        return encoderSensorData.value;
    }

    public long GetOffsetPosition()
    {
        return encoderSensorData.offset;       
    }

    public void setOffsetPosition(int newOffset)
    {
        encoderSensorData.offset = newOffset;
        return;       
    } 

    public (bool, bool, bool) GetButtons()
    {
        return(buttonsSensorData.value1>0, buttonsSensorData.value2>0, buttonsSensorData.value3>0);       
    } 


    public void startCalibrationAcel(SensorType type, int numSamples)
    {

        ref SensorData sensor = ref getSensor(type);
        sensor.calibrationSumAcel += new Vector3(0, 0, 0);
        sensor.calibrationSamplesCollectedAcel = 0;
        sensor.isCalibratingAcel = true;
        sensor.acelCalibratingSamples = numSamples;
        return;
    } 

    public void startCalibrationGyro(SensorType type, int numSamples)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.calibrationSumGyro += new Vector3(0, 0, 0);
        sensor.calibrationSamplesCollectedGyro = 0;
        sensor.isCalibratingGyro = true;
        sensor.gyroCalibratingSamples = numSamples;
        return;
    } 

    public void startCalibrationMag(SensorType type, int numSamples)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.calibrationMaxMag = new Vector3(0, 0, 0);
        sensor.calibrationMinMag = new Vector3(0, 0, 0);
        sensor.magSum =  new Vector3(0, 0, 0);
        sensor.calibrationSamplesCollectedMag = 0;
        sensor.magSampleIndex = 0;
        sensor.isCalibratingMag = true;
        sensor.magCalibratingSamples = numSamples;
        sensor.magCalibrationSamples = new Vector3[sensor.magCalibratingSamples];
        return;
    } 
    public void StartDynamicDeltaTimeCalibration(SensorType type, int numSamples)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.isCalibratingDeltaTime = true;
        sensor.deltaTimeCalibratingSamples = numSamples;
        sensor.deltaTimeSamplesCollected = 0;
        sensor.deltaTimeSum = 0f;
    }

    public void CalibrateRelativeRotation()
    {
        var qMid = tubeSensorData.quaternion;
        var qEnd = endSensorData.quaternion;
        referenceRelativeRotation = Quaternion.Inverse(qMid) * qEnd;
        Debug.Log($"Calibrated relative rotation. {qMid} {qEnd} {referenceRelativeRotation}");
    }
    public bool isCalibrationAcel(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.isCalibratingAcel;
    } 

    public bool isCalibrationGyro(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.isCalibratingGyro;
    } 

    public bool isCalibrationMag(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.isCalibratingMag;
    } 

    public Vector3 getAcelBias(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.acelBias;
    } 

        public Vector3 getGyroBias(SensorType type)
    {       
        ref SensorData sensor = ref getSensor(type);
        return sensor.gyroBias;
    } 

    public Vector3 getMagBias(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return sensor.magBias;
    } 

    public void setAcelBias(SensorType type, Vector3 bias)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.acelBias = bias;
    } 

    public void setGyroBias(SensorType type, Vector3 bias)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.gyroBias = bias;
    } 

    public void setMagBias(SensorType type, Vector3 bias, Matrix4x4 softIron /*, Matrix<double> softIron*/)
    {
        ref SensorData sensor = ref getSensor(type);
        sensor.magBias = bias;
        sensor.softIronMatrix = softIron;
    }

    public static Quaternion ToUnityQuaternion(FusionQuaternion q) {
        // Inversión de X, Y para adaptarse a convenciones de Unity
        return new Quaternion(-q.y, -q.x, q.z, q.w);
    }


    private string MatrixToString(Matrix4x4 m)
    {
        return $"{m.m00:F3} {m.m01:F3} {m.m02:F3}\n" +
            $"{m.m10:F3} {m.m11:F3} {m.m12:F3}\n" +
            $"{m.m20:F3} {m.m21:F3} {m.m22:F3}";
    }
    
    private string MatrixToString(Matrix<double> m)
    {
        if (m.RowCount != 3 || m.ColumnCount != 3)
        {
            throw new ArgumentException("Matrix must be 3x3.");
        }

        return $"{m[0, 0]:F3} {m[0, 1]:F3} {m[0, 2]:F3}\n" +
            $"{m[1, 0]:F3} {m[1, 1]:F3} {m[1, 2]:F3}\n" +
            $"{m[2, 0]:F3} {m[2, 1]:F3} {m[2, 2]:F3}";
    }

    public void GetMagCalibrationReport(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);

        Debug.Log("---- Magnetometer Calibration Report ----");
        Debug.Log($"Samples collected: {sensor.magSampleIndex}");

        Debug.Log($"Bias (Hard Iron Offset):\n{sensor.magBias}");

        // Imprimimos la matriz de corrección Soft Iron
        Debug.Log("Soft Iron Correction Matrix:");
        Debug.Log(MatrixToString(sensor.softIronMatrix));
        
        Debug.Log("-----------------------------------------");
    }

    public FusionAhrsFlags GetFlags(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return FusionAhrsGetFlags(ref sensor.ahrs);
    }

    public FusionAhrsInternalStates  GetInternalStates(SensorType type)
    {
        ref SensorData sensor = ref getSensor(type);
        return FusionAhrsGetInternalStates(ref sensor.ahrs);
    }

}
