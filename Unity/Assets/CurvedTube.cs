using UnityEngine;

public class HalfCurvedTubeSimulator : MonoBehaviour
{
    public Verifier verifier;

    public Transform imuBase;  // Sensor físico base
    public Transform imuTip;   // Sensor físico punta

    public Transform simBase;  // Objeto fijo
    public Transform simTip;   // Punta del tubo

    public float tubeLength = 1.0f;
    public LineRenderer arcRenderer;

    private bool offsetCalibrated = false;
    private float timer = 0f;
    private const float calibrationDelay = 3f;

    void Update()
    {
        // Espera para calibración
        if (!offsetCalibrated)
        {
            timer += Time.deltaTime;
            if (timer >= calibrationDelay)
            {
                verifier.CalibrateRelativeRotation();
                offsetCalibrated = true;
                Debug.Log("✅ Offset de rotación relativo calibrado tras 3 segundos.");
            }
            return;
        }

        // 1. Obtener cuaterniones desde los sensores
        Quaternion qBase = imuBase.rotation;
        Quaternion qTip = imuTip.rotation;

        // 2. Rotación relativa compensada
        Quaternion qMeasuredRelative = Quaternion.Inverse(qBase) * qTip;
        Quaternion qRelative = Quaternion.Inverse(verifier.referenceRelativeRotation) * qMeasuredRelative;

        // 3. Calcular dirección rotada desde base
        Vector3 rotatedDirection = qRelative * simBase.forward;

        // 4. Posicionar y rotar punta
        simTip.position = simBase.position + rotatedDirection.normalized * tubeLength;
        simTip.rotation = qBase * qRelative;

        // 5. Dibujar línea recta
        arcRenderer.positionCount = 2;
        arcRenderer.SetPosition(0, simBase.position);
        arcRenderer.SetPosition(1, simTip.position);

        // 6. Debug visual
        Debug.DrawRay(simBase.position, simBase.forward * 0.2f, Color.green);  // base forward
        Debug.DrawRay(simTip.position, simTip.forward * 0.2f, Color.red);      // punta forward
    }
}
