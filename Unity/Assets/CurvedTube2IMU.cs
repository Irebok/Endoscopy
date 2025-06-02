using UnityEngine;

public class HalfCurvedTubeSimulator : MonoBehaviour
{
    public Verifier verifier;

    public Transform imuBase;
    public Transform imuTip;

    public Transform simBase;
    public Transform simTip;

    public float tubeLength = 1.0f;
    public LineRenderer arcRenderer;

    private bool offsetCalibrated = false;
    private float timer = 0f;
    private const float calibrationDelay = 10f;

    private long lastOffset = 0;


    void Update()
    {
        if (!offsetCalibrated)
        {
            timer += Time.deltaTime;
            if (timer >= calibrationDelay)
            {
                verifier.CalibrateRelativeRotation();
                offsetCalibrated = true;
                Debug.Log("Offset de rotación relativo calibrado tras 10 segundos.");
            }
            return;
        }

        long offset = verifier.GetPosition();
        if (offset != lastOffset)
        {
            float delta = (float)(offset - lastOffset)/50f; 
            simBase.position += new Vector3(0f, 0f, delta);
            lastOffset = offset;
        }
        
        Quaternion qBase = imuBase.rotation;
        Quaternion qTip = imuTip.rotation;
        Quaternion qMeasuredRelative = Quaternion.Inverse(qBase) * qTip;
        Quaternion qRelative = Quaternion.Inverse(verifier.referenceRelativeRotation) * qMeasuredRelative;

        // Eliminar el componente de roll (Z) de qRelative
        Vector3 relativeEuler = qRelative.eulerAngles;
        if (relativeEuler.x > 180f) relativeEuler.x -= 360f;
        if (relativeEuler.y > 180f) relativeEuler.y -= 360f;
        Quaternion qRelativeNoRoll = Quaternion.Euler(relativeEuler.x, relativeEuler.y, 0f);

        // Calcular dirección y puntos clave
        Vector3 baseForward = simBase.forward;
        Vector3 midPoint = simBase.position + baseForward * (tubeLength / 2f);
        Vector3 curvedDirection = qRelativeNoRoll * baseForward;

        // Aplicar roll de la base como rotación del plano de curvatura
        float roll = imuBase.rotation.eulerAngles.z;
        if (roll > 180f) roll -= 360f;
        Quaternion rollRotation = Quaternion.AngleAxis(roll, simBase.forward);
        Vector3 finalDirection = rollRotation * curvedDirection;

        // Obtencion de la posicion de la punta
        Vector3 tipPoint = midPoint + finalDirection.normalized * (tubeLength / 2f);

        // Debug.Log($"Direction {curvedDirection}\t Base roll {roll}\t Final direction {finalDirection}");

        simTip.position = tipPoint;
        simTip.rotation = Quaternion.LookRotation(finalDirection.normalized, simBase.up);


        arcRenderer.positionCount = 3;
        arcRenderer.SetPosition(0, simBase.position);
        arcRenderer.SetPosition(1, midPoint);
        arcRenderer.SetPosition(2, tipPoint);

        Debug.DrawRay(simBase.position, baseForward * 0.2f, Color.green);
        Debug.DrawRay(simTip.position, simTip.forward * 0.2f, Color.red);
    }
}