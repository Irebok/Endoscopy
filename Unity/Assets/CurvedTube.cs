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

        // 0. Actualizar simMid solo si cambia el offset
        long offset = verifier.GetPosition();
        if (offset != lastOffset)
        {
            float delta = (float)(offset - lastOffset)/50f; // ajustar escala si necesario
            simBase.position += new Vector3(0f, 0f, delta);
            lastOffset = offset;
        }
        
        // 1. Obtener rotaciones
        Quaternion qBase = imuBase.rotation;
        Quaternion qTip = imuTip.rotation;

        Vector3 eulerBase = qBase.eulerAngles;
        Vector3 eulerTip = qTip.eulerAngles;


        // 2. Calcular rotación relativa compensada
        Quaternion qMeasuredRelative = Quaternion.Inverse(qBase) * qTip;
        Quaternion qRelative = Quaternion.Inverse(verifier.referenceRelativeRotation) * qMeasuredRelative;

        // 3. Calcular dirección y puntos clave
        Vector3 baseForward = simBase.forward;
        Vector3 midPoint = simBase.position + baseForward * (tubeLength / 2f);
        Vector3 curvedDirection = qRelative * baseForward;
        Vector3 tipPoint = midPoint + curvedDirection.normalized * (tubeLength / 2f);

        // 4. Posicionar y rotar la punta
        simTip.position = tipPoint;
        simTip.rotation = qRelative;
        // Debug.Log($" Euler XYZ: [Base] {eulerBase.x:F1}°, {eulerBase.y:F1}°, {eulerBase.z:F1}° [Tip ] {eulerTip.x:F1}°, {eulerTip.y:F1}°, {eulerTip.z:F1}° [Relative] {curvedDirection.x:F1}°, {curvedDirection.y:F1}°, {curvedDirection.z:F1}°");


        // 5. Dibujar arco (línea compuesta)
        arcRenderer.positionCount = 3;
        arcRenderer.SetPosition(0, simBase.position);
        arcRenderer.SetPosition(1, midPoint);
        arcRenderer.SetPosition(2, tipPoint);

        // 6. Debug visual
        Debug.DrawRay(simBase.position, baseForward * 0.2f, Color.green);
        Debug.DrawRay(simTip.position, simTip.forward * 0.2f, Color.red);
    }
}
