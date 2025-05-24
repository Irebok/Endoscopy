// using UnityEngine;

// public class TubeKinematics3D : MonoBehaviour
// {
//     public Transform midTube;
//     public Transform endTube;
//     public float tubeLength = 200f;

//     void Update()
//     {
//         // Obtener rotaciones
//         Quaternion rotMid = midTube.rotation;
//         Quaternion rotEnd = endTube.rotation;

//         // Calcular rotación relativa
//         Quaternion deltaRot = Quaternion.Inverse(rotMid) * rotEnd;
//         deltaRot.ToAngleAxis(out float angleDeg, out Vector3 axis);
//         float angleRad = angleDeg * Mathf.Deg2Rad;

//         // Manejar casos extremos
//         if (float.IsNaN(angleRad) || axis == Vector3.zero || Mathf.Approximately(angleRad, 0f))
//         {
//             endTube.position = midTube.position + midTube.forward * tubeLength;
//             return;
//         }

//         // Calcular radio del arco
//         float radius = tubeLength / angleRad;

//         // Eje perpendicular a la dirección del tubo
//         Vector3 dir = midTube.forward;
//         Vector3 centerOffset = Quaternion.AngleAxis(90, axis) * dir * radius;
//         Vector3 center = midTube.position + centerOffset;

//         // Vector desde centro al inicio
//         Vector3 startVec = midTube.position - center;

//         // Calcular posición final al aplicar la rotación
//         Vector3 endVec = Quaternion.AngleAxis(angleDeg, axis) * startVec;
//         Vector3 endPos = center + endVec;

//         endTube.position = endPos;

//         Debug.DrawLine(midTube.position, center, Color.yellow);
//         Debug.DrawLine(center, endPos, Color.green);
//         Debug.Log($"angleDeg: {angleDeg}, angleRad: {angleRad}, tubeLength: {tubeLength}, radiusCalc: {tubeLength / angleRad}");
//     }
// }
