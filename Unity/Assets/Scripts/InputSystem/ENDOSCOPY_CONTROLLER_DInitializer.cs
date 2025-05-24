using UnityEngine;
using UnityEngine.InputSystem;

public class ENDOSCOPY_CONTROLLER_DInitializer : MonoBehaviour
{
    private void Awake()
    {
        InputSystem.RegisterLayout<ENDOSCOPY_CONTROLLER_Device>();
    }
}

