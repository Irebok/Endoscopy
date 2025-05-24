using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Layouts;
using UnityEngine.InputSystem.LowLevel;
using UnityEngine.InputSystem.Utilities;
using UnityEngine.InputSystem.Controls;

#if UNITY_EDITOR
using UnityEditor;
#endif

struct ENDOSCOPY_CONTROLLER_DeviceState : IInputStateTypeInfo
{
    public FourCC format => new FourCC('H', 'I', 'D');

    [InputControl(name = "axisAX", layout = "Integer", offset = 1, format = "SHRT")]
    public short axisAX;

    [InputControl(name = "axisAY", layout = "Integer", offset = 3, format = "SHRT")]
    public short axisAY;

    [InputControl(name = "axisAZ", layout = "Integer", offset = 5, format = "SHRT")]
    public short axisAZ;

    [InputControl(name = "axisGX", layout = "Integer", offset = 7, format = "SHRT")]
    public short axisGX;

    [InputControl(name = "axisGY", layout = "Integer", offset = 9, format = "SHRT")]
    public short axisGY;

    [InputControl(name = "axisGZ", layout = "Integer", offset = 11, format = "SHRT")]
    public short axisGZ;

    [InputControl(name = "axisMX", layout = "Integer", offset = 13, format = "SHRT")]
    public short axisMX;

    [InputControl(name = "axisMY", layout = "Integer", offset = 15, format = "SHRT")]
    public short axisMY;

    [InputControl(name = "axisMZ", layout = "Integer", offset = 17, format = "SHRT")]
    public short axisMZ;

    [InputControl(name = "id", layout = "Integer", offset = 19, format = "BYTE")]
    public byte id;

    [InputControl(name = "enableData", layout = "Integer", offset = 20, format = "BYTE")]
    public byte enableData;

    // [InputControl(name = "button1", layout = "Button", bit = 0)]
    // public ButtonControl button1 { get; private set; }

    // [InputControl(name = "button2", layout = "Button", bit = 1)]
    // public ButtonControl button2 { get; private set; }

    // [InputControl(name = "button3", layout = "Button", bit = 2)]
    // public ButtonControl button3 { get; private set; }

    // [InputControl(name = "button4", layout = "Button", bit = 3)]
    // public ButtonControl button4 { get; private set; }
}

[InputControlLayout(stateType = typeof(ENDOSCOPY_CONTROLLER_DeviceState), displayName = "ENDOSCOPY CONTROLLER HID Device", stateFormat = "HID")]
#if UNITY_EDITOR
[InitializeOnLoad]
#endif
public class ENDOSCOPY_CONTROLLER_Device : InputDevice
{
    public IntegerControl axisAX { get; private set; }
    public IntegerControl axisAY { get; private set; }
    public IntegerControl axisAZ { get; private set; }
    public IntegerControl axisGX { get; private set; }
    public IntegerControl axisGY { get; private set; }
    public IntegerControl axisGZ { get; private set; }
    public IntegerControl axisMX { get; private set; }
    public IntegerControl axisMY { get; private set; }
    public IntegerControl axisMZ { get; private set; }
    public IntegerControl id { get; private set; }
    public IntegerControl enableData { get; private set; }

    protected override void FinishSetup()
    {
        base.FinishSetup();
        axisAX = GetChildControl<IntegerControl>("axisAX");
        axisAY = GetChildControl<IntegerControl>("axisAY");
        axisAZ = GetChildControl<IntegerControl>("axisAZ");
        axisGX = GetChildControl<IntegerControl>("axisGX");
        axisGY = GetChildControl<IntegerControl>("axisGY");
        axisGZ = GetChildControl<IntegerControl>("axisGZ");
        axisMX = GetChildControl<IntegerControl>("axisMX");
        axisMY = GetChildControl<IntegerControl>("axisMY");
        axisMZ = GetChildControl<IntegerControl>("axisMZ");
        id = GetChildControl<IntegerControl>("id");
        enableData = GetChildControl<IntegerControl>("enableData");
    }

    public (int  axisX, int  axisY) GetPositionState()
    {
        return (axisAX.ReadValue(), axisAY.ReadValue());
    }
    public (float  axisX, float  axisY, float  axisZ) GetAxisAcelStates()
    {
        return (
            (float)axisAX.ReadValue()* (float)0.122 / 1000,  // Retorna el valor actual del eje X
            (float)axisAY.ReadValue()* (float)0.122 / 1000,  // Retorna el valor actual del eje Y
            (float)axisAZ.ReadValue()* (float)0.122 / 1000  // Retorna el valor actual del eje Z
        );
        // return (
        //     (float)axisAX.ReadValue()* 4 / 32768,  // Retorna el valor actual del eje X
        //     (float)axisAY.ReadValue()* 4 / 32768,  // Retorna el valor actual del eje Y
        //     (float)axisAZ.ReadValue()* 4 / 32768  // Retorna el valor actual del eje Z
        // );
    }
    public (float  axisX, float  axisY, float  axisZ) GetAxisGyroStates()
    {
        return (
            (float)axisGX.ReadValue()* 70 / 1000,  // Retorna el valor actual del eje X
            (float)axisGY.ReadValue()* 70 / 1000,  // Retorna el valor actual del eje Y
            (float)axisGZ.ReadValue()* 70 / 1000  // Retorna el valor actual del eje Z
        );
        // return (
        //     (float)axisGX.ReadValue()* 2000 / 32768,  // Retorna el valor actual del eje X
        //     (float)axisGY.ReadValue()* 2000 / 32768,  // Retorna el valor actual del eje Y
        //     (float)axisGZ.ReadValue()* 2000 / 32768  // Retorna el valor actual del eje Z
        // );
    }
    public (float  axisX, float  axisY, float  axisZ) GetAxisMagStates()
    {
        return (
            (float)axisMX.ReadValue()* (float)0.14 * (float)0.1,  
            (float)axisMY.ReadValue()* (float)0.14 * (float)0.1, 
            (float)axisMZ.ReadValue()* (float)0.14 * (float)0.1  
        );
        // return (
        //     (float)axisMX.ReadValue()* 4 * 100 / 32768,  
        //     (float)axisMY.ReadValue()* 4 * 100 / 32768, 
        //     (float)axisMZ.ReadValue()* 4 * 100 / 32768  
        // );
    }

    public uint GetDt()
    {
        return 1;  // Retorna true si el botón está presionado, false si no lo está.
    }
    public uint GetID()
    {
        return (uint)id.ReadValue();  // Retorna true si el botón está presionado, false si no lo está.
    }
    public uint GetButoons()
    {
        return 1;  // Retorna true si el botón está presionado, false si no lo está.
    }
    public uint GetEnableData()
    {
        return (uint)enableData.ReadValue();  // Retorna true si el botón está presionado, false si no lo está.
    }

    public uint GetTimeStamp()
    {
        return  1; 
    }

    static ENDOSCOPY_CONTROLLER_Device()
    {
        InputSystem.RegisterLayout<ENDOSCOPY_CONTROLLER_Device>(
            matches: new InputDeviceMatcher()
                .WithInterface("HID") 
                .WithCapability("vendorId", 0)
                .WithCapability("productId", 0)
        );
    }

    [RuntimeInitializeOnLoadMethod]
    static void Init() { }

}


