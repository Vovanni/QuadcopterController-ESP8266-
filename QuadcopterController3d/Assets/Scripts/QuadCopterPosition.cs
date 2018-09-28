
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;
 
public class QuadCopterPosition : MonoBehaviour {

    public string YawAxisName = "Yaw";
    public string RollAxisName = "Roll";
    public string PitchAxisName = "Pitch";
    public int Angle = 60;

    private float OldYaw = 0;
    private float OldRoll = 0;
    private float OldPitch = 0;

    private float StartYaw = 0;
    private float StartRoll = 0;
    private float StartPitch = 0;

    private Quaternion StartPosition;

    private void Start()
    {
        StartPosition = transform.rotation;
    }

    void Update () {
        float NewYaw = CrossPlatformInputManager.GetAxis(YawAxisName);
        float NewRoll = CrossPlatformInputManager.GetAxis(RollAxisName);
        float NewPitch = CrossPlatformInputManager.GetAxis(PitchAxisName);

        if (NewRoll == 0 & NewPitch == 0 & NewYaw == 0)
        {
            transform.rotation = StartPosition;
        }

        else
        {
            transform.Rotate((NewPitch - OldPitch) * Angle, (NewYaw - OldYaw) * Angle, -(NewRoll - OldRoll) * Angle);
        }

        OldYaw = NewYaw;
        OldRoll = NewRoll;
        OldPitch = NewPitch;
    }
}
