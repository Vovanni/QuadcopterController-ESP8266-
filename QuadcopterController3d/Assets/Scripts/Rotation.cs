using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

public class Rotation : MonoBehaviour {

    public string ThrottleAxisName = "Throttle";
    public int Speed = 1500;
	
	void Update () {
        transform.Rotate(Vector3.up * Time.deltaTime * Speed * (CrossPlatformInputManager.GetAxis(ThrottleAxisName)+1));
    }
}
