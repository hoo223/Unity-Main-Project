using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Control;
using System.Linq;

public class GripperActionSubsrciber : MonoBehaviour
{

    public static bool grasping_flag = false;

    // ROS Connector
    // private ROSConnection ros;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.01f;
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 0.05f;
    
    // State flag
    private bool GripperOpen = true;

    // Multipliers correspond to the URDF mimic tag for each joint
    private float[] multipliers = new float[] { 1f, 1f, -1f, 1f, 1f, -1f };

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
    private List<ArticulationBody> gripperJoints;

    // Hardcoded variables 
    private int numGripperJoints = 8;
    private int numLeftJoints = 4;
    private int numRightJoints = 4;
    public float gripperAngle = 15;

    public GameObject UR10;
    public GameObject leftPad;
    public GameObject rightPad;
    GripperCollisionChecker leftCollision;
    GripperCollisionChecker rightCollision;
    float time = 0;
    float pre_t = 0;

    List<float> targets = new List<float>();
    List<float> targetVelocities = new List<float>();

    // UI elements
    private Button GripperButton;

    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            curXDrive.stiffness = 20000.0f;
            curXDrive.lowerLimit = -45.8366f;
            curXDrive.upperLimit = 45.8366f;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }

    /// <summary>
    ///     Button callback for the Gripper Action
    /// </summary>
    public void GripperFunc(){
        //Debug.Log("Gripper Action...");
        GripperOpen = !GripperOpen;
        //Debug.Log(GripperOpen);
        StartCoroutine(GripperAction(GripperOpen));
    
    }

    private IEnumerator GripperAction(bool GripperOpen){
        if (!GripperOpen){
            StartCoroutine(IterateToGrip(true));
            yield return new WaitForSeconds(jointAssignmentWait);
        }
        else{
            yield return new WaitForSeconds(poseAssignmentWait);
            // Open the gripper to place the target cube
            StartCoroutine(IterateToGrip(false));
        }
    }

    void Awake()
    {
        var gripperJointNames = new string[] { "right_inner_knuckle", "right_outer_knuckle", "right_inner_finger", "left_inner_knuckle", "left_outer_knuckle", "left_inner_finger"};
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in UR10.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                //Debug.Log(articulationBody.name);
                gripperJoints.Add(articulationBody);
            }
        }
        leftCollision = leftPad.GetComponent<GripperCollisionChecker>();
        rightCollision = rightPad.GetComponent<GripperCollisionChecker>();
    }

    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update () {
 
        time += Time.deltaTime;

        // Pad collision check
        if(rightCollision.isCollision){
            Debug.Log("Right Pad Collision");
        }
        if(leftCollision.isCollision){
            Debug.Log("Left Pad Collision");
        }
        if(grasping_flag == true){
            GripperFunc();
        }
        GripperActionSubsrciber.grasping_flag = false;

    }


}
