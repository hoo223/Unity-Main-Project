using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // JointStateMsg
using RosMessageTypes.Std; // Float64MultiArrayMsg
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


public class UR10ROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed = 0;

    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 1/125.0f;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private List<ArticulationBody> gripperJoints;

    // Hardcoded variables 
    private int numRobotJoints = 6;
    private int numForceTorques = 6;
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float gripperAngle = 45f;

    // Multipliers correspond to the URDF mimic tag for each joint
    private float[] multipliers = new float[] { -1f, -1f, -1f, 1f, 1f, 1f };

    // Variables required for ROS communication 
    public string topicJointState = "/unity_ur10_joint_states";

    public GameObject UR10;
    float pre_t = 0;
    
    List<float> positions = new List<float>();
    List<float> velocities = new List<float>();
    List<float> targets = new List<float>();
    List<float> targetVelocities = new List<float>();
    double[] command;

    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }


    void Awake()
    {
        
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = UR10.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = UR10.transform.Find(arm_link).GetComponent<ArticulationBody>();

        string elbow_link = arm_link + "/forearm_link";
        jointArticulationBodies[2] = UR10.transform.Find(elbow_link).GetComponent<ArticulationBody>();

        string wrist_1_link = elbow_link + "/wrist_1_link";
        jointArticulationBodies[3] = UR10.transform.Find(wrist_1_link).GetComponent<ArticulationBody>();

        string wrist_2_link = wrist_1_link + "/wrist_2_link";
        jointArticulationBodies[4] = UR10.transform.Find(wrist_2_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_2_link + "/wrist_3_link";
        jointArticulationBodies[5] = UR10.transform.Find(hand_link).GetComponent<ArticulationBody>();

        var gripperJointNames = new string[] { "robotiq_85_right_knuckle_link", "robotiq_85_right_finger_tip_link", "robotiq_85_right_inner_knuckle_link", "robotiq_85_left_knuckle_link", "robotiq_85_left_finger_tip_link", "robotiq_85_left_inner_knuckle_link" };
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in UR10.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                gripperJoints.Add(articulationBody);
            }
        }

    }

    
    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Publisher
        ros.RegisterPublisher<JointStateMsg>(topicJointState);

        // Subscriber
        //ROSConnection.instance.Subscribe<Float64MultiArrayMsg>("position_command", PositionCommand);
        ROSConnection.instance.Subscribe<Float64MultiArrayMsg>("velocity_command", VelocityCommand);

        command = new double[numRobotJoints];
    }

    void PositionCommand(Float64MultiArrayMsg actionMessage)
    {
        // Set the joint values for every joint
        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
        {
            var joint1XDrive = jointArticulationBodies[joint].xDrive;
            joint1XDrive.target = (float)actionMessage.data[joint] * Mathf.Rad2Deg;
            jointArticulationBodies[joint].xDrive = joint1XDrive;
        }
    }

    void VelocityCommand(Float64MultiArrayMsg actionMessage)
    {
        command = actionMessage.data;


        float cur_t = timeElapsed;
        float dt = cur_t - pre_t;
        //Debug.Log(dt);
        pre_t = cur_t;

        // Get current state
        jointArticulationBodies[0].GetJointPositions(targets);
        jointArticulationBodies[0].GetJointVelocities(targetVelocities);
        //jointArticulationBodies[0].GetJointForces(targetForces);
        //Debug.Log(targets.Count);


        // method 1
        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
        {
            // targetVelocities[joint] = ((float)command[joint]); // * Mathf.Rad2Deg ;
            // targets[joint] += ((float)command[joint]) * dt; 
            
            var joint1XDrive = jointArticulationBodies[joint].xDrive; 
            //joint1XDrive.stiffness = 0;
            joint1XDrive.targetVelocity = ((float)command[joint]) * Mathf.Rad2Deg; 
            joint1XDrive.target += ((float)command[joint]) * Mathf.Rad2Deg * dt; 
            jointArticulationBodies[joint].xDrive = joint1XDrive;    
        }  

        // method 2
        // jointArticulationBodies[0].SetDriveTargets(targets);
        // jointArticulationBodies[0].SetDriveTargetVelocities(targetVelocities);

        //Debug.Log(targetVelocities.Count);
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        timeElapsed += Time.deltaTime;
        // Debug.Log(timeElapsed);

        // UR10 State
        JointStateMsg joint_states = new JointStateMsg();
        joint_states.position = new double[numRobotJoints];
        jointArticulationBodies[0].GetJointPositions(positions);
        jointArticulationBodies[0].GetJointVelocities(velocities);
        
        joint_states.position[0] = (double)positions[0];
        joint_states.position[1] = (double)positions[1];
        joint_states.position[2] = (double)positions[2];
        joint_states.position[3] = (double)positions[3];
        joint_states.position[4] = (double)positions[4];
        joint_states.position[5] = (double)positions[5];

        joint_states.velocity = new double[numRobotJoints];
        joint_states.velocity[0] = (double)velocities[0];
        joint_states.velocity[1] = (double)velocities[1];
        joint_states.velocity[2] = (double)velocities[2];
        joint_states.velocity[3] = (double)velocities[3];
        joint_states.velocity[4] = (double)velocities[4];
        joint_states.velocity[5] = (double)velocities[5];


        // Finally send the message to server_endpoint.py running in ROS
        ros.Send(topicJointState, joint_states);

    }

}
