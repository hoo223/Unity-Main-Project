//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    [Serializable]
    public class FollowJointTrajectoryFeedback : Message
    {
        public const string k_RosMessageName = "control_msgs/FollowJointTrajectory";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public string[] joint_names;
        public Trajectory.JointTrajectoryPointMsg desired;
        public Trajectory.JointTrajectoryPointMsg actual;
        public Trajectory.JointTrajectoryPointMsg error;
        public string[] multi_dof_joint_names;
        public Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_desired;
        public Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_actual;
        public Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_error;

        public FollowJointTrajectoryFeedback()
        {
            this.header = new Std.HeaderMsg();
            this.joint_names = new string[0];
            this.desired = new Trajectory.JointTrajectoryPointMsg();
            this.actual = new Trajectory.JointTrajectoryPointMsg();
            this.error = new Trajectory.JointTrajectoryPointMsg();
            this.multi_dof_joint_names = new string[0];
            this.multi_dof_desired = new Trajectory.MultiDOFJointTrajectoryPointMsg();
            this.multi_dof_actual = new Trajectory.MultiDOFJointTrajectoryPointMsg();
            this.multi_dof_error = new Trajectory.MultiDOFJointTrajectoryPointMsg();
        }

        public FollowJointTrajectoryFeedback(Std.HeaderMsg header, string[] joint_names, Trajectory.JointTrajectoryPointMsg desired, Trajectory.JointTrajectoryPointMsg actual, Trajectory.JointTrajectoryPointMsg error, string[] multi_dof_joint_names, Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_desired, Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_actual, Trajectory.MultiDOFJointTrajectoryPointMsg multi_dof_error)
        {
            this.header = header;
            this.joint_names = joint_names;
            this.desired = desired;
            this.actual = actual;
            this.error = error;
            this.multi_dof_joint_names = multi_dof_joint_names;
            this.multi_dof_desired = multi_dof_desired;
            this.multi_dof_actual = multi_dof_actual;
            this.multi_dof_error = multi_dof_error;
        }

        public static FollowJointTrajectoryFeedback Deserialize(MessageDeserializer deserializer) => new FollowJointTrajectoryFeedback(deserializer);

        private FollowJointTrajectoryFeedback(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.joint_names, deserializer.ReadLength());
            this.desired = Trajectory.JointTrajectoryPointMsg.Deserialize(deserializer);
            this.actual = Trajectory.JointTrajectoryPointMsg.Deserialize(deserializer);
            this.error = Trajectory.JointTrajectoryPointMsg.Deserialize(deserializer);
            deserializer.Read(out this.multi_dof_joint_names, deserializer.ReadLength());
            this.multi_dof_desired = Trajectory.MultiDOFJointTrajectoryPointMsg.Deserialize(deserializer);
            this.multi_dof_actual = Trajectory.MultiDOFJointTrajectoryPointMsg.Deserialize(deserializer);
            this.multi_dof_error = Trajectory.MultiDOFJointTrajectoryPointMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.joint_names);
            serializer.Write(this.joint_names);
            serializer.Write(this.desired);
            serializer.Write(this.actual);
            serializer.Write(this.error);
            serializer.WriteLength(this.multi_dof_joint_names);
            serializer.Write(this.multi_dof_joint_names);
            serializer.Write(this.multi_dof_desired);
            serializer.Write(this.multi_dof_actual);
            serializer.Write(this.multi_dof_error);
        }

        public override string ToString()
        {
            return "FollowJointTrajectoryFeedback: " +
            "\nheader: " + header.ToString() +
            "\njoint_names: " + System.String.Join(", ", joint_names.ToList()) +
            "\ndesired: " + desired.ToString() +
            "\nactual: " + actual.ToString() +
            "\nerror: " + error.ToString() +
            "\nmulti_dof_joint_names: " + System.String.Join(", ", multi_dof_joint_names.ToList()) +
            "\nmulti_dof_desired: " + multi_dof_desired.ToString() +
            "\nmulti_dof_actual: " + multi_dof_actual.ToString() +
            "\nmulti_dof_error: " + multi_dof_error.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Feedback);
        }
    }
}
