//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Omni
{
    [Serializable]
    public class OmniFeedbackMsg : Message
    {
        public const string k_RosMessageName = "omni_msgs/OmniFeedback";
        public override string RosMessageName => k_RosMessageName;

        //  This is the force as estimated from the applied torques as well as the
        //  current end effector position of the robot arm
        public Geometry.Vector3Msg force;
        public Geometry.Vector3Msg position;

        public OmniFeedbackMsg()
        {
            this.force = new Geometry.Vector3Msg();
            this.position = new Geometry.Vector3Msg();
        }

        public OmniFeedbackMsg(Geometry.Vector3Msg force, Geometry.Vector3Msg position)
        {
            this.force = force;
            this.position = position;
        }

        public static OmniFeedbackMsg Deserialize(MessageDeserializer deserializer) => new OmniFeedbackMsg(deserializer);

        private OmniFeedbackMsg(MessageDeserializer deserializer)
        {
            this.force = Geometry.Vector3Msg.Deserialize(deserializer);
            this.position = Geometry.Vector3Msg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.force);
            serializer.Write(this.position);
        }

        public override string ToString()
        {
            return "OmniFeedbackMsg: " +
            "\nforce: " + force.ToString() +
            "\nposition: " + position.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
