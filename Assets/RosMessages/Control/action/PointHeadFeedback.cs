//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    [Serializable]
    public class PointHeadFeedback : Message
    {
        public const string k_RosMessageName = "control_msgs/PointHead";
        public override string RosMessageName => k_RosMessageName;

        public double pointing_angle_error;

        public PointHeadFeedback()
        {
            this.pointing_angle_error = 0.0;
        }

        public PointHeadFeedback(double pointing_angle_error)
        {
            this.pointing_angle_error = pointing_angle_error;
        }

        public static PointHeadFeedback Deserialize(MessageDeserializer deserializer) => new PointHeadFeedback(deserializer);

        private PointHeadFeedback(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.pointing_angle_error);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.pointing_angle_error);
        }

        public override string ToString()
        {
            return "PointHeadFeedback: " +
            "\npointing_angle_error: " + pointing_angle_error.ToString();
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
