//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Omni
{
    [Serializable]
    public class OmniButtonEventMsg : Message
    {
        public const string k_RosMessageName = "omni_msgs/OmniButtonEvent";
        public override string RosMessageName => k_RosMessageName;

        public int grey_button;
        public int white_button;

        public OmniButtonEventMsg()
        {
            this.grey_button = 0;
            this.white_button = 0;
        }

        public OmniButtonEventMsg(int grey_button, int white_button)
        {
            this.grey_button = grey_button;
            this.white_button = white_button;
        }

        public static OmniButtonEventMsg Deserialize(MessageDeserializer deserializer) => new OmniButtonEventMsg(deserializer);

        private OmniButtonEventMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.grey_button);
            deserializer.Read(out this.white_button);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.grey_button);
            serializer.Write(this.white_button);
        }

        public override string ToString()
        {
            return "OmniButtonEventMsg: " +
            "\ngrey_button: " + grey_button.ToString() +
            "\nwhite_button: " + white_button.ToString();
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
