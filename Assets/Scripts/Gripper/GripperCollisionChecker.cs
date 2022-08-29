using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperCollisionChecker : MonoBehaviour
{
    public bool isCollision = false;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnCollisionEnter (Collision collision)
    {
        Debug.Log("on collision enter");
        isCollision = true;
        if (collision.collider.gameObject.CompareTag("object"))
        {
            Debug.Log("object");
        }
        // else if (collision.collider.gameObject.CompareTag("left_gripper"))
        // {
        //     Debug.Log("left_gripper");
        // }
        // else if (collision.collider.gameObject.CompareTag("right_gripper"))
        // {
        //     Debug.Log("right_gripper");
        // }
    }

}
