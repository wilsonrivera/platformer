using System;
using System.Collections;
using UnityEngine;

namespace DefaultNamespace
{

    public class Move : MonoBehaviour
    {

        private ObjectController _controller;

        private void Awake()
        {
            _controller = GetComponent<ObjectController>();
        }

        private void Update()
        {
            if (Input.GetKey(KeyCode.A))
            {
                _controller.SetHorizontalVelocity(-3f);
            }
            else if (Input.GetKey(KeyCode.D))
            {
                _controller.SetHorizontalVelocity(3f);
            }
            else
            {
                _controller.SetHorizontalVelocity(0f);
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                _controller.SetVerticalForce(5);
            }
        }

    }

}