using System;
using System.Collections;
using UnityEngine;

namespace DefaultNamespace
{

    public class Move : MonoBehaviour
    {

        private ObjectController _controller;
        private bool _jumpEnabled;
        private float _jumpedAt;

        public bool JumpEnabled
        {
            get { return _jumpEnabled; }
            set
            {
                _jumpEnabled = value;
                if (value)
                {
                    _jumpedAt = Time.time;
                }
            }
        }

        private void Awake()
        {
            _controller = GetComponent<ObjectController>();
        }

        private void Update()
        {
            if (_controller.State.IsGrounded && Time.time - _jumpedAt > 0.05f)
            {
                JumpEnabled = true;
            }

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

            if (Input.GetKeyDown(KeyCode.Space) && JumpEnabled)
            {
                _jumpedAt = Time.time;
                JumpEnabled = false;
                _controller.SetVerticalForce(Mathf.Sqrt(2f * 2f * Mathf.Abs(_controller.Gravity)));
            }
        }

    }

}