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

            var movementSpeed = 0f;
            if (Input.GetKey(KeyCode.A))
            {
                movementSpeed = -3f;
            }
            else if (Input.GetKey(KeyCode.D))
            {
                movementSpeed = 3f;
            }

            var decay = _controller.State.IsGrounded ? 20f : 5f;
            _controller.SetHorizontalForce(Mathf.Lerp(_controller.Speed.x, movementSpeed, Time.deltaTime * decay));
            if (Input.GetKeyDown(KeyCode.Space) && JumpEnabled)
            {
                _jumpedAt = Time.time;
                JumpEnabled = false;
                if (_controller.State.OnPlatformController)
                {
                    StartCoroutine(_controller.IgnoreCollider(_controller.StandingOnCollider, 0.05f));
                    _controller.DetachFromPlatform();
                }

                _controller.SetVerticalForce(Mathf.Sqrt(2f * 2f * Mathf.Abs(_controller.Gravity)));
            }
        }

    }

}