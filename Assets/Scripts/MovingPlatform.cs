using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class MovingPlatform : PlatformBase
    {

        public Vector3 CurrentSpeed;
        private ObjectController _controller;

        public bool up = true;

        private void FixedUpdate()
        {
            var initialPosition = transform.position;
            var finalPosition = initialPosition + (up ? Vector3.up : Vector3.down) * (1f * Time.fixedDeltaTime);

            transform.position = finalPosition;
            Physics2D.SyncTransforms();

            CurrentSpeed = (finalPosition - initialPosition) / Time.fixedDeltaTime;
        }

    }

}