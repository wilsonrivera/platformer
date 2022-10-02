using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class MovingPlatform : PlatformBase
    {

        public Vector2 CurrentSpeed;
        private ObjectController _controller;

        public bool up = true;

        private void FixedUpdate()
        {
            Vector2 initialPosition = transform.position;
            var finalPosition = initialPosition + (up ? Vector2.up : Vector2.down) * (1f * Time.fixedDeltaTime);

            transform.position = finalPosition;
            Physics2D.SyncTransforms();

            CurrentSpeed = (finalPosition - initialPosition) / Time.fixedDeltaTime;
        }

    }

}