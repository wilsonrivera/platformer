using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class MovingPlatform : PlatformBase
    {

        public Vector3 CurrentSpeed;

        public bool up = true;

        private void FixedUpdate()
        {
            var initialPosition = transform.position;
            transform.position += (up ? Vector3.up : Vector3.down) * (1f * Time.fixedDeltaTime);
            var finalPosition = transform.position;

            CurrentSpeed = (finalPosition - initialPosition) / Time.deltaTime;
        }

    }

}