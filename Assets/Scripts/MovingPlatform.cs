using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class MovingPlatform : PlatformBase
    {

        private void FixedUpdate()
        {
            transform.position += Vector3.right * Time.fixedDeltaTime;
        }

    }

}