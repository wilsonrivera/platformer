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

    }

}