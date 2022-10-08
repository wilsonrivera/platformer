using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class BouncePad : MonoBehaviour
    {

        [SerializeField] private float jumpBoost = 1f;

        private ObjectController _controller;

        private void OnCollisionEnter2D(Collision2D col)
        {
            if (!col.transform.TryGetComponent<ObjectController>(out var controller)) return;
            if (col.contacts[0].normal != -(Vector2)transform.up)
            {
                return;
            }

            _controller = controller;
        }

        private void LateUpdate()
        {
            if (!_controller) return;

            var boost = Mathf.Sqrt(2f * jumpBoost * -_controller.Gravity);
            _controller.SetVerticalForce(boost);
            if (_controller.transform.TryGetComponent<Move>(out var move))
            {
                move.JumpEnabled = false;
            }

            _controller = null;
        }

    }

}