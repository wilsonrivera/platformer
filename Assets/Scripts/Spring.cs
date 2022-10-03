using System;
using UnityEngine;

namespace DefaultNamespace
{

    public class Spring : MonoBehaviour
    {

        private void OnCollisionEnter2D(Collision2D col)
        {
            if (!col.gameObject.TryGetComponent<ObjectController>(out var controller)) return;
            if (col.contacts[0].normal == (Vector2)transform.up)
            {
                return;
            }

            controller.SetHorizontalForce(-10f);
        }

    }

}