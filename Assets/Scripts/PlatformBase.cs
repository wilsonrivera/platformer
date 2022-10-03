using UnityEngine;

namespace DefaultNamespace
{

    [RequireComponent(typeof(Collider2D))]
    public abstract class PlatformBase : MonoBehaviour
    {

        public virtual void OnControllerEnter(ObjectController controller)
        {
        }

        public virtual void OnControllerStay(ObjectController controller)
        {
        }

        public virtual void OnControllerExit(ObjectController controller)
        {
        }

    }

}