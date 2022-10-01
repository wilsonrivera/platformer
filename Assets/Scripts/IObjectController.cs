using UnityEngine;

namespace DefaultNamespace
{

    public interface IObjectController
    {

        Vector2 Position { get; }

        Vector2 ForcesApplied { get; }

        Vector2 Velocity { get; }

        /// <summary>
        /// Moves the controller's transform to the desired position.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="findSafePosition"></param>
        void SetTransformPosition(Vector2 position, bool findSafePosition = true);

        /// <summary>
        /// Teleports the controller to the ground.
        /// </summary>
        void WarpToGround();

        /// <summary>
        /// Use this to add force to the controller.
        /// </summary>
        /// <param name="value">The force to add to the controller.</param>
        void AddForce(Vector2 value);

        /// <summary>
        /// Use this to add horizontal force to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to add to the controller.</param>
        void AddHorizontalForce(float value);

        /// <summary>
        /// Use this to add vertical force to the controller.
        /// </summary>
        /// <param name="value">The vertical force to add to the controller.</param>
        void AddVerticalForce(float value);

        /// <summary>
        /// Use this to set the force applied to the controller.
        /// </summary>
        /// <param name="value">The force to apply to the controller.</param>
        void SetForce(Vector2 value);

        /// <summary>
        /// Use this to set the horizontal force applied to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to apply to the controller.</param>
        void SetHorizontalForce(float value);

        /// <summary>
        /// Use this to set the vertical force applied to the controller.
        /// </summary>
        /// <param name="value">The vertical force to apply to the controller.</param>
        void SetVerticalForce(float value);

    }

}