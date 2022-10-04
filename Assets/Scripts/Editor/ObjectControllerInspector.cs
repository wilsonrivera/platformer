using System.Globalization;
using UnityEditor;
using UnityEngine;

namespace DefaultNamespace.Editor
{

    [CanEditMultipleObjects]
    [CustomEditor(typeof(ObjectController))]
    public class ObjectControllerInspector : UnityEditor.Editor
    {

        private SavedBool _showState;
        private readonly GUIContent sShowStateLabel = new("State");

        private SerializedProperty _skinWidthProp;
        private SerializedProperty _numberOfVerticalRaysProp;
        private SerializedProperty _collisionMaskProp;
        private SerializedProperty _oneWayPlatformMaskProp;

        private void OnEnable()
        {
            _showState = new SavedBool($"ObjectController.{target.GetInstanceID()}.{nameof(_showState)}", false);
            _skinWidthProp = serializedObject.FindProperty("skinWidth");
            _numberOfVerticalRaysProp = serializedObject.FindProperty("numberOfVerticalRays");
            _collisionMaskProp = serializedObject.FindProperty("collisionMask");
            _oneWayPlatformMaskProp = serializedObject.FindProperty("oneWayPlatformMask");
        }

        /// <inheritdoc />
        public override void OnInspectorGUI()
        {
            var controller = (ObjectController)target;

            EditorGUILayout.Space();
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.LabelField("Applied Gravity", controller.Gravity.ToString(CultureInfo.InvariantCulture));
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
            DrawDefaultInspector();

            _showState.value = EditorGUILayout.Foldout((bool)_showState, sShowStateLabel, true);
            if (_showState.value)
            {
                EditorGUI.BeginDisabledGroup(true);
                EditorGUILayout.LabelField("Grounded", controller.State.IsGrounded.ToString());
                EditorGUILayout.LabelField("Ceilinged", controller.State.IsCeilinged.ToString());
                EditorGUILayout.LabelField("Falling", controller.State.IsFalling.ToString());

                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Colliding Left", controller.State.IsCollidingLeft.ToString());
                EditorGUILayout.LabelField("Colliding Right", controller.State.IsCollidingRight.ToString());
                EditorGUILayout.LabelField("Colliding Above", controller.State.IsCollidingAbove.ToString());
                EditorGUILayout.LabelField("Colliding Below", controller.State.IsGrounded.ToString());

                EditorGUILayout.Space();
                EditorGUILayout.ObjectField("Standing On", controller.StandingOn, typeof(GameObject), true);
                EditorGUILayout.ObjectField("Current Wall", controller.CurrentWall, typeof(GameObject), true);

                EditorGUI.EndDisabledGroup();
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Move to ground", GUILayout.Height(24)))
            {
                MoveToGround(controller);
            }
        }

        private void MoveToGround(ObjectController controller)
        {
            if (Application.isPlaying)
            {
                // If the application is playing, instead of trying to calculate the distance ourself, we'll use the
                // `WarpToGround` method, which should accomplish the same function, except can only be used
                // at runtime
                controller.WarpToGround();
                return;
            }

            var collider = controller.gameObject.GetComponent<BoxCollider2D>();
            var skinWidth = _skinWidthProp.floatValue;
            var numberOfVerticalRays = _numberOfVerticalRaysProp.intValue;
            var collisionMask = _collisionMaskProp.intValue | _oneWayPlatformMaskProp.intValue;

            var bounds = collider.bounds;
            bounds.Expand(-2f * skinWidth);
            var bottomLeft = bounds.min;
            var bottomRight = new Vector2(bounds.max.x, bounds.min.y);

            var position = controller.transform.position;
            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = Vector2.Lerp(bottomLeft, bottomRight, i * (float)(numberOfVerticalRays - 1));
                var raycastHit = Physics2D.Raycast(rayOrigin, Vector2.down, 100f, collisionMask);
                if (!raycastHit) continue;

                position.y -= raycastHit.distance - skinWidth;
                if (controller.transform.position == position) break;
                Undo.RecordObject(controller.transform, "Move controller to ground");
                controller.transform.position = position;
                break;
            }
        }

    }

}