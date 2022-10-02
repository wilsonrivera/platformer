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

        private void OnEnable()
        {
            _showState = new SavedBool($"ObjectController.{target.GetInstanceID()}.{nameof(_showState)}", false);
        }

        /// <inheritdoc />
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            EditorGUILayout.Space();
            var controller = (ObjectController)target;

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
        }

    }

}