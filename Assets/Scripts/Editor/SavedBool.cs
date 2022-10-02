using UnityEditor;

namespace DefaultNamespace.Editor
{

    public sealed class SavedBool
    {

        private readonly string _name;
        private bool _isLoaded;
        private bool _value;

        public SavedBool(string name, bool value)
        {
            _name = name;
            _isLoaded = false;
            _value = value;
        }

        public bool value
        {
            get
            {
                Load();
                return _value;
            }
            set
            {
                Load();
                if (_value == value)
                    return;

                _value = value;
                EditorPrefs.SetBool(_name, value);
            }
        }

        private void Load()
        {
            if (_isLoaded)
                return;

            _isLoaded = true;
            _value = EditorPrefs.GetBool(_name, _value);
        }

        public static implicit operator bool(SavedBool s)
        {
            return s.value;
        }

    }

}