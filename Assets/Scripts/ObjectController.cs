using System;
using UnityEngine;

namespace DefaultNamespace
{

    [RequireComponent(typeof(BoxCollider2D))]
    public class ObjectController : MonoBehaviour
    {

        private const float kSmallFloatValue = 0.0001f;
        private const float kSkinWidthFloatFudgeFactor = 0.001f;

        [SerializeField] private float gravityScale = 1f;
        [SerializeField] private float groundFriction = 30f;
        [SerializeField] private float airFriction = 15f;

        [Header("Collisions")]
        [SerializeField] private LayerMask collisionMask;
        [SerializeField] private LayerMask oneWayPlatformMask;

        [Header("Raycasting")]
        [Tooltip("A small value added to all raycasts to accomodate for edge cases")]
        [SerializeField] [Range(0.001f, 0.1f)] private float skinWidth = 0.015f;
        [Tooltip("The number of rays cast horizontally")]
        [SerializeField] [Range(3, 20)] private int numberOfHorizontalRays = 8;
        [Tooltip("The number of rays cast vertically")]
        [SerializeField] [Range(3, 20)] private int numberOfVerticalRays = 8;

        // Components
        protected Transform _transform;
        protected BoxCollider2D _boxCollider;
        // State tracking
        protected Vector2 _speed;
        protected Vector2 _externalForce;
        protected float _currentGravity;
        protected float _slowFallFactor;
        protected Vector2 _previousPosition;
        protected Vector2 _deltaMovement;
        protected ObjectControllerState _state;

        protected RaycastOriginInfo _raycastOrigins;
        protected Vector2 _originalColliderSize;
        protected Vector2 _originalColliderOffset;
        protected bool _colliderResized;
        // Internal state tracking
        private float _verticalDistanceBetweenRays;
        private float _horizontalDistanceBetweenRays;

        public Vector2 Position { get; private set; }

        public Vector2 ForcesApplied { get; private set; }

        public Vector2 Velocity { get; protected set; }

        public ObjectControllerState State => _state;

        /// <summary>
        /// Gets the time (in seconds) since the last time the character was grounded
        /// </summary>
        public float TimeAirborne { get; protected set; }

        /// <summary>
        /// Is this controller affected by gravity?
        /// </summary>
        public bool IsGravityActive { get; protected set; } = true;

        public float GravityScale { get; protected set; }

        public bool IgnoreFriction { get; protected set; }

        public float GroundFriction { get; protected set; }

        public float AirFriction { get; protected set; }

        /// <summary>
        /// The object the controller is standing on.
        /// </summary>
        public GameObject StandingOn { get; protected set; }

        /// <summary>
        /// The object the controller was standing on last frame.
        /// </summary>
        public GameObject StandingOnLastFrame { get; protected set; }

        /// <summary>
        /// The collider the controller is standing on.
        /// </summary>
        public Collider2D StandingOnCollider { get; protected set; }

        public GameObject CurrentWall { get; protected set; }

        protected virtual float MinimumMovementThreshold => 0.01f;

        protected virtual void Awake()
        {
            _transform = transform;
            _boxCollider = GetComponent<BoxCollider2D>();

            _originalColliderSize = _boxCollider.size;
            _originalColliderOffset = _boxCollider.offset;

            GravityScale = gravityScale;
            GroundFriction = groundFriction;
            AirFriction = airFriction;

            _state.Reset();
            CalculateDistanceBetweenRays();
            UpdateRaycastOrigins();
        }

        protected virtual void Start()
        {
            _previousPosition = _transform.position;
            Position = _previousPosition;
            GravityScale = gravityScale;
            GroundFriction = groundFriction;
            AirFriction = airFriction;
        }

        protected virtual void FixedUpdate()
        {
            if (Time.timeScale == 0f)
            {
                return;
            }

            UpdateGravity();
            UpdateExternalForce();
            _state.Reset();

            ForcesApplied = _speed + _externalForce;
            _deltaMovement = ForcesApplied * Time.deltaTime;

            UpdateRaycastOrigins();
            if (_deltaMovement.x != 0f) HandleHorizontalCollisions();
            if (_deltaMovement.y != 0f) HandleVerticalCollisions();
            MoveTransform();
            UpdateRaycastOrigins();
            UpdateState();

            Velocity = _deltaMovement / Time.deltaTime;
            if (Velocity.magnitude < MinimumMovementThreshold)
            {
                Velocity = Vector2.zero;
            }
        }

        protected virtual void LateUpdate()
        {
            TimeAirborne = _state.IsGrounded ? 0f : TimeAirborne + Time.deltaTime;
        }

        protected virtual void OnDrawGizmosSelected()
        {
            var col = _boxCollider;
            if (!col && !TryGetComponent(out col)) return;

            var bounds = col.bounds;
            bounds.Expand(-2f * skinWidth);

            var origins = new RaycastOriginInfo
            {
                bottomLeft = bounds.min,
                bottomRight = new Vector2(bounds.max.x, bounds.min.y),
                topLeft = new Vector2(bounds.min.x, bounds.max.y),
                topRight = bounds.max
            };

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(origins.bottomLeft, origins.bottomRight);
            Gizmos.DrawLine(origins.topLeft, origins.topRight);
            Gizmos.DrawLine(origins.bottomLeft, origins.topLeft);
            Gizmos.DrawLine(origins.bottomRight, origins.topRight);
        }

        /// <summary>
        /// Use this to set the velocity applied to the controller.
        /// </summary>
        /// <param name="value">The velocity to apply to the controller.</param>
        public void SetVelocity(Vector2 value)
        {
            _speed = value;
        }

        /// <summary>
        /// Use this to set the horizontal velocity applied to the controller.
        /// </summary>
        /// <param name="value">The horizontal velocity to apply to the controller.</param>
        public void SetHorizontalVelocity(float value)
        {
            _speed.x = value;
        }

        /// <summary>
        /// Use this to set the vertical velocity applied to the controller.
        /// </summary>
        /// <param name="value">The vertical velocity to apply to the controller.</param>
        public void SetVerticalVelocity(float value)
        {
            _speed.y = value;
        }

        /// <summary>
        /// Use this to add force to the controller.
        /// </summary>
        /// <param name="value">The force to add to the controller.</param>
        public void AddForce(Vector2 value)
        {
            _externalForce += value;
        }

        /// <summary>
        /// Use this to add horizontal force to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to add to the controller.</param>
        public void AddHorizontalForce(float value)
        {
            _externalForce.x += value;
        }

        /// <summary>
        /// Use this to add vertical force to the controller.
        /// </summary>
        /// <param name="value">The vertical force to add to the controller.</param>
        public void AddVerticalForce(float value)
        {
            _externalForce.y += value;
        }

        /// <summary>
        /// Use this to set the force applied to the controller.
        /// </summary>
        /// <param name="value">The force to apply to the controller.</param>
        public void SetForce(Vector2 value)
        {
            _externalForce = value;
            if (_speed.y < 0f)
            {
                // Reset the gravity
                _speed.y = 0f;
            }
        }

        /// <summary>
        /// Use this to set the horizontal force applied to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to apply to the controller.</param>
        public void SetHorizontalForce(float value)
        {
            _externalForce.x = value;
        }

        /// <summary>
        /// Use this to set the vertical force applied to the controller.
        /// </summary>
        /// <param name="value">The vertical force to apply to the controller.</param>
        public void SetVerticalForce(float value)
        {
            _externalForce.y = value;
            if (_speed.y < 0f)
            {
                // Reset the gravity
                _speed.y = 0f;
            }
        }

        /// <summary>
        /// Slows the character's fall by the specified factor.
        /// </summary>
        /// <param name="factor">The factor, must be between 0 and 1.</param>
        public virtual void SlowFall(float factor)
        {
            _slowFallFactor = factor == 0f ? 0f : Mathf.Clamp(factor, 0.01f, 1f);
        }

        /// <summary>
        /// Activates or deactivates the gravity for this controller only.
        /// </summary>
        /// <param name="isActive">If set to <c>true</c>, activates the gravity. If set to <c>false</c>, turns it off.</param>
        public virtual void SetGravityActive(bool isActive)
        {
            IsGravityActive = isActive;
        }

        public virtual void SetGravityScale(float value)
        {
            GravityScale = Mathf.Clamp(value, 0f, 25f);
        }

        public virtual void SetFrictionIgnored(bool ignore)
        {
            IgnoreFriction = ignore;
        }

        public virtual void SetGroundFriction(float value)
        {
            GroundFriction = value;
        }

        public virtual void SetAirFriction(float value)
        {
            AirFriction = value;
        }

        /// <summary>
        /// Moves the controller's transform to the desired position.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="moveToClosestPosition"></param>
        public void TeleportTo(Vector2 position, bool moveToClosestPosition = true)
        {
            Position = moveToClosestPosition ? GetClosestSafePosition(position) : position;
            Velocity = Vector2.zero;
            transform.position = Position;
            Physics2D.SyncTransforms();
        }

        /// <summary>
        /// Teleports the controller to the ground.
        /// </summary>
        public void WarpToGround()
        {
        }

        /// <summary>
        /// Resizes the collider to the provided <paramref name="newSize"/>.
        /// </summary>
        /// <param name="newSize">The new size for the collider.</param>
        public virtual void ResizeCollider(Vector2 newSize)
        {
            var newYOffset = _originalColliderOffset.y - (_originalColliderSize.y - newSize.y) / 2f;

            _boxCollider.size = newSize;
            _boxCollider.offset = newYOffset * Vector3.up;
            CalculateDistanceBetweenRays();
            UpdateRaycastOrigins();
            _colliderResized = true;
        }

        /// <summary>
        /// Returns the collider to its initial size.
        /// </summary>
        public virtual void ResetColliderSize()
        {
            _boxCollider.size = _originalColliderSize;
            _boxCollider.offset = _originalColliderOffset;
            CalculateDistanceBetweenRays();
            UpdateRaycastOrigins();
            _colliderResized = false;
        }

        /// <summary>
        /// Determines whether the box collider can return to the original size.
        /// </summary>
        /// <returns><c>true</c> if the box collider can go back to original size; otherwise, <c>false</c>.</returns>
        public virtual bool ColliderCanGoBackToOriginalSize()
        {
            if (_boxCollider.size == _originalColliderSize)
            {
                // The collider is already at original size
                return true;
            }

            UpdateRaycastOrigins();
            var up = _transform.up;
            var headCheckDistance = _originalColliderSize.y * transform.localScale.y;

            // Cast two rays above the controller to check for obstacles. If we didn't hit anything, we can go back
            // to original size, otherwise we can't
            var topLeft = _raycastOrigins.topLeft + (Vector2)up * kSmallFloatValue;
            bool headCheckLeft = Physics2D.Raycast(topLeft, up, headCheckDistance, collisionMask);

            var topRight = _raycastOrigins.topRight + (Vector2)up * kSmallFloatValue;
            bool headCheckRight = Physics2D.Raycast(topRight, up, headCheckDistance, collisionMask);
            return !headCheckLeft && !headCheckRight;
        }

        /// <summary>
        /// This should be called anytime you have to modify the <see cref="BoxCollider2D"/> at runtime. It
        /// will recalculate the distance between the rays used for collision detection.
        /// </summary>
        protected void CalculateDistanceBetweenRays()
        {
            var bounds = _boxCollider.bounds;
            bounds.Expand(skinWidth * -2f);

            var size = bounds.size;
            var localScale = _transform.localScale;

            // Horizontal
            var colliderUsableHeight = size.y * Mathf.Abs(localScale.y);
            _verticalDistanceBetweenRays = colliderUsableHeight / (numberOfHorizontalRays - 1);

            // Vertical
            var colliderUsableWidth = size.x * Mathf.Abs(localScale.x);
            _horizontalDistanceBetweenRays = colliderUsableWidth / (numberOfVerticalRays - 1);
        }

        // <summary>
        // Resets the <see cref="RaycastOrigins"/> to the current extents of the collider inset by the
        // <see cref="SkinWidth"/>. It is inset to avoid casting a ray from a position directly touching another
        // collider which results in wonky normal data.
        // </summary>
        protected void UpdateRaycastOrigins()
        {
            var bounds = _boxCollider.bounds;
            bounds.Expand(-2f * skinWidth);

            _raycastOrigins.bottomLeft = bounds.min;
            _raycastOrigins.bottomRight = new Vector2(bounds.max.x, bounds.min.y);
            _raycastOrigins.topLeft = new Vector2(bounds.min.x, bounds.max.y);
            _raycastOrigins.topRight = bounds.max;
        }

        /// <summary>
        /// Updates the character's vertical speed according to gravity, gravity scale and other properties.
        /// </summary>
        protected virtual void UpdateGravity()
        {
            if (!IsGravityActive || GravityScale == 0f)
            {
                // Either gravity is not active for this controller or the controller doesn't have any gravity scale
                return;
            }

            _currentGravity = Physics2D.gravity.y * GravityScale * Time.fixedDeltaTime;
            _speed.y += _currentGravity;
            if (_slowFallFactor != 0f)
            {
                _speed.y *= _slowFallFactor;
            }

            // if (speed.y > 0)
            // {
            //     speed.y += g;
            // }
            // else
            // {
            //     externalForce.y += g;
            // }
        }

        /// <summary>
        /// Reduces the external force over time according to the air or ground frictions.
        /// </summary>
        protected virtual void UpdateExternalForce()
        {
            if (IgnoreFriction)
            {
                return;
            }

            var friction = _state.IsGrounded ? GroundFriction : AirFriction;
            _externalForce = Vector2.MoveTowards(
                _externalForce,
                Vector2.zero, _externalForce.magnitude * friction * Time.fixedDeltaTime
            );

            if (_externalForce.magnitude <= MinimumMovementThreshold)
            {
                _externalForce = Vector2.zero;
            }
        }

        protected void MoveTransform()
        {
            if (_deltaMovement.magnitude < MinimumMovementThreshold) return;

            _previousPosition = _transform.position;
            Position = _previousPosition + _deltaMovement;

            Debug.DrawRay(_previousPosition, _deltaMovement * 3f, Color.green);
            _transform.Translate(_deltaMovement, Space.Self);
            Physics2D.SyncTransforms();
        }

        protected virtual void UpdateState()
        {
            IgnoreFriction = false;
        }

        /// <summary>
        ///  Returns the closest "safe" point (not overlapping any platform) to the destination.
        /// </summary>
        /// <param name="destination"></param>
        /// <returns></returns>
        protected virtual Vector2 GetClosestSafePosition(Vector2 destination)
        {
            var layerMask = collisionMask | oneWayPlatformMask;
            var hit = Physics2D.OverlapBox(destination, _boxCollider.size, 0f, layerMask);
            if (!hit)
            {
                return destination;
            }

            // If the original destination wasn't safe, we find the closest safe point between
            // our controller and the obstacle
            destination -= 0.05f * (Vector2)(hit.transform.position - (Vector3)Position).normalized;
            hit = Physics2D.OverlapBox(destination, _boxCollider.size, 0, layerMask);
            return !hit ? destination : Position;
        }

        /// <summary>
        /// Checks for collisions in the horizontal axis and adjust the speed accordingly to stop at the
        /// collided object.
        /// </summary>
        protected void HandleHorizontalCollisions()
        {
            var isGoingRight = _deltaMovement.x > 0f;
            var rayDistance = Mathf.Abs(_deltaMovement.x) + skinWidth;
            var rayDirection = isGoingRight ? Vector2.right : Vector2.left;
            var initialRayOrigin = isGoingRight ? _raycastOrigins.bottomRight : _raycastOrigins.bottomLeft;

            CurrentWall = null;
            for (var i = 0; i < numberOfHorizontalRays; i++)
            {
                var rayOrigin = initialRayOrigin;
                rayOrigin += Vector2.up * (_horizontalDistanceBetweenRays * i);

                Debug.DrawRay(rayOrigin, rayDirection * rayDistance, Color.red);
                var raycastHit = Physics2D.Raycast(rayOrigin, rayDirection, rayDistance, collisionMask);
                if (!raycastHit)
                {
                    continue;
                }

                var minDistance = Mathf.Min(Mathf.Abs(_deltaMovement.x), raycastHit.distance - skinWidth);
                _deltaMovement.x = minDistance * rayDirection.x;
                rayDistance = Mathf.Min(Mathf.Abs(_deltaMovement.x) + skinWidth, raycastHit.distance);

                _state.IsCollidingLeft = !isGoingRight;
                _state.IsCollidingRight = isGoingRight;
                CurrentWall = raycastHit.transform.gameObject;

                _externalForce.x = 0f;
                if (Mathf.Abs(_deltaMovement.x) < kSmallFloatValue)
                {
                    _deltaMovement.x = 0f;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                    break;
            }
        }

        /// <summary>
        /// Checks for collisions in the vertical axis and adjust the speed accordingly to stop at the
        /// collided object.
        /// </summary>
        protected void HandleVerticalCollisions()
        {
            var isGoingUp = _deltaMovement.y > 0f;
            var rayDistance = Mathf.Abs(_deltaMovement.y) + skinWidth;
            var rayDirection = isGoingUp ? Vector2.up : Vector2.down;
            var initialRayOrigin = isGoingUp ? _raycastOrigins.topLeft : _raycastOrigins.bottomLeft;

            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = initialRayOrigin;
                rayOrigin += Vector2.right * (_verticalDistanceBetweenRays * i + _deltaMovement.x);

                Debug.DrawRay(rayOrigin, rayDirection * rayDistance, Color.red);
                var raycastHit = Physics2D.Raycast(rayOrigin, rayDirection, rayDistance, collisionMask);
                if (!isGoingUp && !raycastHit)
                {
                    raycastHit = Physics2D.Raycast(rayOrigin, rayDirection, rayDistance, oneWayPlatformMask);
                }

                if (!raycastHit)
                {
                    continue;
                }

                _deltaMovement.y = (raycastHit.distance - skinWidth) * rayDirection.y;
                rayDistance = raycastHit.distance;

                _state.IsCollidingAbove = isGoingUp;
                _state.IsCollidingBelow = !isGoingUp;
                _state.IsFalling = !isGoingUp && !_state.IsCollidingBelow;
                if (!isGoingUp)
                {
                    StandingOn = raycastHit.transform.gameObject;
                    StandingOnCollider = raycastHit.collider;
                }

                _speed.y = 0f;
                _externalForce.y = 0f;
                if (Mathf.Abs(_deltaMovement.y) < kSmallFloatValue)
                {
                    _deltaMovement.y = 0f;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                {
                    break;
                }
            }
        }

        [Serializable] protected struct RaycastOriginInfo
        {

            public Vector2 topLeft;
            public Vector2 bottomLeft;
            public Vector2 topRight;
            public Vector2 bottomRight;

        }

        [Serializable] public struct ObjectControllerState
        {

            /// <summary>
            /// Is the controller colliding with something below it?
            /// </summary>
            public bool IsCollidingBelow { get; set; }

            /// <summary>
            /// Is the controller colliding with something above it?
            /// </summary>
            public bool IsCollidingAbove { get; set; }

            /// <summary>
            /// Is the controller colliding with something to the left?
            /// </summary>
            public bool IsCollidingLeft { get; set; }

            /// <summary>
            /// Is the controller colliding with something to the right?
            /// </summary>
            public bool IsCollidingRight { get; set; }

            /// <summary>
            /// Is the controller colliding with anything?
            /// </summary>
            public bool HasCollision => IsCollidingBelow || IsCollidingAbove || IsCollidingLeft || IsCollidingRight;

            /// <summary>
            /// Is the controller grounded?
            /// </summary>
            public bool IsGrounded => IsCollidingBelow;

            /// <summary>
            /// Was the controller grounded last frame?
            /// </summary>
            public bool WasGroundedLastFrame { get; private set; }

            /// <summary>
            /// Did the controller just become grounded?
            /// </summary>
            public bool JustGotGrounded => IsGrounded && !WasGroundedLastFrame;

            /// <summary>
            /// Is the controller ceilinged?
            /// </summary>
            public bool IsCeilinged => IsCollidingAbove;

            /// <summary>
            /// Was the controller ceilinged last frame?
            /// </summary>
            public bool WasCeilingedLastFrame { get; private set; }

            /// <summary>
            /// Did the controller just become ceilinged?
            /// </summary>
            public bool JustGotCeilinged => IsCeilinged && !WasCeilingedLastFrame;

            /// <summary>
            /// Is the controller falling?
            /// </summary>
            public bool IsFalling { get; set; }

            /// <summary>
            /// Was the controller falling last frame?
            /// </summary>
            public bool WasFallingLastFrame { get; private set; }

            /// <summary>
            /// Did the controller just started falling?
            /// </summary>
            public bool JustStartedFalling => IsFalling && !WasFallingLastFrame;

            public void Reset()
            {
                WasGroundedLastFrame = IsGrounded;
                WasCeilingedLastFrame = IsCeilinged;
                WasFallingLastFrame = IsFalling;
                IsFalling = true;
                IsCollidingBelow = false;
                IsCollidingAbove = false;
                IsCollidingLeft = false;
                IsCollidingRight = false;
            }

        }

    }

}