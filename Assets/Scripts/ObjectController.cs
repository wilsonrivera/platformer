using System;
using UnityEngine;

namespace DefaultNamespace
{

    [DefaultExecutionOrder(100)]
    [RequireComponent(typeof(BoxCollider2D))]
    public class ObjectController : MonoBehaviour
    {

        private const float kSmallFloatValue = 0.0001f;
        private const float kSkinWidthFloatFudgeFactor = 0.001f;
        private const float kMovementDirectionThreshold = 0.0001f;

        [SerializeField] private float gravityScale = 1f;

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
        protected ObjectControllerState _state;
        private ObjectControllerBounds _bounds;

        protected float _currentGravity;
        protected float _slowFallFactor;

        protected bool _shouldComputeSpeed;
        protected Vector2 _speed;
        protected Vector2 _externalForce;
        protected Vector2 _deltaMovement;
        protected MovementDirection _movementDirection = MovementDirection.Right;
        protected MovementDirection _storedMovementDirection;

        protected bool _colliderResized;
        protected Vector2 _originalColliderSize;
        protected Vector2 _originalColliderOffset;

        protected PlatformController _platformController;

        private RaycastOriginsInfo _raycastOrigins;
        private float _verticalDistanceBetweenRays;
        private float _horizontalDistanceBetweenRays;
        private readonly RaycastHit2D[] _sideHitsBuffer = new RaycastHit2D[4];
        private readonly RaycastHit2D[] _belowHitsBuffer = new RaycastHit2D[4];
        private readonly RaycastHit2D[] _aboveHitsBuffer = new RaycastHit2D[4];

        /// <summary>
        /// Gets the world space position of the controller.
        /// </summary>
        public Vector2 Position => _transform.position;

        public Vector2 Speed => _speed;

        public Vector2 ExternalForce => _externalForce;

        public Vector2 ForcesApplied { get; protected set; }

        public Vector2 WorldSpeed { get; protected set; }

        public ObjectControllerState State => _state;

        public ObjectControllerBounds Bounds => _bounds;

        public RaycastOriginsInfo RaycastOrigins => _raycastOrigins;

        /// <summary>
        /// Is this controller affected by gravity?
        /// </summary>
        public bool IsGravityActive { get; protected set; } = true;

        public float GravityScale { get; protected set; }

        /// <summary>
        /// Gets the gravity applied to the controller.
        /// </summary>
        public float Gravity
            => !IsGravityActive
                ? 0f
                : Physics2D.gravity.y * (Application.isPlaying ? GravityScale : gravityScale);

        /// <summary>
        /// Gets the time (in seconds) since the last time the character was grounded
        /// </summary>
        public float TimeAirborne { get; protected set; }

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

        protected virtual void Awake()
        {
            _transform = transform;
            _boxCollider = GetComponent<BoxCollider2D>();
            // gameObject.AddComponent<LatePhysicsSync>();

            if (TryGetComponent<Rigidbody2D>(out var rb))
            {
                rb.bodyType = RigidbodyType2D.Kinematic;
                rb.useFullKinematicContacts = true;
            }

            _bounds = new ObjectControllerBounds(_boxCollider);
            _originalColliderSize = _boxCollider.size;
            _originalColliderOffset = _boxCollider.offset;

            GravityScale = gravityScale;

            _state.Reset();
            CalculateDistanceBetweenRays();
            UpdateRaycastOrigins();
        }

        protected virtual void FixedUpdate()
        {
            if (Time.timeScale == 0f)
            {
                return;
            }

            UpdateGravity();
            OnFrameEnter();
            UpdateRaycastOrigins();
            HandlePlatform();

            ForcesApplied = _speed;

            DetermineMovementDirection();
            HandleCollisionsToTheSide();
            HandleCollisionsBelow();
            HandleCollisionsAbove();

            MoveTransform();

            UpdateRaycastOrigins();
            ComputeNewSpeed();

            _externalForce.x = 0;
            _externalForce.y = 0;

            OnFrameExit();

            WorldSpeed = _speed;
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
            var origins = RaycastOriginsInfo.FromBounds(bounds);

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(origins.BottomLeft, origins.BottomRight);
            Gizmos.DrawLine(origins.TopLeft, origins.TopRight);
            Gizmos.DrawLine(origins.BottomLeft, origins.TopLeft);
            Gizmos.DrawLine(origins.BottomRight, origins.TopRight);
        }

        /// <summary>
        /// Use this to add force to the controller.
        /// </summary>
        /// <param name="value">The force to add to the controller.</param>
        public virtual void AddForce(Vector2 value)
        {
            _speed += value;
            _externalForce += value;
        }

        /// <summary>
        /// Use this to add horizontal force to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to add to the controller.</param>
        public virtual void AddHorizontalForce(float value)
        {
            _speed.x += value;
            _externalForce.x += value;
        }

        /// <summary>
        /// Use this to add vertical force to the controller.
        /// </summary>
        /// <param name="value">The vertical force to add to the controller.</param>
        public virtual void AddVerticalForce(float value)
        {
            _speed.y += value;
            _externalForce.y += value;
        }

        /// <summary>
        /// Use this to set the force applied to the controller.
        /// </summary>
        /// <param name="value">The force to apply to the controller.</param>
        public virtual void SetForce(Vector2 value)
        {
            _speed = value;
            _externalForce = value;
        }

        /// <summary>
        /// Use this to set the horizontal force applied to the controller.
        /// </summary>
        /// <param name="value">The horizontal force to apply to the controller.</param>
        public virtual void SetHorizontalForce(float value)
        {
            _speed.x = value;
            _externalForce.x = value;
        }

        /// <summary>
        /// Use this to set the vertical force applied to the controller.
        /// </summary>
        /// <param name="value">The vertical force to apply to the controller.</param>
        public virtual void SetVerticalForce(float value)
        {
            _speed.y = value;
            _externalForce.y = value;
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

        /// <summary>
        /// Moves the controller's transform to the desired position.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="moveToClosestPosition"></param>
        public virtual void TeleportTo(Vector2 position, bool moveToClosestPosition = true)
        {
            var nextPosition = moveToClosestPosition ? GetClosestSafePosition(position) : position;
            if (Position == nextPosition)
            {
                return;
            }

            _transform.position = nextPosition;
            Physics2D.SyncTransforms();
            UpdateRaycastOrigins();
        }

        /// <summary>
        /// Teleports the controller to the ground.
        /// </summary>
        public virtual void WarpToGround()
        {
            UpdateRaycastOrigins();

            var smallestDistance = float.MaxValue;
            RaycastHit2D closestRaycastHit = default;
            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var raycastHit = Physics2D.Raycast(
                    _raycastOrigins.BottomLeft,
                    Vector2.down,
                    100f,
                    collisionMask | oneWayPlatformMask
                );

                if (!raycastHit || raycastHit.distance >= smallestDistance)
                {
                    continue;
                }

                smallestDistance = raycastHit.distance;
                closestRaycastHit = raycastHit;
            }

            if (!closestRaycastHit)
            {
                return;
            }

            var pos = _transform.position;
            pos.y -= closestRaycastHit.distance - skinWidth;
            TeleportTo(pos, false);

            _speed = Vector2.zero;
            _externalForce = Vector2.zero;
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
            var topLeft = _raycastOrigins.TopLeft + (Vector2)up * kSmallFloatValue;
            bool headCheckLeft = Physics2D.Raycast(topLeft, up, headCheckDistance, collisionMask);

            var topRight = _raycastOrigins.TopRight + (Vector2)up * kSmallFloatValue;
            bool headCheckRight = Physics2D.Raycast(topRight, up, headCheckDistance, collisionMask);
            return !headCheckLeft && !headCheckRight;
        }

        /// <summary>
        /// Disconnects the controller from the current platform controller.
        /// </summary>
        public virtual void DetachFromPlatform()
        {
            if (!_platformController)
            {
                return;
            }

            SetGravityActive(true);
            // State.OnAMovingPlatform=false;
            _platformController = null;
            // _movingPlatformCurrentGravity=0;
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

        /// <summary>
        /// Resets the raycast origins to the current extents of the collider inset by the
        /// <see cref="skinWidth"/>. It is inset to avoid casting a ray from a position directly touching another
        /// collider which results in wonky normal data.
        /// </summary>
        protected void UpdateRaycastOrigins()
        {
            var bounds = _boxCollider.bounds;
            bounds.Expand(-2f * skinWidth);

            _bounds.UpdateSize();
            _raycastOrigins = RaycastOriginsInfo.FromBounds(bounds);
        }

        protected virtual void OnFrameEnter()
        {
            _deltaMovement = _speed * Time.deltaTime;
            _shouldComputeSpeed = true;

            CurrentWall = null;
            StandingOnLastFrame = StandingOn;
            StandingOn = null;
            StandingOnCollider = null;

            _state.Reset();
        }

        protected virtual void OnFrameExit()
        {
        }

        /// <summary>
        /// Updates the character's vertical speed according to gravity, gravity scale and other properties.
        /// </summary>
        protected virtual void UpdateGravity()
        {
            _currentGravity = Gravity * Time.fixedDeltaTime;
            if (IsGravityActive)
            {
                // Only apply gravity when it is enabled for this controller
                _speed.y += _currentGravity;
            }

            if (_slowFallFactor != 0f)
            {
                _speed.y *= _slowFallFactor;
            }
        }

        protected virtual void ComputeNewSpeed()
        {
            var deltaTime = Time.deltaTime;
            if (!_shouldComputeSpeed || deltaTime == 0f) return;

            _speed = _deltaMovement / deltaTime;
            if (Mathf.Abs(_speed.x) < kSmallFloatValue) _speed.x = 0f;
            if (Mathf.Abs(_speed.y) < kSmallFloatValue) _speed.y = 0f;

            _shouldComputeSpeed = false;
        }

        protected virtual void MoveTransform()
        {
            if (_deltaMovement.magnitude < kSmallFloatValue)
            {
                return;
            }

            Debug.DrawRay(Position, _deltaMovement * 3f, Color.green);
            _transform.Translate(_deltaMovement, Space.Self);
        }

        /// <summary>
        /// Determines the current movement direction.
        /// </summary>
        protected virtual void DetermineMovementDirection()
        {
            _movementDirection = _storedMovementDirection;
            switch (_speed.x)
            {
                case < -kMovementDirectionThreshold:
                    _movementDirection = MovementDirection.Left;
                    break;
                case > kMovementDirectionThreshold:
                    _movementDirection = MovementDirection.Right;
                    break;
                default:
                {
                    _movementDirection = _externalForce.x switch
                    {
                        < -kMovementDirectionThreshold => MovementDirection.Left,
                        > kMovementDirectionThreshold  => MovementDirection.Right,
                        _                              => _movementDirection
                    };

                    break;
                }
            }

            if (_platformController)
            {
                if (Mathf.Abs(_platformController.CurrentSpeed.x) > Mathf.Abs(_speed.x))
                {
                    _movementDirection = Mathf.Sign(_platformController.CurrentSpeed.x) switch
                    {
                        < -kMovementDirectionThreshold => MovementDirection.Left,
                        > kMovementDirectionThreshold  => MovementDirection.Right,
                        _                              => _movementDirection
                    };
                }
            }

            _storedMovementDirection = _movementDirection;
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
        /// If the controller is standing on a platform controller, we match its speed.
        /// </summary>
        protected virtual void HandlePlatform()
        {
            if (!_platformController) return;
            {
                if (!float.IsNaN(_platformController.CurrentSpeed.x) &&
                    !float.IsNaN(_platformController.CurrentSpeed.y))
                {
                    _transform.Translate(_platformController.CurrentSpeed * Time.deltaTime);
                }

                if (Time.timeScale == 0 ||
                    float.IsNaN(_platformController.CurrentSpeed.x) ||
                    float.IsNaN(_platformController.CurrentSpeed.y) ||
                    Time.deltaTime <= 0f ||
                    _state.WasCeilingedLastFrame)
                {
                    return;
                }

                // State.OnAMovingPlatform = true;

                SetGravityActive(false);

                // _movingPlatformCurrentGravity = _movingPlatformsGravity;

                _deltaMovement.y = _platformController.CurrentSpeed.y * Time.deltaTime;
                _speed = -_deltaMovement / Time.deltaTime;
                _speed.x = -_speed.x;

                UpdateRaycastOrigins();
            }
        }

        protected virtual void HandleCollisionsToTheSide()
        {
            var isGoingRight = _movementDirection > 0f;
            var rayDirection = isGoingRight ? Vector2.right : Vector2.left;
            var movementDirection = isGoingRight ? MovementDirection.Right : MovementDirection.Left;
            var rayDistance = Mathf.Abs(_deltaMovement.x) + skinWidth;
            var initialRayOrigin = isGoingRight ? _raycastOrigins.BottomRight : _raycastOrigins.BottomLeft;
            var smallestHitDistance = float.MaxValue;
            RaycastHit2D closestRaycastHit = default;

            if (_deltaMovement.x == 0f)
            {
                // Compensate by multiplying the distance by 2 when we are not moving at all
                rayDistance *= 2f;
            }

            for (var i = 0; i < numberOfHorizontalRays; i++)
            {
                var rayOrigin = initialRayOrigin;
                rayOrigin += Vector2.up * (_horizontalDistanceBetweenRays * i);

                Debug.DrawRay(rayOrigin, rayDirection * rayDistance, Color.red);
                var raycastHitCount = Physics2D.RaycastNonAlloc(
                    rayOrigin,
                    rayDirection,
                    _sideHitsBuffer,
                    rayDistance,
                    collisionMask
                );

                if (raycastHitCount == 0) continue;
                for (var x = 0; x < raycastHitCount; x++)
                {
                    var raycastHit = _sideHitsBuffer[x];
                    if (raycastHit.distance >= smallestHitDistance)
                    {
                        continue;
                    }

                    rayDistance = Mathf.Min(Mathf.Abs(_deltaMovement.x) + skinWidth, raycastHit.distance);
                    smallestHitDistance = raycastHit.distance;
                    closestRaycastHit = raycastHit;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                {
                    break;
                }
            }

            if (!closestRaycastHit || _movementDirection == movementDirection)
            {
                return;
            }

            var minDistance = Mathf.Min(Mathf.Abs(_deltaMovement.x), closestRaycastHit.distance - skinWidth);
            _deltaMovement.x = minDistance * rayDirection.x;
            _shouldComputeSpeed = true;

            _state.IsCollidingLeft = !isGoingRight;
            _state.IsCollidingRight = isGoingRight;
            CurrentWall = closestRaycastHit.transform.gameObject;
            if (Mathf.Abs(_deltaMovement.x) < kSmallFloatValue)
            {
                _deltaMovement.x = 0f;
            }
        }

        protected virtual void HandleCollisionsBelow()
        {
            _state.IsFalling = _deltaMovement.y < -kSmallFloatValue;
            if (Physics2D.gravity.y > 0f && !_state.IsFalling)
            {
                _state.IsCollidingBelow = true;
                return;
            }

            var smallestHitDistance = float.MaxValue;
            RaycastHit2D closestRaycastHit = default;
            var rayDistance = Mathf.Abs(_deltaMovement.y) + skinWidth;
            if (_platformController) rayDistance *= 3f;

            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = _raycastOrigins.BottomLeft;
                rayOrigin += Vector2.right * (_verticalDistanceBetweenRays * i + _deltaMovement.x);

                Debug.DrawRay(rayOrigin, Vector2.down * rayDistance, Color.red);
                var raycastHitCount = Physics2D.RaycastNonAlloc(
                    rayOrigin,
                    Vector2.down,
                    _belowHitsBuffer,
                    rayDistance,
                    collisionMask | oneWayPlatformMask
                );

                if (raycastHitCount == 0) continue;
                for (var x = 0; x < raycastHitCount; x++)
                {
                    var raycastHit = _belowHitsBuffer[x];
                    if (raycastHit.distance >= smallestHitDistance)
                    {
                        continue;
                    }

                    rayDistance = raycastHit.distance;
                    smallestHitDistance = raycastHit.distance;
                    closestRaycastHit = raycastHit;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                {
                    break;
                }
            }

            if (!closestRaycastHit)
            {
                DetachFromPlatform();
                return;
            }

            _state.IsFalling = false;
            if (_externalForce.y > 0f && _speed.y > 0f)
            {
                // We are jumping, only apply that
                _state.IsCollidingBelow = false;
                _deltaMovement.y = _speed.y * Time.deltaTime;
            }
            else
            {
                // We have landed on something
                StandingOn = closestRaycastHit.transform.gameObject;
                StandingOnCollider = closestRaycastHit.collider;
                _state.IsCollidingBelow = true;

                _deltaMovement.y = -closestRaycastHit.distance + skinWidth;
            }

            if (!_state.WasGroundedLastFrame && _speed.y > 0f) _deltaMovement.y += _speed.y * Time.deltaTime;
            if (Mathf.Abs(_deltaMovement.y) < kSmallFloatValue) _deltaMovement.y = 0;

            //
            if (StandingOn && StandingOn.transform.TryGetComponent<PlatformController>(out var platform))
            {
                DetachFromPlatform();
                _platformController = platform;
            }
            else
            {
                DetachFromPlatform();
            }
        }

        protected virtual void HandleCollisionsAbove()
        {
            var rayDistance = _state.IsGrounded ? skinWidth : Mathf.Abs(_deltaMovement.y); // + skinWidth;
            var smallestHitDistance = float.MaxValue;
            RaycastHit2D closestRaycastHit = default;

            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = _raycastOrigins.TopLeft;
                rayOrigin += Vector2.right * (_verticalDistanceBetweenRays * i + _deltaMovement.x);

                Debug.DrawRay(rayOrigin, Vector2.up * rayDistance, Color.red);
                var raycastHitCount = Physics2D.RaycastNonAlloc(
                    rayOrigin,
                    Vector2.up,
                    _aboveHitsBuffer,
                    rayDistance,
                    collisionMask
                );

                if (raycastHitCount == 0) continue;
                for (var x = 0; x < raycastHitCount; x++)
                {
                    var raycastHit = _aboveHitsBuffer[x];
                    if (raycastHit.distance >= smallestHitDistance)
                    {
                        continue;
                    }

                    rayDistance = raycastHit.distance;
                    smallestHitDistance = rayDistance;
                    closestRaycastHit = raycastHit;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                {
                    break;
                }
            }

            if (!closestRaycastHit)
            {
                return;
            }

            _state.IsCollidingAbove = true;
            _deltaMovement.y = closestRaycastHit.distance - skinWidth;
            if (_state.IsGrounded && _deltaMovement.y < 0f) _deltaMovement.y = 0f;
            if (!_state.WasCeilingedLastFrame) _speed = new Vector2(_speed.x, 0f);

            SetVerticalForce(0f);
        }

        private sealed class LatePhysicsSync : MonoBehaviour
        {

            private void LateUpdate()
            {
                Physics2D.SyncTransforms();
            }

        }

        protected enum MovementDirection : byte
        {

            Left,
            Right

        }

        public struct ObjectControllerState
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

        public struct ObjectControllerBounds
        {

            private readonly BoxCollider2D _collider;

            public ObjectControllerBounds(BoxCollider2D collider)
            {
                _collider = collider;
                Width = 0f;
                Height = 0f;
            }

            public float Width { get; private set; }

            public float Height { get; private set; }

            public Vector2 Top
            {
                get
                {
                    var bounds = _collider.bounds;
                    return new Vector2(bounds.center.x, bounds.max.y);
                }
            }

            public Vector2 Bottom
            {
                get
                {
                    var bounds = _collider.bounds;
                    return new Vector2(bounds.center.x, bounds.min.y);
                }
            }

            public Vector2 Left
            {
                get
                {
                    var bounds = _collider.bounds;
                    return new Vector2(bounds.min.x, bounds.center.y);
                }
            }

            public Vector2 Right
            {
                get
                {
                    var bounds = _collider.bounds;
                    return new Vector2(bounds.max.x, bounds.center.y);
                }
            }

            public void UpdateSize()
            {
                var bounds = _collider.bounds;
                Width = Vector2.Distance(bounds.min, new Vector2(bounds.max.x, bounds.min.y));
                Height = Vector2.Distance(bounds.min, new Vector2(bounds.min.x, bounds.max.y));
            }

        }

        public struct RaycastOriginsInfo
        {

            private RaycastOriginsInfo(Vector2 topLeft, Vector2 bottomLeft, Vector2 topRight, Vector2 bottomRight)
            {
                TopLeft = topLeft;
                BottomLeft = bottomLeft;
                TopRight = topRight;
                BottomRight = bottomRight;
            }

            public Vector2 TopLeft { get; }

            public Vector2 BottomLeft { get; }

            public Vector2 TopRight { get; }

            public Vector2 BottomRight { get; }

            public static RaycastOriginsInfo FromBounds(Bounds bounds)
                => new(
                    new Vector2(bounds.min.x, bounds.max.y),
                    bounds.min,
                    bounds.max,
                    new Vector2(bounds.max.x, bounds.min.y)
                );

        }

    }

}