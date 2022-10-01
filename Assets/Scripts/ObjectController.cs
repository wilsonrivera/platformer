using System;
using UnityEngine;

namespace DefaultNamespace
{

    [RequireComponent(typeof(BoxCollider2D))]
    public class ObjectController : MonoBehaviour, IObjectController
    {

        private const float kSkinWidthFloatFudgeFactor = 0.001f;

        [SerializeField] private float gravityScale = 1f;
        [SerializeField] private float groundFriction = 30f;
        [SerializeField] private float airFriction = 15f;

        [Header("Collisions")]
        [SerializeField] private LayerMask collisionMask;
        [SerializeField] private LayerMask oneWayPlatformMask;

        [Header("Raycasting")]
        [Tooltip("A small value added to all raycasts to accomodate for edge cases")]
        [SerializeField] [Range(0.001f, 0.1f)] private float skinWidth = 0.05f;
        [Tooltip("The number of rays cast horizontally")]
        [SerializeField] [Range(3, 20)] private int numberOfHorizontalRays = 8;
        [Tooltip("The number of rays cast vertically")]
        [SerializeField] [Range(3, 20)] private int numberOfVerticalRays = 8;

        private Transform _transform;
        private BoxCollider2D _boxCollider;
        //
        protected Vector2 speed;
        protected Vector2 externalForce;
        //
        private Vector2 _previousPosition;
        private CollisionStateInfo _collisionState;
        private RaycastOriginInfo _raycastOrigins;
        private float _verticalDistanceBetweenRays;
        private float _horizontalDistanceBetweenRays;
        //
        private float? _customGravityScale;
        private float? _customGroundFriction;
        private float? _customAirFriction;

        /// <inheritdoc />
        public Vector2 Position { get; private set; }

        /// <inheritdoc />
        public Vector2 ForcesApplied { get; private set; }

        /// <inheritdoc />
        public Vector2 Velocity { get; private set; }

        protected virtual float MinimumMovementThreshold => 0.01f;

        protected float GravityScale => _customGravityScale ?? gravityScale;

        protected bool IgnoreFriction { get; private set; }

        protected float GroundFriction => _customGroundFriction ?? groundFriction;

        protected float AirFriction => _customAirFriction ?? airFriction;

        protected virtual void Awake()
        {
            _transform = transform;
            _boxCollider = GetComponent<BoxCollider2D>();
        }

        protected virtual void Start()
        {
            _previousPosition = _transform.position;
            Position = _previousPosition;

            CalculateDistanceBetweenRays();
            UpdateRaycastOrigins();
            CheckGrounded(1); // At the start we always check if the object is grounded facing right

            // SetTransformPosition(new Vector2(-10f, 0f));
            WarpToGround();
        }

        protected virtual void FixedUpdate()
        {
            if (Time.timeScale == 0f)
            {
                return;
            }

            _collisionState.Reset();
            UpdateGravity();
            UpdateExternalForce();

            ForcesApplied = speed + externalForce;
            var deltaMovement = ForcesApplied * Time.deltaTime;
            Velocity = MoveTransform(deltaMovement) / Time.deltaTime;
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

        /// <inheritdoc />
        public virtual void SetTransformPosition(Vector2 position, bool findSafePosition = true)
        {
            Position = findSafePosition ? GetClosestSafePosition(position) : position;
            Velocity = Vector2.zero;
            transform.position = Position;
            Physics2D.SyncTransforms();
        }

        /// <inheritdoc />
        public virtual void WarpToGround()
        {
        }

        /// <inheritdoc />
        public void AddForce(Vector2 value)
        {
            externalForce += value;
        }

        /// <inheritdoc />
        public void AddHorizontalForce(float value)
        {
            externalForce.x += value;
        }

        /// <inheritdoc />
        public void AddVerticalForce(float value)
        {
            externalForce.y += value;
        }

        /// <inheritdoc />
        public void SetForce(Vector2 value)
        {
            externalForce = value;
            if (speed.y < 0f)
            {
                // Reset the gravity
                speed.y = 0f;
            }
        }

        /// <inheritdoc />
        public void SetHorizontalForce(float value)
        {
            externalForce.x = value;
        }

        /// <inheritdoc />
        public void SetVerticalForce(float value)
        {
            externalForce.y = value;
            if (speed.y < 0f)
            {
                // Reset the gravity
                speed.y = 0f;
            }
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
            var g = Physics2D.gravity.y * GravityScale * Time.fixedDeltaTime;
            speed.y += g;
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

            var friction = _collisionState.isGrounded ? GroundFriction : AirFriction;
            externalForce = Vector2.MoveTowards(
                externalForce,
                Vector2.zero, externalForce.magnitude * friction * Time.fixedDeltaTime
            );

            if (externalForce.magnitude <= MinimumMovementThreshold)
            {
                externalForce = Vector2.zero;
            }
        }

        /// <summary>
        /// Tries to move according to current speed and checking for collisions.
        /// </summary>
        protected virtual Vector2 MoveTransform(Vector2 deltaMovement)
        {
            var go = gameObject;
            var goLayer = go.layer;
            go.layer = Physics2D.IgnoreRaycastLayer;

            UpdateRaycastOrigins();
            PreMoveTransform(ref deltaMovement);
            if (deltaMovement.x != 0f) HandleHorizontalCollisions(ref deltaMovement);
            if (deltaMovement.y != 0f) HandleVerticalCollisions(ref deltaMovement);

            if (deltaMovement != Vector2.zero)
            {
                _previousPosition = _transform.position;
                Position = _previousPosition + deltaMovement;

                Debug.DrawRay(_previousPosition, deltaMovement * 3f, Color.green);
                _transform.Translate(deltaMovement, Space.Self);
                Physics2D.SyncTransforms();
            }

            PostMoveTransform();
            go.layer = goLayer;
            return deltaMovement;
        }

        protected virtual void PreMoveTransform(ref Vector2 deltaMove)
        {
            CheckGrounded(Math.Sign(deltaMove.x));
        }

        protected virtual void PostMoveTransform()
        {
            IgnoreFriction = false;
            if (_collisionState.verticalHit)
            {
                if ((_collisionState.isCollidingBelow && ForcesApplied.y < 0f) ||
                    (_collisionState.isCollidingAbove && ForcesApplied.y > 0f))
                {
                    speed.y = 0f;
                    externalForce.y = 0f;
                }
            }

            if (_collisionState.horizontalHit)
            {
                if ((_collisionState.isCollidingLeft && ForcesApplied.x < 0f) ||
                    (_collisionState.isCollidingRight && ForcesApplied.x > 0f))
                {
                    externalForce.x = 0f;
                }
            }

            var isGrounded = _collisionState.isGrounded;
            var wasGroundedLastFrame = _collisionState.wasGroundedLastFrame;
            _collisionState.becameGroundedThisFrame = isGrounded && !wasGroundedLastFrame;
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
        /// Checks if character is touching the ground.
        /// </summary>
        /// <param name="direction">Direction the character is moving.</param>
        protected void CheckGrounded(int direction)
        {
            var isGoingRight = direction >= 0;
            var rayDirection = isGoingRight ? Vector2.right : Vector2.left;
            var initialRayOrigin = isGoingRight ? _raycastOrigins.bottomLeft : _raycastOrigins.bottomRight;

            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = initialRayOrigin;
                rayOrigin += rayDirection * (_verticalDistanceBetweenRays * i);
                rayOrigin.y += skinWidth * 2f;

                var raycastHit = Physics2D.Raycast(rayOrigin, Vector2.down, skinWidth * 4f, collisionMask);
                if (!raycastHit) //  && ignorePlatformsTime <= 0
                {
                    raycastHit = Physics2D.Raycast(rayOrigin, Vector2.down, skinWidth * 4f, oneWayPlatformMask);
                    if (raycastHit.distance <= 0f)
                    {
                        continue;
                    }
                }

                if (!raycastHit)
                {
                    continue;
                }

                _collisionState.isGrounded = true;
                _collisionState.isCollidingBelow = true;
                _collisionState.verticalHit = raycastHit;
                Debug.DrawRay(rayOrigin, Vector2.down * (skinWidth * 2f), Color.blue);
                break;
            }
        }

        /// <summary>
        /// Checks for collisions in the horizontal axis and adjust the speed accordingly to stop at the
        /// collided object.
        /// </summary>
        /// <param name="deltaMovement">The current object deltaMove used for the raycast lenght.</param>
        protected virtual void HandleHorizontalCollisions(ref Vector2 deltaMovement)
        {
            var isGoingRight = deltaMovement.x > 0f;
            var rayDistance = Mathf.Abs(deltaMovement.x) + skinWidth;
            var rayDirection = isGoingRight ? Vector2.right : Vector2.left;
            var initialRayOrigin = isGoingRight ? _raycastOrigins.bottomRight : _raycastOrigins.bottomLeft;

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

                var minDistance = Mathf.Min(Mathf.Abs(deltaMovement.x), raycastHit.distance - skinWidth);
                deltaMovement.x = minDistance * rayDirection.x;
                rayDistance = Mathf.Min(Mathf.Abs(deltaMovement.x) + skinWidth, raycastHit.distance);

                _collisionState.isCollidingLeft = !isGoingRight;
                _collisionState.isCollidingRight = isGoingRight;
                _collisionState.horizontalHit = raycastHit;

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
        /// <param name="deltaMovement">The current object deltaMove used for the raycast lenght.</param>
        protected virtual void HandleVerticalCollisions(ref Vector2 deltaMovement)
        {
            var isGoingUp = deltaMovement.y > 0f;
            var rayDistance = Mathf.Abs(deltaMovement.y) + skinWidth;
            var rayDirection = isGoingUp ? Vector2.up : Vector2.down;
            var initialRayOrigin = isGoingUp ? _raycastOrigins.topLeft : _raycastOrigins.bottomLeft;

            for (var i = 0; i < numberOfVerticalRays; i++)
            {
                var rayOrigin = initialRayOrigin;
                rayOrigin += Vector2.right * (_verticalDistanceBetweenRays * i + deltaMovement.x);

                Debug.DrawRay(rayOrigin, rayDirection * rayDistance, Color.red);
                var raycastHit = Physics2D.Raycast(rayOrigin, rayDirection, rayDistance, collisionMask);
                if (isGoingUp && !raycastHit)
                {
                    raycastHit = Physics2D.Raycast(rayOrigin, rayDirection, rayDistance, oneWayPlatformMask);
                }

                if (!raycastHit)
                {
                    continue;
                }

                deltaMovement.y = (raycastHit.distance - skinWidth) * rayDirection.y;
                rayDistance = raycastHit.distance;

                _collisionState.isCollidingAbove = isGoingUp;
                _collisionState.isCollidingBelow = !isGoingUp;
                _collisionState.verticalHit = raycastHit;
                if (_collisionState.isCollidingBelow)
                {
                    // After performing the movement, this controller have has been grounded, this is just so we
                    // don't need to wait for the next frame to flag the controller as grounded
                    _collisionState.isGrounded = true;
                }

                // We add a small fudge factor for the float operations here. if our rayDistance is smaller
                // than the width + fudge bail out because we have a direct impact
                if (rayDistance < skinWidth + kSkinWidthFloatFudgeFactor)
                    break;
            }
        }

        [Serializable] protected struct RaycastOriginInfo
        {

            public Vector2 topLeft;
            public Vector2 bottomLeft;
            public Vector2 topRight;
            public Vector2 bottomRight;

        }

        [Serializable] protected struct CollisionStateInfo
        {

            public bool isGrounded;
            public bool wasGroundedLastFrame;
            public bool becameGroundedThisFrame;

            public RaycastHit2D verticalHit;
            public RaycastHit2D horizontalHit;

            public bool isCollidingBelow;
            public bool isCollidingAbove;
            public bool isCollidingLeft;
            public bool isCollidingRight;

            public void Reset()
            {
                wasGroundedLastFrame = isGrounded;
                isGrounded = false;
                becameGroundedThisFrame = false;

                verticalHit = default;
                horizontalHit = default;

                isCollidingBelow = false;
                isCollidingAbove = false;
                isCollidingLeft = false;
                isCollidingRight = false;
            }

        }

    }

}