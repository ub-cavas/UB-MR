using Unity.Cinemachine;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace CAVAS.UB_MR.CameraControls
{
    /// <summary>
    /// Runtime free-fly camera override for scene inspection. Attach this to an always-active
    /// GameObject; it will create and activate a separate passive CinemachineCamera.
    /// </summary>
    public class FreeFlyCinemachineCameraController : MonoBehaviour
    {
        [Header("Camera")]
        [SerializeField] CinemachineCamera freeFlyCamera;
        [SerializeField] Camera outputCamera;
        [SerializeField] string cameraName = "Free Fly Cinemachine Camera";
        [SerializeField] bool createCameraOnAwake = true;
        [SerializeField] bool activateOnStart;
        [SerializeField] int inactivePriority = -100;
        [SerializeField] int activePriority = 1000;
        [SerializeField] Transform activationStartTransform;
        [SerializeField] bool startFromOutputCamera = true;

        [Header("Movement")]
        [SerializeField] float moveSpeed = 10f;
        [SerializeField] float fastMoveMultiplier = 4f;
        [SerializeField] float slowMoveMultiplier = 0.25f;
        [SerializeField] float lookSensitivity = 0.12f;
        [SerializeField] float rollSpeed = 75f;
        [SerializeField] bool useUnscaledTime = true;
        [SerializeField] bool requireRightMouseForLook = true;
        [SerializeField] bool lockCursorWhileActive = true;

#if ENABLE_LEGACY_INPUT_MANAGER
        [Header("Legacy Input")]
        [SerializeField] KeyCode toggleKey = KeyCode.F;
        [SerializeField] KeyCode cancelKey = KeyCode.Escape;
        [SerializeField] KeyCode forwardKey = KeyCode.W;
        [SerializeField] KeyCode backwardKey = KeyCode.S;
        [SerializeField] KeyCode rightKey = KeyCode.D;
        [SerializeField] KeyCode leftKey = KeyCode.A;
        [SerializeField] KeyCode upKey = KeyCode.E;
        [SerializeField] KeyCode downKey = KeyCode.Q;
        [SerializeField] KeyCode rollClockwiseKey = KeyCode.C;
        [SerializeField] KeyCode rollCounterClockwiseKey = KeyCode.Z;
        [SerializeField] KeyCode fastKey = KeyCode.LeftShift;
        [SerializeField] KeyCode slowKey = KeyCode.LeftControl;
#endif

        bool isActive;
        CursorLockMode previousCursorLockMode;
        bool previousCursorVisible;

        void Awake()
        {
            if (outputCamera == null)
                outputCamera = Camera.main != null ? Camera.main : FindFirstObjectByType<Camera>();

            if (outputCamera != null && outputCamera.GetComponent<CinemachineBrain>() == null)
                outputCamera.gameObject.AddComponent<CinemachineBrain>();

            if (freeFlyCamera == null && createCameraOnAwake)
                freeFlyCamera = CreateFreeFlyCamera();

            SetCameraLive(false);
        }

        void Start()
        {
            if (activateOnStart)
                Activate();
        }

        void Update()
        {
            if (WasTogglePressed())
            {
                if (isActive)
                    Deactivate();
                else
                    Activate();
            }

            if (!isActive || freeFlyCamera == null)
                return;

            if (WasCancelPressed())
            {
                Deactivate();
                return;
            }

            UpdateMotion(DeltaTime());
        }

        public void Activate()
        {
            if (isActive)
                return;

            if (freeFlyCamera == null)
                freeFlyCamera = CreateFreeFlyCamera();

            if (freeFlyCamera == null)
            {
                Debug.LogWarning("Free-fly camera could not be created or assigned.");
                return;
            }

            PlaceCameraAtActivationStart();
            isActive = true;
            SetCameraLive(true);
            CaptureCursor();
        }

        public void Deactivate()
        {
            if (!isActive)
                return;

            isActive = false;
            SetCameraLive(false);
            RestoreCursor();
        }

        CinemachineCamera CreateFreeFlyCamera()
        {
            GameObject cameraObject = new GameObject(cameraName);
            cameraObject.transform.SetPositionAndRotation(transform.position, transform.rotation);
            cameraObject.SetActive(false);

            CinemachineCamera camera = cameraObject.AddComponent<CinemachineCamera>();
            camera.Priority.Value = inactivePriority;
            camera.StandbyUpdate = CinemachineVirtualCameraBase.StandbyUpdateMode.Never;
            return camera;
        }

        void SetCameraLive(bool live)
        {
            if (freeFlyCamera == null)
                return;

            freeFlyCamera.Priority.Value = live ? activePriority : inactivePriority;

            if (freeFlyCamera.gameObject != gameObject)
                freeFlyCamera.gameObject.SetActive(live);
            else
                freeFlyCamera.enabled = live;
        }

        void PlaceCameraAtActivationStart()
        {
            Transform startTransform = activationStartTransform;

            if (startTransform == null && startFromOutputCamera && outputCamera != null)
                startTransform = outputCamera.transform;

            if (startTransform == null)
                startTransform = transform;

            freeFlyCamera.transform.SetPositionAndRotation(startTransform.position, startTransform.rotation);
            freeFlyCamera.ForceCameraPosition(startTransform.position, startTransform.rotation);
        }

        void UpdateMotion(float deltaTime)
        {
            Transform cameraTransform = freeFlyCamera.transform;

            Vector3 localMove = ReadLocalMove();
            if (localMove.sqrMagnitude > 1f)
                localMove.Normalize();

            float speedMultiplier = ReadSpeedMultiplier();
            cameraTransform.position += cameraTransform.TransformDirection(localMove) * moveSpeed * speedMultiplier * deltaTime;

            Vector2 lookDelta = ReadLookDelta();
            if (lookDelta.sqrMagnitude > 0f)
            {
                Vector3 euler = cameraTransform.eulerAngles;
                euler.x = NormalizeAngle(euler.x - lookDelta.y * lookSensitivity);
                euler.y += lookDelta.x * lookSensitivity;
                cameraTransform.rotation = Quaternion.Euler(euler);
            }

            float rollInput = ReadRollInput();
            if (!Mathf.Approximately(rollInput, 0f))
                cameraTransform.Rotate(Vector3.forward, -rollInput * rollSpeed * deltaTime, Space.Self);
        }

        Vector3 ReadLocalMove()
        {
            Vector3 move = Vector3.zero;

#if ENABLE_INPUT_SYSTEM
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null)
            {
                if (keyboard.wKey.isPressed)
                    move.z += 1f;
                if (keyboard.sKey.isPressed)
                    move.z -= 1f;
                if (keyboard.dKey.isPressed)
                    move.x += 1f;
                if (keyboard.aKey.isPressed)
                    move.x -= 1f;
                if (keyboard.eKey.isPressed)
                    move.y += 1f;
                if (keyboard.qKey.isPressed)
                    move.y -= 1f;
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKey(forwardKey))
                move.z += 1f;
            if (Input.GetKey(backwardKey))
                move.z -= 1f;
            if (Input.GetKey(rightKey))
                move.x += 1f;
            if (Input.GetKey(leftKey))
                move.x -= 1f;
            if (Input.GetKey(upKey))
                move.y += 1f;
            if (Input.GetKey(downKey))
                move.y -= 1f;
#endif

            return move;
        }

        Vector2 ReadLookDelta()
        {
            if (requireRightMouseForLook && !IsLookButtonPressed())
                return Vector2.zero;

#if ENABLE_INPUT_SYSTEM
            Mouse mouse = Mouse.current;
            return mouse != null ? mouse.delta.ReadValue() : Vector2.zero;
#elif ENABLE_LEGACY_INPUT_MANAGER
            return new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y")) * 10f;
#else
            return Vector2.zero;
#endif
        }

        float ReadRollInput()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
                return 0f;

            float roll = 0f;
            if (keyboard.cKey.isPressed)
                roll += 1f;
            if (keyboard.zKey.isPressed)
                roll -= 1f;
            return roll;
#elif ENABLE_LEGACY_INPUT_MANAGER
            float roll = 0f;
            if (Input.GetKey(rollClockwiseKey))
                roll += 1f;
            if (Input.GetKey(rollCounterClockwiseKey))
                roll -= 1f;
            return roll;
#else
            return 0f;
#endif
        }

        float ReadSpeedMultiplier()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
                return 1f;

            if (keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed)
                return fastMoveMultiplier;

            if (keyboard.leftCtrlKey.isPressed || keyboard.rightCtrlKey.isPressed)
                return slowMoveMultiplier;

            return 1f;
#elif ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKey(fastKey))
                return fastMoveMultiplier;

            if (Input.GetKey(slowKey))
                return slowMoveMultiplier;

            return 1f;
#else
            return 1f;
#endif
        }

        bool WasTogglePressed()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard.fKey.wasPressedThisFrame)
                return true;
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKeyDown(toggleKey);
#else
            return false;
#endif
        }

        bool WasCancelPressed()
        {
#if ENABLE_INPUT_SYSTEM
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && keyboard.escapeKey.wasPressedThisFrame)
                return true;
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKeyDown(cancelKey);
#else
            return false;
#endif
        }

        bool IsLookButtonPressed()
        {
#if ENABLE_INPUT_SYSTEM
            Mouse mouse = Mouse.current;
            if (mouse != null)
                return mouse.rightButton.isPressed;
#endif

#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetMouseButton(1);
#else
            return false;
#endif
        }

        float DeltaTime()
        {
            return useUnscaledTime ? Time.unscaledDeltaTime : Time.deltaTime;
        }

        static float NormalizeAngle(float angle)
        {
            angle %= 360f;
            if (angle > 180f)
                angle -= 360f;
            return angle;
        }

        void CaptureCursor()
        {
            if (!lockCursorWhileActive)
                return;

            previousCursorLockMode = Cursor.lockState;
            previousCursorVisible = Cursor.visible;
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }

        void RestoreCursor()
        {
            if (!lockCursorWhileActive)
                return;

            Cursor.lockState = previousCursorLockMode;
            Cursor.visible = previousCursorVisible;
        }

        void OnDisable()
        {
            if (isActive)
                Deactivate();
        }
    }
}
