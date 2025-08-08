using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class DepthCaptureManager : MonoBehaviour
{
    private static DepthCaptureManager instance;
    public static DepthCaptureManager Instance => instance;
    
    [Header("Depth Capture Settings")]
    [SerializeField] private RenderTexture depthTexture;
    [SerializeField] private bool debugMode = true;
    
    private Camera mainCamera;
    private Texture2D readbackTexture;
    private bool isReading = false;
    
    void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(this);
            return;
        }
        instance = this;
        
        mainCamera = GetComponent<Camera>();
        
        // Ensure depth texture is enabled in URP settings
        var urpAsset = GraphicsSettings.currentRenderPipeline as UniversalRenderPipelineAsset;
        if (urpAsset != null)
        {
            // This ensures depth texture is available for the shader
            urpAsset.supportsCameraDepthTexture = true;
        }
    }
    
    public RenderTexture GetOrCreateDepthTexture(int width, int height)
    {
        if (depthTexture == null || depthTexture.width != width || depthTexture.height != height)
        {
            if (depthTexture != null)
                depthTexture.Release();
            
            depthTexture = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat);
            depthTexture.name = "DepthCaptureTexture";
            depthTexture.filterMode = FilterMode.Point; // No filtering for accurate depth values
            depthTexture.Create();  
        }
        return depthTexture;
    }
    
    void Update()
    {
        // Handle mouse click to query depth
        if (Input.GetMouseButtonDown(0) && !isReading)
        {
            StartCoroutine(ReadDepthAtMousePosition());
        }
        
        // Debug visualization (optional)
        if (debugMode && Input.GetKeyDown(KeyCode.D))
        {
            ToggleDebugVisualization();
        }
    }
    
    private IEnumerator ReadDepthAtMousePosition()
    {
        if (depthTexture == null)
        {
            Debug.LogWarning("Depth texture not available");
            yield break;
        }
        
        isReading = true;
        
        // Wait for end of frame to ensure rendering is complete
        yield return new WaitForEndOfFrame();
        
        Vector3 mousePos = Input.mousePosition;
        
        // Clamp mouse position to screen bounds
        mousePos.x = Mathf.Clamp(mousePos.x, 0, Screen.width - 1);
        mousePos.y = Mathf.Clamp(mousePos.y, 0, Screen.height - 1);
        
        // Create temporary texture for readback if needed
        if (readbackTexture == null || 
            readbackTexture.width != depthTexture.width || 
            readbackTexture.height != depthTexture.height)
        {
            if (readbackTexture != null)
                Destroy(readbackTexture);
            
            readbackTexture = new Texture2D(depthTexture.width, depthTexture.height, TextureFormat.RFloat, false);
        }
        
        // Read the depth texture
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = depthTexture;
        
        readbackTexture.ReadPixels(new Rect(0, 0, depthTexture.width, depthTexture.height), 0, 0);
        readbackTexture.Apply();
        
        RenderTexture.active = currentRT;
        
        // Convert mouse position to texture coordinates
        int x = Mathf.RoundToInt((mousePos.x / Screen.width) * depthTexture.width);
        int y = Mathf.RoundToInt((mousePos.y / Screen.height) * depthTexture.height);
        
        // Get the depth value
        Color depthColor = readbackTexture.GetPixel(x, y);
        float depthInMeters = depthColor.r; // Depth stored in red channel
        
        // Output to console
        Debug.Log($"=== Depth Query ===");
        Debug.Log($"Mouse Position: ({mousePos.x}, {mousePos.y})");
        Debug.Log($"Texture Coordinates: ({x}, {y})");
        Debug.Log($"Z-Distance: {depthInMeters:F3} meters");
        
        // Also show world position for context
        Vector3 worldPos = mainCamera.ScreenToWorldPoint(new Vector3(mousePos.x, mousePos.y, depthInMeters));
        Debug.Log($"World Position: {worldPos}");
        Debug.Log($"==================");
        
        isReading = false;
    }
    
    private void ToggleDebugVisualization()
    {
        // Optional: Add debug visualization quad
        GameObject debugQuad = GameObject.Find("DepthDebugQuad");
        
        if (debugQuad == null)
        {
            debugQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
            debugQuad.name = "DepthDebugQuad";
            debugQuad.transform.position = new Vector3(0, 0, 5);
            debugQuad.transform.localScale = new Vector3(4, 3, 1);
            
            Material debugMat = new Material(Shader.Find("Unlit/Texture"));
            debugMat.mainTexture = depthTexture;
            debugQuad.GetComponent<Renderer>().material = debugMat;
            
            Debug.Log("Debug visualization enabled - showing depth texture on quad");
        }
        else
        {
            Destroy(debugQuad);
            Debug.Log("Debug visualization disabled");
        }
    }
    
    void OnDestroy()
    {
        if (depthTexture != null)
        {
            depthTexture.Release();
            Destroy(depthTexture);
        }
        
        if (readbackTexture != null)
        {
            Destroy(readbackTexture);
        }
        
        if (instance == this)
        {
            instance = null;
        }
    }
    
    // Public API for getting depth at specific screen coordinates
    public float GetDepthAtScreenPoint(Vector2 screenPoint, System.Action<float> callback)
    {
        StartCoroutine(ReadDepthAtPoint(screenPoint, callback));
        return -1f; // Return immediately, actual value comes through callback
    }
    
    private IEnumerator ReadDepthAtPoint(Vector2 screenPoint, System.Action<float> callback)
    {
        // Similar to ReadDepthAtMousePosition but with custom coordinates
        yield return new WaitForEndOfFrame();
        
        // ... (similar implementation)
        
        if (callback != null)
            callback(0f); // Replace with actual depth
    }
}
