using UnityEngine;

public class SDFRaycaster : MonoBehaviour
{
    public ComputeShader raycastShader;
    public SDFTexture sdf;
    public Vector3[] directions;
    public float maxDistance = 10f;
    public int maxIterations = 64;
    public float hitThreshold = 0.001f;

    ComputeBuffer directionsBuffer;
    ComputeBuffer intersectionsBuffer;
    Vector4[] intersectionData;

    public Vector4[] Intersections => intersectionData;

    void Start()
    {
        if (raycastShader == null || sdf == null || directions == null || directions.Length == 0)
            return;

        directionsBuffer = new ComputeBuffer(directions.Length, sizeof(float) * 3);
        directionsBuffer.SetData(directions);

        intersectionsBuffer = new ComputeBuffer(directions.Length, sizeof(float) * 4);

        int kernel = raycastShader.FindKernel("CSMain");
        raycastShader.SetBuffer(kernel, "_Directions", directionsBuffer);
        raycastShader.SetBuffer(kernel, "_Intersections", intersectionsBuffer);
        raycastShader.SetMatrix("_WorldToSDFSpace", sdf.worldToSDFTexCoords);
        raycastShader.SetTexture(kernel, "_SDF", sdf.sdf);
        raycastShader.SetFloat("_Margin", 0.0f);
        raycastShader.SetInt("_MaxIterations", maxIterations);
        raycastShader.SetFloat("_MaxDistance", maxDistance);
        raycastShader.SetFloat("_HitThreshold", hitThreshold);

        uint threadGroupSizeX;
        raycastShader.GetKernelThreadGroupSizes(kernel, out threadGroupSizeX, out _, out _);
        int groupCount = Mathf.CeilToInt((float)directions.Length / threadGroupSizeX);
        raycastShader.Dispatch(kernel, groupCount, 1, 1);

        intersectionData = new Vector4[directions.Length];
        intersectionsBuffer.GetData(intersectionData);
        foreach (Vector4 v4 in intersectionData)
        {
            Debug.Log(v4); 
        }
        //Debug.Log(intersectionData);
    }

    [ContextMenu("RayCast")]
    void TestSDF()
    {
        if (raycastShader == null || sdf == null || directions == null || directions.Length == 0)
            return;

        directionsBuffer = new ComputeBuffer(directions.Length, sizeof(float) * 3);
        directionsBuffer.SetData(directions);

        intersectionsBuffer = new ComputeBuffer(directions.Length, sizeof(float) * 4);

        int kernel = raycastShader.FindKernel("CSMain");
        raycastShader.SetBuffer(kernel, "_Directions", directionsBuffer);
        raycastShader.SetBuffer(kernel, "_Intersections", intersectionsBuffer);
        raycastShader.SetMatrix("_WorldToSDFSpace", sdf.worldToSDFTexCoords);
        raycastShader.SetTexture(kernel, "_SDF", sdf.sdf);
        raycastShader.SetFloat("_Margin", 0.0f);
        raycastShader.SetInt("_MaxIterations", maxIterations);
        raycastShader.SetFloat("_MaxDistance", maxDistance);
        raycastShader.SetFloat("_HitThreshold", hitThreshold);

        uint threadGroupSizeX;
        raycastShader.GetKernelThreadGroupSizes(kernel, out threadGroupSizeX, out _, out _);
        int groupCount = Mathf.CeilToInt((float)directions.Length / threadGroupSizeX);
        raycastShader.Dispatch(kernel, groupCount, 1, 1);

        intersectionData = new Vector4[directions.Length];
        intersectionsBuffer.GetData(intersectionData);
        foreach (Vector4 v4 in intersectionData)
        {
            Debug.Log(v4); 
        }
        
    }

    void OnDestroy()
    {
        directionsBuffer?.Release();
        intersectionsBuffer?.Release();
    }
}
