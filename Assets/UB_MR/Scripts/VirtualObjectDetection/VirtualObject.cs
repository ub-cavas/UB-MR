using UnityEngine;

namespace CAVAS.UB_MR.DT.Sensors
{
    public class VirtualObject : MonoBehaviour
    {
        [SerializeField] Vector3 sdfTextureSize;
        [SerializeField] int sdfTextureResolution = 32;
        [SerializeField] MeshToSDF meshToSDF;
        [SerializeField] BoxCollider boundingBox;
        
        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {
            // Bounding Box Database
            VirtualBoundingBoxDetector.AddVirtualObjectToDatabase(this);
            // Render Tex for LiDAR modification
            //this.meshToSDF.sdfTexture.sdf = SetupSDF();
        }

        RenderTexture SetupSDF()
        {
            // SDF Texture properties
            this.meshToSDF.sdfTexture.size = sdfTextureSize;
            this.meshToSDF.sdfTexture.resolution = sdfTextureResolution;
            // Render Texture properties
            int width = (int)sdfTextureSize.x * sdfTextureResolution;
            int height = (int)sdfTextureSize.y * sdfTextureResolution;
            int depth = (int)sdfTextureSize.z * sdfTextureResolution;
            RenderTexture rt = new RenderTexture(width, height, depth);
            rt.dimension = UnityEngine.Rendering.TextureDimension.Tex3D;
            rt.enableRandomWrite = true;
            rt.name = "SDFTexture";
            return rt;
        }

        public Bounds GetBoundingBox()
        {
            return boundingBox.bounds;
        }

        void OnDestroy()
        {
            if (this.meshToSDF.sdfTexture != null && this.meshToSDF.sdfTexture.sdf != null)
            {
                Debug.Log("Destroying SDFTexture");
                Destroy(this.meshToSDF.sdfTexture.sdf);
            }
           
        }
    }
}
