Audi A2 — CARLA 0.9.16 — Unity-ready textured model and SDF proxy

Copy the Audi_A2_Unity folder into your Unity project's Assets folder, retaining Textures beside the FBX files.

Audi_A2_Textured.fbx: one static rest-pose mesh, 26,749 triangles, 17 material slots. Dimensions: 3.70537 x 1.78868 x 1.54905 metres in Blender/source axes.
Audi_A2_SDF_Proxy.fbx: one untextured, connected closed manifold mesh, 8,000 triangles. Dimensions: 3.71756 x 1.80044 x 1.56376 metres. Intended for SDF baking; an SDF volume has not yet been baked.

Both exports preserve the CARLA source pivot and alignment. Object scales are 1,1,1. No manufacturer-size rescaling was applied. Both use the same FBX export configuration as the Mustang: Y-up / -Z-forward, FBX unit conversion, triangulated, selected meshes only, no cameras, lights, armatures or animation. The original rigged Audi_A2.blend remains unchanged in the separate Audi_A2 output folder.

Unity import: begin with Scale Factor 1 and file-unit conversion enabled. Give both models identical transforms. Check the vehicle length is about 3.705 Unity units. The CARLA source longitudinal direction was retained; rotate both together if your application requires a particular forward axis. The proxy's voxel envelope extends about 1 mm below source ground level, so keep the shared pivot rather than independently grounding each mesh.

Textures are included separately. The visible mesh retains the previously prepared approximate Blender materials, not an exact reproduction of CARLA's Unreal shaders. Unity Built-in/URP/HDRP materials may need adjustment, especially glass, bodywork and normal maps. Blender's green-channel inversion for Unreal normal maps is a node operation and may need recreation or baking in Unity. material_texture_mapping.json records the Blender image nodes and connections. No Unity project or render-pipeline validation was requested.

Proxy construction reproduces the Mustang method: triangle voxelization at longest dimension / 256 (14.474 mm), morphological closing, enclosed-volume filling, largest connected solid, occupancy smoothing, Blender voxel union, five smoothing iterations, and decimation to 8,000 triangles. The proxy represents a solid silhouette with small detail simplified; it is not an exact CAD reconstruction or an interior/cabin mesh.

Validation: one connected component, zero boundary/non-manifold/non-contiguous edges, no degenerate faces, outward orientation and positive signed volume. Both exported FBX files were reimported independently in Blender. Textured triangle count, texture paths, proxy topology, origin and scale passed. Maximum corresponding bounding-box limit difference is 8.32 mm. Maximum sampled deviation caused by the final simplification is 5.56 mm relative to the pre-simplified reconstructed proxy (not the original render mesh). See proxy_validation.json and export_validation.json.

Use the textured mesh for rendering and disable the proxy renderer. Enable Read/Write only if the chosen SDF baker requires CPU mesh access. Do not use this detailed concave proxy as an automatically generated convex physics collider.

The companion Audi_A2_SDF.blend has the textured mesh preserved but hidden and the proxy selected. Toggle visibility in Blender's Outliner to compare. Its textures are packed.
