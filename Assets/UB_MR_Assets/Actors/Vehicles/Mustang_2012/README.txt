Mustang Unity export

Copy this entire folder into your Unity project's Assets folder, retaining Textures beside the FBX files.

Mustang_Textured.fbx: 17 mesh parts, 40,621 triangles, 20 material assignments. Vehicle length 4.78 metres.
Mustang_SDF_Proxy.fbx: one untextured mesh, 8,000 triangles, closed manifold surface. Slightly larger envelope from voxel reconstruction (4.795 m long).

Both files use identical origins, scale and axis conversion. They are triangulated, exported Y-up / -Z-forward, with FBX unit conversion and no cameras, lights or animations.

Start with Unity model Scale Factor 1 and file unit conversion enabled. Use identical import settings and transforms for both assets. Check that the visible vehicle length is 4.78 Unity units. The source pivot is preserved, not moved to the ground.

The Textures folder contains all linked images. FBX includes material assignments and supported texture references; Unity shader appearance may require adjustment for Built-in, URP or HDRP, particularly glass, alpha and normal maps. material_texture_mapping.json records the Blender image connections. No Unity project was available for shader verification.

The proxy is intended for SDF baking. Disable its renderer when displaying the textured model. Enable CPU mesh Read/Write only if your chosen SDF baker requires it. Do not automatically use this detailed proxy as a convex physics collider.

Validation: both FBX files were reimported in a clean Blender process. Triangle counts, dimensions and texture paths passed; proxy had zero non-manifold edges and positive signed volume. See export_validation.json.
