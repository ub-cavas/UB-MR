CARLA vehicle: /Game/Carla/Static/Car/4Wheeled/BmwIsetta/SK_BMW_Isetta.SK_BMW_Isetta
Textured rest-pose FBX and untextured 8000-triangle target proxy for SDF baking.
Copy this folder into Unity Assets. Matching origin/scale; FBX Y-up / -Z-forward; preserve matching transforms.
Material appearance is approximate; exported Unreal textures are provided for Unity shader setup. Normal maps may require channel convention adjustment.
Disable proxy rendering. Read/Write only if required by the baker. This is a proxy mesh, not a baked SDF volume.
See validation JSON and material warnings. Inspect previews and thin parts before production use.

Exterior repair: source-guided body/glass fitting with local triangle-crossing/inversion guards, silhouette-constrained fill, and area-weighted shading normals. See source_settings.json, exterior_validation.json, and geometry_comparison.png. The proxy approximates small trim, wheel recesses, and seams; the textured original is unchanged.
