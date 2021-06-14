"""

@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines helper functions to create primitives and set their properties
            into the stage programmatically.
"""

from typing import Tuple, Optional
from pxr import Usd, UsdGeom, UsdShade, Sdf, Semantics, PhysicsSchema


def create_prim(stage: Usd.Stage, path: str, prim_type: str, 
                translation: Optional[Tuple[float, float, float]] = None,
                rotation: Optional[Tuple[float, float, float]] = None,
                scale: Optional[Tuple[float, float, float]] = None,
                ref: Optional[str] = None,
                semantic_label: Optional[str] = None,
                attributes: Optional[dict] = {}) -> Usd.Prim:
    """Create a prim, apply specified transforms, apply semantic label and set specified attributes.

    @note The order in which the axis rotations are recorded in a Vec3* for the six rotateABC Euler triples
          is always the same: vec[0] = X, vec[1] = Y, vec[2] = Z . The A, B, C in the op name dictate the
          order in which their corresponding elements are consumed by the rotation, not how they are laid out.
          Reference: https://graphics.pixar.com/usd/docs/api/class_usd_geom_xform_op.html

    :param stage: The stage to add prim to.
    :param path: The path of the new prim.
    :param prim_type: Prim type name
    :param translation: prim translation (applied last)
    :param rotation: prim rotation in radians with rotation order XYZ.
    :param scale: scaling factor in x, y, z.
    :param ref: Path to the USD that this prim will reference.
    :param semantic_label: Semantic label.
    :param attributes: Key-value pairs of prim attributes to set.
    """
    # Define prim in the input stage
    prim = stage.DefinePrim(path, prim_type)
    # Apply attributes from the input dictionary
    for k, v in attributes.items():
        prim.GetAttribute(k).Set(v)
    # Load reference USD file.
    if ref:
        prim.GetReferences().AddReference(ref)
    # Apply semantic label to the prim
    if semantic_label:
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set(semantic_label)
    # Apply XFORM related properties to the prim
    xform_api = UsdGeom.XformCommonAPI(prim)
    # Apply rotation in XYZ coordinates in world frame
    if rotation:
        xform_api.SetRotate(rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
    # Apply scale to the prim
    if scale:
        xform_api.SetScale(scale)
    # Apply transform (x, y, z) to the prim in world frame
    if translation:
        xform_api.SetTranslate(translation)

    return prim


def add_preview_surface(stage: Usd.Stage, prim: Usd.Prim,
                        diffuse: Tuple[float, float, float],
                        roughness: float,
                        metallic: float):
    """Add a preview surface material using the metallic workflow.
    
    Reference: https://graphics.pixar.com/usd/docs/UsdPreviewSurface-Proposal.html

    :param stage: The stage.
    :param prim: Geometric primitive in the scene.
    :param diffuse: Parameter used as diffuseColor when using the specular workflow.
                    When using metallic workflow this is interpreted as albedo.
    :param roughness: Roughness for the specular lobe. The value ranges from 0 to 1.
    :param metallic: Shading properties of material (dielectric to metallic).
    """
    # Create material associated to the prim
    path = f"{prim.GetPath()}/mat"
    material = UsdShade.Material.Define(stage, path)
    # Create a shader associated with the material
    pbrShader = UsdShade.Shader.Define(stage, f"{path}/shader")
    pbrShader.CreateIdAttr("UsdPreviewSurface")
    pbrShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(diffuse)
    pbrShader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    pbrShader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    # Bind shader to the material surface
    material.CreateSurfaceOutput().ConnectToSource(pbrShader, "surface")
    # Bind material to the prim
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def add_collision_mask(prim: Usd.Prim):
    """Add collision properties to the prim
    
    :param prim: Geometric primitive in the scene.
    """
    collisionAPI = PhysicsSchema.CollisionAPI.Apply(prim)
    collisionAPI.CreatePhysicsMaterialRel()
    collisionAPI.CreateCollisionGroupRel()

# EOF
