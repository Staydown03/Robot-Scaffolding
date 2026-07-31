"""
Loads the sensor PCB geometry for the 3D viewer.

Prefers tessellating the real STEP file with OpenCascade (via cadquery-ocp) so
a revised CAD export drops straight in. If OCP is not installed, falls back to
building the board procedurally from dimensions measured out of that same STEP
file, which for this part is exact - it is a plain extruded slab with filleted
corners and four mounting holes.
"""

import math
import os

import numpy as np
import pyvista as pv

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STEP = os.path.join(HERE, "..", "cad", "HE_Sensor_fil.step")

# Measured from HE_Sensor_fil.step. Used by the procedural fallback and to
# recenter whichever source is used.
BOARD_W = 25.5      # mm, X
BOARD_H = 17.75     # mm, Y
BOARD_T = 1.75      # mm, Z
CORNER_R = 2.5      # corner fillet radius
HOLE_R = 1.25       # mounting holes, 2.5 mm diameter
HOLE_X = 10.25      # hole / fillet center offsets from board center
HOLE_Y = 6.375


def load_board(step_path=DEFAULT_STEP, deflection=0.05):
    """Return (mesh, source) with the die face at z=0 and the board below it.

    The board is recentered so the sensor die sits at the origin, which is the
    frame the displacement solver works in.
    """
    mesh = None
    source = "procedural"

    if step_path and os.path.exists(step_path):
        try:
            mesh = _load_step(step_path, deflection)
            source = "STEP"
        except ImportError:
            pass
        except Exception as exc:  # malformed or unreadable CAD
            print(f"Could not tessellate {step_path} ({exc}); using procedural board.")

    if mesh is None:
        mesh = _procedural_board()

    return _recenter(mesh), source


def _load_step(path, deflection):
    """Tessellate a STEP file into a PyVista mesh via OpenCascade."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:  # IFSelect_RetDone
        raise RuntimeError("STEP reader rejected the file")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError("STEP file contained no shape")

    BRepMesh_IncrementalMesh(shape, deflection, False, 0.2, True)

    points = []
    faces = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is not None:
            offset = len(points)
            transform = location.Transformation()
            for i in range(1, triangulation.NbNodes() + 1):
                node = triangulation.Node(i).Transformed(transform)
                points.append((node.X(), node.Y(), node.Z()))
            for i in range(1, triangulation.NbTriangles() + 1):
                a, b, c = triangulation.Triangle(i).Get()
                faces.append((3, offset + a - 1, offset + b - 1, offset + c - 1))
        explorer.Next()

    if not faces:
        raise RuntimeError("STEP shape produced no triangles")

    mesh = pv.PolyData(np.array(points, dtype=float), np.hstack(faces).astype(np.int64))
    return mesh.clean().compute_normals(auto_orient_normals=True, split_vertices=False)


def _procedural_board():
    """Build the board from its measured dimensions (exact for this part).

    Triangulates the outline and the four holes together as a constrained
    Delaunay cap, then extrudes that into a solid. Subtracting the holes with
    boolean_difference instead is tempting but unreliable: VTK's booleans need
    watertight, consistently-oriented input and will otherwise hand back a mesh
    carrying the cutting tool's own extent.
    """
    loops = [_outline_loop()] + [
        _circle_loop(hx, hy, HOLE_R)
        for hx, hy in ((HOLE_X, HOLE_Y), (-HOLE_X, HOLE_Y),
                       (-HOLE_X, -HOLE_Y), (HOLE_X, -HOLE_Y))
    ]

    points = []
    lines = []
    for loop in loops:
        offset = len(points)
        points.extend((x, y, 0.0) for x, y in loop)
        closed = list(range(offset, offset + len(loop))) + [offset]
        lines.extend([len(closed)] + closed)

    cloud = pv.PolyData(np.array(points, dtype=float))
    boundary = pv.PolyData(
        np.array(points, dtype=float), lines=np.array(lines, dtype=np.int64)
    )
    cap = cloud.delaunay_2d(edge_source=boundary)

    board = cap.extrude((0, 0, BOARD_T), capping=True).triangulate().clean()
    return board.compute_normals(auto_orient_normals=True, split_vertices=False)


def _outline_loop(per_corner=12):
    """Board outline: four filleted corners joined by straight edges."""
    corners = (
        (HOLE_X, HOLE_Y, 0.0),
        (-HOLE_X, HOLE_Y, math.pi / 2),
        (-HOLE_X, -HOLE_Y, math.pi),
        (HOLE_X, -HOLE_Y, 3 * math.pi / 2),
    )
    loop = []
    for cx, cy, start in corners:
        for step in range(per_corner + 1):
            angle = start + (math.pi / 2) * step / per_corner
            loop.append((cx + CORNER_R * math.cos(angle),
                         cy + CORNER_R * math.sin(angle)))
    return loop


def _circle_loop(cx, cy, radius, segments=32):
    return [
        (cx + radius * math.cos(2 * math.pi * i / segments),
         cy + radius * math.sin(2 * math.pi * i / segments))
        for i in range(segments)
    ]


def _recenter(mesh):
    """Move the board so the die sits at the origin.

    The STEP is authored with the board spanning z = 0..BOARD_T. The component
    face is taken to be the +Z face, so shifting it down by the board thickness
    puts the die at z = 0 with the board occupying z = -BOARD_T..0 and the
    magnet stack hanging below at negative z.
    """
    bounds = mesh.bounds
    mesh = mesh.translate(
        (
            -(bounds[0] + bounds[1]) / 2.0,
            -(bounds[2] + bounds[3]) / 2.0,
            -bounds[5],
        ),
        inplace=False,
    )
    return mesh


def magnet_mesh(radius, height, center_z, resolution=64):
    """Disc magnet, axis along z."""
    return pv.Cylinder(
        center=(0.0, 0.0, center_z), direction=(0.0, 0.0, 1.0),
        radius=radius, height=height, resolution=resolution,
    ).triangulate()
