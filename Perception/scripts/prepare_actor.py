#!/usr/bin/python3
"""Turn Mixamo-via-Blender Collada animations into a Gazebo-ready animated
person, for walk_person.py.

    prepare_actor.py [anims_dir] [original_dae] [out_dir]

defaults:
    anims_dir     /workspace/standing_person_src/anims   (walk.dae required;
                  run.dae, dance.dae, idle.dae, or any other name optional)
    original_dae  /workspace/standing_person_src/meshes/standing.dae
    out_dir       /workspace/Perception/models/animated_person

Re-run it whenever you add or change an animation. For each <name>.dae it
writes out_dir/<name>.dae, plus the textures and meta.json (height, and per
animation its length and natural ground speed) that walk_person.py reads.

What it fixes, and why:
  * Blender exports the Mixamo rig under an "Armature" node carrying a 0.01
    scale and a 90deg rotation. Gazebo mis-applies that node (the person
    renders as a giant shredded mesh), so it's folded into the skeleton
    itself, joint by joint, leaving an identity root like Gazebo's own actor
    meshes. In the same step the person is scaled to TARGET_HEIGHT_M,
    turned to face +X (the direction Gazebo actors walk), and lifted so his
    feet are on the ground (z = 0).
  * Blender pads every animation to the scene's frame range (250 frames), so
    a ~1s walk cycle would play and then freeze for 7s each loop. Keyframes
    after the last change are cut.
  * Natural ground speed of walk/run cycles (from how fast the planted foot
    slides backward), so walk_person.py can move him at a matching speed
    and advance the animation by distance instead of time: no foot sliding.
  * The skin (walk.dae's mesh) had lost every texture: the parts were joined
    in Blender, which kept only the active part's UVs (the jeans) and zeroed
    the rest, and Mixamo then dropped the materials. Triangle order survived
    the round trip, so each part's UVs and texture are copied back from the
    original standing.dae by matching triangle shapes. Skipped if the skin
    already has materials (e.g. a different, properly textured character).
"""
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

NS = "http://www.collada.org/2005/11/COLLADASchema"
C = "{%s}" % NS
ET.register_namespace("", NS)
TARGET_HEIGHT_M = 1.9  # matches person_detector's person_height_m
SKIN = "walk"


# --- Collada helpers -------------------------------------------------------

def floats(el):
    return np.array([float(x) for x in el.text.split()]) if el is not None and el.text else np.zeros(0)


def set_floats(el, values, fmt="%.7g"):
    el.text = " ".join(fmt % v for v in np.ravel(values))
    el.set("count", str(np.size(values)))


def mat(el):
    return floats(el).reshape(4, 4)


def by_id(root):
    return {e.get("id"): e for e in root.iter() if e.get("id")}


def scene_roots(root):
    return [n for vs in root.iter(C + "visual_scene") for n in vs.findall(C + "node")]


def find_armature(root):
    """(armature node or None, its root joints). Armature = the top-level
    non-joint node whose children include joints."""
    for n in scene_roots(root):
        if n.get("type") == "JOINT":
            return None, [n]
        joints = [c for c in n.findall(C + "node") if c.get("type") == "JOINT"]
        if joints:
            return n, joints
    sys.exit("No skeleton (JOINT nodes) found.")


def joint_nodes(root_joints):
    out = []
    def rec(n):
        if n.get("type") == "JOINT":
            out.append(n)
        for c in n.findall(C + "node"):
            rec(c)
    for r in root_joints:
        rec(r)
    return out


class Channels:
    """Matrix animation channels: node id -> (input source, output source)."""

    def __init__(self, root):
        self.ch = {}
        ids = by_id(root)
        for an in root.iter(C + "animation"):
            for ch in an.findall(C + "channel"):
                target = ch.get("target")
                if not target.endswith("/transform"):
                    continue
                sampler = ids[ch.get("source")[1:]]
                inp = {i.get("semantic"): ids[i.get("source")[1:]] for i in sampler.findall(C + "input")}
                self.ch[target.split("/")[0]] = (an, inp)

    def times(self, nid):
        return floats(self.ch[nid][1]["INPUT"].find(C + "float_array"))

    def mats(self, nid):
        return floats(self.ch[nid][1]["OUTPUT"].find(C + "float_array")).reshape(-1, 4, 4)

    def set_mats(self, nid, m):
        set_floats(self.ch[nid][1]["OUTPUT"].find(C + "float_array"), m)

    def trim(self, n):
        """Keep the first n keys of every channel (all sources of each sampler)."""
        for an, inp in self.ch.values():
            for src in inp.values():
                acc = src.find(".//" + C + "accessor")
                stride = int(acc.get("stride", "1"))
                arr = src.find(C + "float_array")
                if arr is not None:
                    set_floats(arr, floats(arr)[: n * stride])
                else:
                    names = src.find(C + "Name_array")
                    names.text = " ".join(names.text.split()[:n])
                    names.set("count", str(n))
                acc.set("count", str(n))


def quat(R):
    """Rotation matrix -> unit quaternion [w, x, y, z]."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rot_z(a):
    m = np.eye(4)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = math.cos(a), -math.sin(a), math.sin(a), math.cos(a)
    return m


# --- Skeleton transform ---------------------------------------------------

class Rig:
    """One file's skeleton + animation, with the armature folded in."""

    def __init__(self, path):
        self.path = path
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.arm, self.root_joints = find_armature(self.root)
        self.A = mat(self.arm.find(C + "matrix")) if self.arm is not None and self.arm.find(C + "matrix") is not None else np.eye(4)
        self.joints = joint_nodes(self.root_joints)
        self.ch = Channels(self.root)
        self.nkeys = max((len(self.ch.times(j.get("id"))) for j in self.joints if j.get("id") in self.ch.ch), default=0)
        self.sA = abs(np.linalg.det(self.A[:3, :3])) ** (1 / 3)

    def local(self, j, i):
        """Joint j's local matrix at key i, or its rest matrix for i=None."""
        nid = j.get("id")
        if i is not None and nid in self.ch.ch:
            m = self.ch.mats(nid)
            return m[min(i, len(m) - 1)]
        return mat(j.find(C + "matrix"))

    def world(self, i, E=None, k=1.0, dz=0.0):
        """World matrix of every joint (by sid, name and id) at key i (None:
        rest pose), under the correction E (rotation) * k (scale) applied on
        top of the armature, plus dz. Rigid: the armature's scale goes into
        the translations (D) instead of the rotations."""
        T = (E if E is not None else np.eye(4)) @ np.diag([k, k, k, 1]) @ self.A
        c = self.sA * k
        D, Dinv = np.diag([c, c, c, 1]), np.diag([1 / c, 1 / c, 1 / c, 1])
        out = {}
        def rec(n, parent, is_root):
            L = self.local(n, i)
            L2 = T @ L @ Dinv if is_root else D @ L @ Dinv
            if is_root:
                L2 = L2.copy(); L2[2, 3] += dz
            W = parent @ L2
            for key in (n.get("sid"), n.get("name"), n.get("id")):
                if key:
                    out[key] = W
            for ch in n.findall(C + "node"):
                if ch.get("type") == "JOINT":
                    rec(ch, W, False)
        for r in self.root_joints:
            rec(r, np.eye(4), True)
        return out

    def last_change(self):
        """Index of the last key at which any channel still changes."""
        last = 0
        for j in self.joints:
            nid = j.get("id")
            if nid in self.ch.ch:
                m = self.ch.mats(nid)
                diff = np.abs(np.diff(m, axis=0)).reshape(len(m) - 1, -1).max(axis=1)
                moving = np.nonzero(diff > 1e-5)[0]
                if len(moving):
                    last = max(last, moving[-1] + 1)
        return last

    def apply(self, E, k, dz, keep):
        """Bake the correction G = E * k * armature into the file, leaving only
        rigid matrices and the armature as identity, like Gazebo's own actor
        meshes: root joints (rest + keys) -> G L D^-1 (+dz), other joints ->
        D L D^-1 (D = the overall scale on translations). If the file has a
        skin: vertices -> G v (+dz), normals -> rotated, bind shape ->
        identity, inverse binds -> inverse of the new rest pose (the rest
        pose is the bind pose in Blender's exports). Trims keys to `keep`."""
        G = E @ np.diag([k, k, k, 1]) @ self.A
        c = self.sA * k
        D, Dinv = np.diag([c, c, c, 1]), np.diag([1 / c, 1 / c, 1 / c, 1])
        rest_new = self.world(None, E, k, dz)  # before the matrices change
        roots = {r.get("id") for r in self.root_joints}
        def fix(L, is_root):
            L2 = G @ L @ Dinv if is_root else D @ L @ Dinv
            if is_root:
                L2 = L2.copy(); L2[..., 2, 3] += dz
            return L2
        for j in self.joints:
            is_root = j.get("id") in roots
            m = j.find(C + "matrix")
            set_floats(m, fix(mat(m), is_root))
            m.attrib.pop("count", None)
            if j.get("id") in self.ch.ch:
                self.ch.set_mats(j.get("id"), np.array([fix(L, is_root) for L in self.ch.mats(j.get("id"))]))
        if self.arm is not None and self.arm.find(C + "matrix") is not None:
            set_floats(self.arm.find(C + "matrix"), np.eye(4))
            self.arm.find(C + "matrix").attrib.pop("count", None)
        ids = by_id(self.root)
        R = G[:3, :3] / c  # pure rotation
        for skin in self.root.iter(C + "skin"):
            bsm = skin.find(C + "bind_shape_matrix")
            B = mat(bsm) if bsm is not None else np.eye(4)
            if bsm is not None:
                set_floats(bsm, np.eye(4)); bsm.attrib.pop("count", None)
            srcs = {s.get("id"): s for s in skin.findall(C + "source")}
            jin = {i.get("semantic"): srcs[i.get("source")[1:]] for i in skin.find(C + "joints").findall(C + "input")}
            names = jin["JOINT"].find(C + "Name_array").text.split()
            set_floats(jin["INV_BIND_MATRIX"].find(C + "float_array"),
                       np.array([np.linalg.inv(rest_new[n]) for n in names]))
            mesh = ids[skin.get("source")[1:]].find(C + "mesh")
            for vin in mesh.find(C + "vertices").findall(C + "input"):
                arr = ids[vin.get("source")[1:]].find(C + "float_array")
                V = floats(arr).reshape(-1, 3)
                if vin.get("semantic") == "POSITION":
                    Vh = np.c_[V, np.ones(len(V))] @ (G @ B).T
                    Vh[:, 2] += dz
                    set_floats(arr, Vh[:, :3])
            for grp in mesh:
                for inp in grp.findall(C + "input"):
                    if inp.get("semantic") == "NORMAL":
                        arr = ids[inp.get("source")[1:]].find(C + "float_array")
                        N = floats(arr).reshape(-1, 3) @ (R @ B[:3, :3]).T
                        N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
                        set_floats(arr, N)
                        break
        self.ch.trim(keep)


class Skin:
    """Skinned mesh of the skin file, to compute posed vertex positions."""

    def __init__(self, root):
        skin = root.find(".//" + C + "skin")
        ids = by_id(root)
        geo = ids[skin.get("source")[1:]]
        mesh = geo.find(C + "mesh")
        vin = [i for i in mesh.find(C + "vertices").findall(C + "input") if i.get("semantic") == "POSITION"][0]
        self.V = floats(ids[vin.get("source")[1:]].find(C + "float_array")).reshape(-1, 3)
        self.bsm = mat(skin.find(C + "bind_shape_matrix")) if skin.find(C + "bind_shape_matrix") is not None else np.eye(4)
        srcs = {s.get("id"): s for s in skin.findall(C + "source")}
        jin = {i.get("semantic"): srcs[i.get("source")[1:]] for i in skin.find(C + "joints").findall(C + "input")}
        self.names = jin["JOINT"].find(C + "Name_array").text.split()
        vw = skin.find(C + "vertex_weights")
        win = {i.get("semantic"): (int(i.get("offset")), srcs[i.get("source")[1:]]) for i in vw.findall(C + "input")}
        w = floats(win["WEIGHT"][1].find(C + "float_array"))
        vc = [int(x) for x in vw.find(C + "vcount").text.split()]
        v = np.array([int(x) for x in vw.find(C + "v").text.split()]).reshape(-1, 2)
        self.rows = np.repeat(np.arange(len(vc)), vc)
        self.joint = v[:, win["JOINT"][0]]
        self.weight = w[v[:, win["WEIGHT"][0]]]

    def posed(self, world, rest, G):
        """Vertex positions with joints at `world`, given the skin rig's rest
        pose `rest` (both from Rig.world under the same correction G) - the
        same skinning Gazebo does on the converted file."""
        skinm = np.array([world[n] @ np.linalg.inv(rest[n]) for n in self.names])
        vh = np.c_[self.V, np.ones(len(self.V))] @ (G @ self.bsm).T
        per = np.einsum("nij,nj->ni", skinm[self.joint], vh[self.rows]) * self.weight[:, None]
        out = np.zeros((len(self.V), 4))
        np.add.at(out, self.rows, per)
        return out[:, :3]


# --- Measurements -----------------------------------------------------------

def find_joint(world, *keys):
    for name in world:
        low = name.lower()
        if all(k in low for k in keys):
            return name
    return None


def facing_angle(rig):
    """Average horizontal heading of toes relative to ankles, in the armature
    frame (no correction), over the animation."""
    vecs = []
    for i in range(0, max(rig.nkeys, 1), 3):
        w = rig.world(i)
        for side in ("left", "right"):
            foot, toe = find_joint(w, side, "foot"), find_joint(w, side, "toe")
            if foot and toe:
                vecs.append(w[toe][:2, 3] - w[foot][:2, 3])
    if not vecs:
        sys.exit("Couldn't find foot/toe joints to work out which way he faces.")
    v = np.mean(vecs, axis=0)
    return math.atan2(v[1], v[0])


def ground_speed(rig, E, k, keep):
    """Natural ground speed (m/s) of a cyclic gait, from the planted foot's
    backward slide (he faces +X after E). 0 for animations without one."""
    times = rig.ch.times(rig.root_joints[0].get("id"))[:keep] if rig.root_joints[0].get("id") in rig.ch.ch else None
    if times is None or len(times) < 4:
        return 0.0
    speeds = []
    for side in ("left", "right"):
        name = None
        pos = []
        for i in range(keep):
            w = rig.world(i, E, k)
            name = name or find_joint(w, side, "foot")
            pos.append(w[name][:3, 3])
        pos = np.array(pos)
        vx = np.diff(pos[:, 0]) / np.diff(times)
        z = pos[:-1, 2]
        planted = z <= np.percentile(z, 25)
        speeds.append(-vx[planted].mean())
    return float(np.mean(speeds))


# --- Texture restore ----------------------------------------------------------

def tri_groups(geo):
    mesh = geo.find(C + "mesh")
    return [t for t in mesh if t.tag in (C + "triangles", C + "polylist")]


def tri_data(geo, root):
    """Corner positions (n,3,3) and uvs (n,3,2) over all triangle groups."""
    ids = by_id(root)
    mesh = geo.find(C + "mesh")
    vin = [i for i in mesh.find(C + "vertices").findall(C + "input") if i.get("semantic") == "POSITION"][0]
    P = floats(ids[vin.get("source")[1:]].find(C + "float_array")).reshape(-1, 3)
    pos, uvs = [], []
    for t in tri_groups(geo):
        ins = t.findall(C + "input")
        if t.tag == C + "polylist" and set(t.find(C + "vcount").text.split()) != {"3"}:
            sys.exit("Non-triangle faces in %s; triangulate before exporting." % geo.get("id"))
        stride = max(int(i.get("offset")) for i in ins) + 1
        p = np.array([int(x) for x in t.find(C + "p").text.split()]).reshape(-1, stride)
        vo = int([i for i in ins if i.get("semantic") == "VERTEX"][0].get("offset"))
        uvi = [i for i in ins if i.get("semantic") == "TEXCOORD"]
        pos.append(P[p[:, vo]].reshape(-1, 3, 3))
        if uvi:
            UV = floats(ids[uvi[0].get("source")[1:]].find(C + "float_array")).reshape(-1, 2)
            uvs.append(UV[p[:, int(uvi[0].get("offset"))]].reshape(-1, 3, 2))
        else:
            uvs.append(np.zeros((len(p) // 3, 3, 2)))
    return np.concatenate(pos), np.concatenate(uvs)


def shape(T):
    e = np.stack([np.linalg.norm(T[:, 1] - T[:, 0], axis=1),
                  np.linalg.norm(T[:, 2] - T[:, 1], axis=1),
                  np.linalg.norm(T[:, 0] - T[:, 2], axis=1)], 1)
    return e / (e.sum(1, keepdims=True) + 1e-12)


def restore_textures(skin_root, orig_path, out_dir):
    """Give the skin back the original's per-part UVs and textures."""
    geo = skin_root.find(".//" + C + "skin")
    geo = by_id(skin_root)[geo.get("source")[1:]]
    if skin_root.find(".//" + C + "library_materials/" + C + "material") is not None:
        print("  skin already has materials; leaving its textures alone")
        return
    orig = ET.parse(orig_path).getroot()
    oids = by_id(orig)
    parts = []  # (geometry, material id, corner uvs, shape) per original triangle group
    for ig in orig.iter(C + "instance_geometry"):
        g = oids[ig.get("url")[1:]]
        binds = {b.get("symbol"): b.get("target")[1:] for b in ig.iter(C + "instance_material")}
        pos, uvs = tri_data(g, orig)
        o = 0
        for t in tri_groups(g):
            n = int(t.get("count"))
            parts.append((g.get("id"), binds.get(t.get("material")), uvs[o:o + n], shape(pos[o:o + n])))
            o += n
    npos, _ = tri_data(geo, skin_root)
    nshape = shape(npos)
    # Greedy: at each offset take the unused part whose triangle shapes match.
    order, o, left = [], 0, list(range(len(parts)))
    while o < len(nshape):
        best = min(left, key=lambda i: np.abs(nshape[o:o + len(parts[i][3])] - parts[i][3]).mean()
                   if o + len(parts[i][3]) <= len(nshape) else 1e9)
        err = np.abs(nshape[o:o + len(parts[best][3])] - parts[best][3]).mean()
        if err > 1e-3:
            print(f"  can't match triangles at {o} to the original (err {err:.4f}); textures not restored")
            return
        order.append(best); left.remove(best); o += len(parts[best][3])
    print("  restored textures for parts:", ", ".join(parts[i][0].replace("-mesh", "") for i in order))

    # New mesh: one triangle group per part, per-corner UVs from the original.
    mesh = geo.find(C + "mesh")
    groups = tri_groups(geo)
    ins = groups[0].findall(C + "input")
    stride = max(int(i.get("offset")) for i in ins) + 1
    p = np.concatenate([np.array([int(x) for x in t.find(C + "p").text.split()]).reshape(-1, stride) for t in groups])
    for t in groups:
        mesh.remove(t)
    uv_all = np.concatenate([parts[i][2] for i in order]).reshape(-1, 2)
    uvsrc = ET.SubElement(mesh, C + "source", id=geo.get("id") + "-restored-uv")
    fa = ET.SubElement(uvsrc, C + "float_array", id=geo.get("id") + "-restored-uv-array")
    set_floats(fa, uv_all)
    tc = ET.SubElement(uvsrc, C + "technique_common")
    acc = ET.SubElement(tc, C + "accessor", source="#" + fa.get("id"), count=str(len(uv_all)), stride="2")
    ET.SubElement(acc, C + "param", name="S", type="float")
    ET.SubElement(acc, C + "param", name="T", type="float")
    # Sources must come before <vertices>/<triangles>: move it up.
    mesh.remove(uvsrc)
    mesh.insert(list(mesh).index(mesh.find(C + "vertices")), uvsrc)
    keep = [i for i in ins if i.get("semantic") != "TEXCOORD"]
    corner = 0
    for i in order:
        n = len(parts[i][2])
        tri = ET.SubElement(mesh, C + "triangles", material=parts[i][1], count=str(n))
        for k_, inp in enumerate(keep):
            ET.SubElement(tri, C + "input", semantic=inp.get("semantic"), source=inp.get("source"), offset=str(k_))
        ET.SubElement(tri, C + "input", semantic="TEXCOORD", source="#" + uvsrc.get("id"), offset=str(len(keep)), set="0")
        rows = p[corner:corner + 3 * n]
        cols = [rows[:, int(inp.get("offset"))] for inp in keep] + [np.arange(corner, corner + 3 * n)]
        ET.SubElement(tri, C + "p").text = " ".join(map(str, np.stack(cols, 1).ravel()))
        corner += 3 * n

    # Copy the used materials/effects/images, images into out_dir/textures.
    used = {parts[i][1] for i in order}
    libs = {}
    for tag in ("library_images", "library_effects", "library_materials"):
        lib = skin_root.find(C + tag)
        if lib is None:
            lib = ET.Element(C + tag)
            skin_root.insert(list(skin_root).index(skin_root.find(C + "library_geometries")), lib)
        libs[tag] = lib
    os.makedirs(os.path.join(out_dir, "textures"), exist_ok=True)
    orig_dir = os.path.dirname(orig_path)
    for mid in used:
        m = oids[mid]
        libs["library_materials"].append(m)
        eff = oids[m.find(C + "instance_effect").get("url")[1:]]
        libs["library_effects"].append(eff)
        for init in eff.iter(C + "init_from"):
            img = oids.get(init.text)
            if img is not None and img.tag == C + "image":
                src = os.path.normpath(os.path.join(orig_dir, img.find(C + "init_from").text))
                shutil.copy(src, os.path.join(out_dir, "textures", os.path.basename(src)))
                img = ET.fromstring(ET.tostring(img))
                img.find(C + "init_from").text = "textures/" + os.path.basename(src)
                if img.get("id") not in {e.get("id") for e in libs["library_images"]}:
                    libs["library_images"].append(img)
    ic = skin_root.find(".//" + C + "instance_controller")
    bm = ic.find(C + "bind_material")
    if bm is not None:
        ic.remove(bm)
    tc = ET.SubElement(ET.SubElement(ic, C + "bind_material"), C + "technique_common")
    for mid in used:
        im = ET.SubElement(tc, C + "instance_material", symbol=mid, target="#" + mid)
        ET.SubElement(im, C + "bind_vertex_input", semantic="UVMap", input_semantic="TEXCOORD", input_set="0")


# --- Main ------------------------------------------------------------------------

def main():
    anims_dir = sys.argv[1] if len(sys.argv) > 1 else "/workspace/standing_person_src/anims"
    orig = sys.argv[2] if len(sys.argv) > 2 else "/workspace/standing_person_src/meshes/standing.dae"
    out_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "models", "animated_person")
    out_dir = os.path.abspath(out_dir)
    names = sorted(f[:-4] for f in os.listdir(anims_dir) if f.lower().endswith(".dae"))
    if SKIN not in names:
        sys.exit(f"{anims_dir} needs a {SKIN}.dae (it also provides the mesh).")
    os.makedirs(out_dir, exist_ok=True)

    # Correction from the skin file: face +X, TARGET_HEIGHT_M tall.
    skin_rig = Rig(os.path.join(anims_dir, SKIN + ".dae"))
    skin = Skin(skin_rig.root)
    E = rot_z(-facing_angle(skin_rig))
    G1 = E @ skin_rig.A
    verts = skin.posed(skin_rig.world(0, E, 1.0), skin_rig.world(None, E, 1.0), G1)
    height0 = verts[:, 2].max() - verts[:, 2].min()
    k = TARGET_HEIGHT_M / height0
    print(f"skin: {height0:.2f} tall as exported -> scale {k:.3f} to {TARGET_HEIGHT_M}m; "
          f"turned {math.degrees(math.atan2(E[1, 0], E[0, 0])):.0f}deg to face +X")

    G = E @ np.diag([k, k, k, 1]) @ skin_rig.A
    rest = skin_rig.world(None, E, k)  # cached: converting walk.dae rewrites it

    meta = {"height_m": TARGET_HEIGHT_M, "skin": SKIN + ".dae", "animations": {}}
    for name in [SKIN] + [n for n in names if n != SKIN]:
        rig = skin_rig if name == SKIN else Rig(os.path.join(anims_dir, name + ".dae"))
        keep = max(rig.last_change() + 1, 2)
        root_id = rig.root_joints[0].get("id")
        times = rig.ch.times(root_id) if root_id in rig.ch.ch else np.array([0.0, 1.0])
        # Feet on the ground: lowest skinned point over the clip -> z 0.
        low = min(skin.posed(rig.world(i, E, k), rest, G)[:, 2].min()
                  for i in range(0, keep, max(1, keep // 15)))
        speed = ground_speed(rig, E, k, keep) if name in ("walk", "run") else 0.0
        rig.apply(E, k, -low, keep)
        if name == SKIN:
            # Once something sets an actor's position (walk_person.py's plugin),
            # Gazebo puts his root joint (hips) at it instead of where the
            # animation holds it, so he's spawned this high to keep his feet
            # on the ground (and each clip's root_z curve adjusts it).
            rid = rig.root_joints[0].get("id")
            hips = rig.ch.mats(rid)[:, 2, 3] if rid in rig.ch.ch else mat(rig.root_joints[0].find(C + "matrix"))[2:3, 3]
            meta["root_height_m"] = round(float(hips.mean()), 4)
            # Gazebo also replaces the hips' animated rotation with their rest
            # rotation (the skin's), so each clip's rotation relative to it is
            # saved too (root_q), for flips, rolls and the hips' sway.
            skin_rest_R = mat(rig.root_joints[0].find(C + "matrix"))[:3, :3]
        if name == SKIN:
            restore_textures(rig.root, orig, out_dir)
        rig.tree.write(os.path.join(out_dir, name + ".dae"), xml_declaration=True, encoding="utf-8")
        # Gazebo loops a clip over [0, last key time]; and since it ignores
        # the hips' animated position and rotation once the plugin positions
        # him (see root_height_m), the hips' motion is saved for
        # walk_person.py to send along: height (crouches, jumps), x/y drift
        # (in his own frame: x forward), and rotation relative to rest (flips).
        length = float(times[keep - 1]) if keep > 1 else 0.0
        rid = rig.root_joints[0].get("id")
        R = rig.ch.mats(rid) if rid in rig.ch.ch else mat(rig.root_joints[0].find(C + "matrix"))[None]
        quats = [quat(Rk[:3, :3] @ skin_rest_R.T) for Rk in R]
        for i in range(1, len(quats)):  # same hemisphere, so interpolation takes the short way
            if np.dot(quats[i], quats[i - 1]) < 0:
                quats[i] = -quats[i]
        r4 = lambda a: [round(float(v), 4) for v in a]
        meta["animations"][name] = {"file": name + ".dae", "length_s": round(length, 4),
                                    "ground_speed_mps": round(speed, 3),
                                    "root_t": r4(times[:len(R)]),
                                    "root_x": r4(R[:, 0, 3]), "root_y": r4(R[:, 1, 3]),
                                    "root_z": r4(R[:, 2, 3]),
                                    "root_q": [r4(q) for q in quats]}
        print(f"  {name}: {keep}/{len(times)} keys kept ({length:.2f}s)"
              + (f", ground speed {speed:.2f} m/s" if speed else "") + f", lifted {-low:+.3f}m")
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("wrote", out_dir)


if __name__ == "__main__":
    main()
