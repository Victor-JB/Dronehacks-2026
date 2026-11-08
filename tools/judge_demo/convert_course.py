"""STEP -> surface meshes for SENSOR/RENDERER ONLY, never planner input."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import gmsh
import numpy as np
import trimesh


def convert(source,out):
    out.mkdir(parents=True,exist_ok=True)
    gmsh.initialize()
    try:
        gmsh.option.setString('Geometry.OCCTargetUnit','M')
        gmsh.model.occ.importShapes(str(source)); gmsh.model.occ.synchronize()
        gmsh.option.setNumber('Mesh.MeshSizeMax',.13)
        gmsh.option.setNumber('Mesh.MeshSizeMin',.025)
        gmsh.model.mesh.generate(2)
        tags,coordinates,_=gmsh.model.mesh.getNodes()
        vertices=np.array(coordinates).reshape(-1,3)
        mapping={int(t):i for i,t in enumerate(tags)}
        groups=[]; entities=gmsh.model.getEntities(3)
        if not entities: entities=gmsh.model.getEntities(2)
        for index,(dim,tag) in enumerate(entities):
            name=gmsh.model.getEntityName(dim,tag) or f'part_{tag}'
            surfaces=[tag] if dim==2 else [t for d,t in gmsh.model.getBoundary([(dim,tag)],oriented=False) if d==2]
            faces=[]
            for surface in surfaces:
                types,_,nodes=gmsh.model.mesh.getElements(2,surface)
                for kind,values in zip(types,nodes):
                    if kind==2:
                        faces.extend([[mapping[int(t)] for t in tri] for tri in np.array(values).reshape(-1,3)])
            if not faces: continue
            mesh=trimesh.Trimesh(vertices=vertices,faces=np.array(faces),process=False)
            mesh.remove_unreferenced_vertices()
            filename=f'{index:02d}_'+re.sub(r'[^a-z0-9]+','_',name.lower())[-70:]+'.ply'
            mesh.export(out/filename)
            groups.append(dict(name=name,file=filename,bounds=mesh.bounds.tolist(),triangles=len(mesh.faces),color=gmsh.model.getColor(dim,tag)))
        metadata=dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            units='metres via Geometry.OCCTargetUnit=M',groups=groups,
            simulator_geometry_only=True,gmsh_version=gmsh.__version__)
        (out/'course.json').write_text(json.dumps(metadata,indent=2))
        print(json.dumps(metadata,indent=2))
    finally: gmsh.finalize()


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--step',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();convert(a.step,a.out)
