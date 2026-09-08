from pathlib import Path
import sys,json,hashlib,platform,importlib.metadata as md,xml.etree.ElementTree as ET,itertools
import numpy as np
import trimesh
import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jp
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir())
sys.path.insert(0,str(ROOT/'portable_oscbf'))
from work import obb_collision_model as model
from work import dpax_collision as dc
import fcl
out={'python':sys.executable,'versions':{p:md.version(p) for p in ['jax','jaxlib','numpy','trimesh','python-fcl']},'meshes':[],'urdf':{},'cases':[]}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for i,name in enumerate(model.OBB_LINK_NAMES):
 p=ROOT/'models/ninezzhou/meshes'/f'{name}.STL'
 mesh=trimesh.load_mesh(p)
 local=(mesh.vertices-model.OBB_LOCAL_CENTERS_M[i])@model.OBB_LOCAL_ROTATIONS[i]
 excess=np.max(np.abs(local)-model.OBB_HALF_EXTENTS_M[i],axis=1)
 copy=ROOT/'portable_oscbf/urdf/meshes'/p.name
 out['meshes'].append({'link':name,'vertices':len(mesh.vertices),'max_excess_m':float(excess.max()),'outside_1um':int(np.sum(excess>1e-6)),'sha256':sha(p),'portable_same':sha(p)==sha(copy)})
for rel in ['models/ninezzhou/urdf/ninezzhou.urdf','portable_oscbf/urdf/ninezzhou.urdf']:
 p=ROOT/rel; tree=ET.parse(p)
 out['urdf'][rel]={'sha256':sha(p),'collisions':{l.attrib['name']:[ET.tostring(c,encoding='unicode') for c in l.findall('collision')] for l in tree.findall('link')},'fixed_joints':[ET.tostring(j,encoding='unicode') for j in tree.findall('joint') if j.attrib['type']=='fixed']}
active=set(map(tuple,model.OBB_COLLISION_PAIRS.tolist()))
out['omitted_nonadjacent']=[[model.OBB_LINK_NAMES[i],model.OBB_LINK_NAMES[j]] for i,j in itertools.combinations(range(10),2) if j-i>=2 and (i,j) not in active]
I=jp.eye(4); zero=jp.zeros(3); R=jp.eye(3)
fn=jax.jit(lambda t,a,b: dc._obb_pair_distance(I,zero,R,a,I,t,R,b))
for name,t,a,b in [('gap',[3.,0,0],[1,1,1],[1,1,1]),('unequal_separated',[3.1,.17,.23],[1,.7,.6],[.4,.3,.2]),('unequal_separated_robot_scale',[.31,.017,.023],[.1,.07,.06],[.04,.03,.02]),('touch',[2.,0,0],[1,1,1],[1,1,1]),('overlap',[1.5,0,0],[1,1,1],[1,1,1]),('contained',[0,0,0],[1,1,1],[.2,.2,.2]),('identical',[0,0,0],[1,1,1],[1,1,1])]:
 t,a,b=map(lambda v:np.array(v,dtype=float),(t,a,b))
 obj1=fcl.CollisionObject(fcl.Box(*(2*a)),fcl.Transform())
 obj2=fcl.CollisionObject(fcl.Box(*(2*b)),fcl.Transform(t))
 cr=fcl.CollisionResult(); fcl.collide(obj1,obj2,fcl.CollisionRequest(enable_contact=True),cr)
 dr=fcl.DistanceResult(); fd=fcl.distance(obj1,obj2,fcl.DistanceRequest(enable_nearest_points=True),dr)
 out['cases'].append({'name':name,'translation':t.tolist(),'half_a':a.tolist(),'half_b':b.tolist(),'analytic_solid_unsigned_m':float(np.linalg.norm(np.maximum(np.abs(t)-a-b,0))),'kernel_m':float(fn(t,a,b)),'fcl_unsigned_return':fd,'fcl_collision':cr.is_collision})
# Independent finite differences at separated boxes, away from contact.
t=jp.array([3.1,.17,.23]); a=jp.array([1.,.7,.6]); b=jp.array([.4,.3,.2])
g=np.array(jax.grad(fn,argnums=0)(t,a,b)); eps=1e-5
fd=np.array([(float(fn(t+jp.eye(3)[i]*eps,a,b))-float(fn(t-jp.eye(3)[i]*eps,a,b)))/(2*eps) for i in range(3)])
out['translation_derivative']={'jax':g.tolist(),'central_difference':fd.tolist(),'eps':eps,'max_error':float(np.max(np.abs(g-fd))),'analytic_gradient':[1.,0.,0.],'analytic_gradient_max_error':float(np.max(np.abs(g-np.array([1.,0.,0.]))))}
Ts=np.asarray(dc.link_transforms(jp.zeros(9)))
v6=np.asarray(trimesh.load_mesh(ROOT/'models/ninezzhou/meshes/Link6.STL').vertices)
v6world=v6@Ts[6,:3,:3].T+Ts[6,:3,3]
v6in5=(v6world-Ts[5,:3,3])@Ts[5,:3,:3]
e6=np.max(np.abs((v6in5-model.OBB_LOCAL_CENTERS_M[5])@model.OBB_LOCAL_ROTATIONS[5])-model.OBB_HALF_EXTENTS_M[5],axis=1)
out['link6_in_link5_obb_zero']={'outside_1um':int(np.sum(e6>1e-6)),'vertices':len(e6),'max_excess_m':float(e6.max())}

out['source_hashes']={str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'portable_oscbf/work/dpax_collision.py',ROOT/'portable_oscbf/work/obb_collision_model.py',Path(__file__)]}
Path(__file__).with_name('audit-results.json').write_text(json.dumps(out,indent=2,allow_nan=False))
print(json.dumps({k:v for k,v in out.items() if k not in ['urdf','source_hashes']},indent=2))
