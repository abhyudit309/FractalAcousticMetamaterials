# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
### Sierpinski Carpet Code
import numpy as np
N = 3 # order of Sierpinski carpet
a = 0.10000 # edge length of starting square; give sufficient zeros to maintain accuracy
d1 = 0.790000 # distance between source and patterned face 
d2 = 0.010000 # thickness of the specimen
d3 = 0.10000 # thickness of air cavity
d4 = 0.10000 # thickness of air cavity
FCC = np.array([[0,0]]) # final center coordinates
Edge_lengths = np.empty(((8**N-1)//7,1)) # edge lengths of square holes
for n in range(1,N+1):
    L = a/(3**n)
    # all possible (x,y) coordinates of the centres
    centres = np.linspace(3*L/2,a-3*L/2,3**(n-1))
    # generating them as (x,y) pairs
    origins = np.zeros((9**(n-1),2))
    c = 0
    for i in range(0,len(centres)):
        for j in range(0,len(centres)):
            origins[c,:] = np.array([centres[i], centres[j]])
            c = c+1 
    # removing centres/origins in pre-existing holes
    row_delete = []
    if (n>=2):
        for k in range(0,origins.shape[0]):
            for m in range(1,n):
                for t in range((8**(m-1)+6)//7,(8**(m)-1)//7+1):
                    if abs(origins[k,0]-FCC[t-1,0]) < a/(2*3**m) and abs(origins[k,1]-FCC[t-1,1]) < a/(2*3**m):
                        row_delete.append(k)            
    origins = np.delete(origins,row_delete,0)
    FCC = np.concatenate((FCC,origins),axis=0)    
    if n==1:
        FCC = np.delete(FCC,0,0)
    Edge_lengths[(8**(n-1)+6)//7-1:(8**(n)-1)//7] = L 
data = np.concatenate((FCC,Edge_lengths),axis=1)
###
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=5.0)
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(-a/2, -a/2), 
    point2=(a/2, a/2))
mdb.models['Model-1'].Part(dimensionality=THREE_D, name='AirInsideCarpet', 
    type=DEFORMABLE_BODY)
mdb.models['Model-1'].parts['AirInsideCarpet'].BaseSolidExtrude(depth=d1, 
    sketch=mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0.0,0.0,0.0):
        face_index = i
        edge_index = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getEdges()[0]
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=7.07, name='__profile__', 
    sheetSize=282.84, transform=
    mdb.models['Model-1'].parts['AirInsideCarpet'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index], 
    sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0)))
mdb.models['Model-1'].parts['AirInsideCarpet'].projectReferencesOntoSketch(
    filter=COPLANAR_EDGES, sketch=
    mdb.models['Model-1'].sketches['__profile__'])
for i in range(0,data.shape[0]):
    mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(data[i,0]-0.5*data[i,2]-0.5*a, data[i,1]-0.5*data[i,2]-0.5*a), point2=(data[i,0]+0.5*data[i,2]-0.5*a, data[i,1]+0.5*data[i,2]-0.5*a))
mdb.models['Model-1'].parts['AirInsideCarpet'].SolidExtrude(depth=d2, 
    flipExtrudeDirection=OFF, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index])
del mdb.models['Model-1'].sketches['__profile__']
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0.0,0.0,-d2):
        face_index = i
        edge_index = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getEdges()[0]
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=7.07, name='__profile__', 
    sheetSize=282.84, transform=
    mdb.models['Model-1'].parts['AirInsideCarpet'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index], 
    sketchOrientation=RIGHT, origin=(0.0, 0.0, -d2)))
mdb.models['Model-1'].parts['AirInsideCarpet'].projectReferencesOntoSketch(
    filter=COPLANAR_EDGES, sketch=
    mdb.models['Model-1'].sketches['__profile__'])
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(-a/2, -a/2), point2=(a/2, a/2))
mdb.models['Model-1'].parts['AirInsideCarpet'].SolidExtrude(depth=d3, 
    flipExtrudeDirection=OFF, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index])
del mdb.models['Model-1'].sketches['__profile__']
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0.0,0.0,round(-d2-d3,5)):
        face_index = i
        edge_index = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getEdges()[0]
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=7.07, name='__profile__', 
    sheetSize=282.84, transform=
    mdb.models['Model-1'].parts['AirInsideCarpet'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index], 
    sketchOrientation=RIGHT, origin=(0.0, 0.0, round(-d2-d3,5))))
mdb.models['Model-1'].parts['AirInsideCarpet'].projectReferencesOntoSketch(
    filter=COPLANAR_EDGES, sketch=
    mdb.models['Model-1'].sketches['__profile__'])
for i in range(0,data.shape[0]):
    mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(data[i,0]-0.5*data[i,2]-0.5*a, data[i,1]-0.5*data[i,2]-0.5*a), point2=(data[i,0]+0.5*data[i,2]-0.5*a, data[i,1]+0.5*data[i,2]-0.5*a))
mdb.models['Model-1'].parts['AirInsideCarpet'].SolidExtrude(depth=d2, 
    flipExtrudeDirection=OFF, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index])
del mdb.models['Model-1'].sketches['__profile__']  
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0.0,0.0,round(-2*d2-d3,5)):
        face_index = i
        edge_index = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getEdges()[0]
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=7.07, name='__profile__', 
    sheetSize=282.84, transform=
    mdb.models['Model-1'].parts['AirInsideCarpet'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index], 
    sketchOrientation=RIGHT, origin=(0.0, 0.0, round(-2*d2-d3,5))))
mdb.models['Model-1'].parts['AirInsideCarpet'].projectReferencesOntoSketch(
    filter=COPLANAR_EDGES, sketch=
    mdb.models['Model-1'].sketches['__profile__'])
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(-a/2, -a/2), point2=(a/2, a/2))
mdb.models['Model-1'].parts['AirInsideCarpet'].SolidExtrude(depth=d4, 
    flipExtrudeDirection=OFF, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index])
del mdb.models['Model-1'].sketches['__profile__']
### making partitions
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0,0,0):
        v1 = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getVertices()[0:3]        
mdb.models['Model-1'].parts['AirInsideCarpet'].PartitionCellByPlaneThreePoints(
    cells=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), point1=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v1[0]], point2=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v1[1]], point3=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v1[2]])
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0,0,-d2):
        v2 = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getVertices()[0:3]   
mdb.models['Model-1'].parts['AirInsideCarpet'].PartitionCellByPlaneThreePoints(
    cells=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), point1=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v2[0]], point2=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v2[1]], point3=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v2[2]])
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0,0,round(-d2-d3,5)):
        v3 = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getVertices()[0:3]   
mdb.models['Model-1'].parts['AirInsideCarpet'].PartitionCellByPlaneThreePoints(
    cells=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), point1=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v3[0]], point2=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v3[1]], point3=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v3[2]])
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getCentroid()[0] == (0,0,round(-2*d2-d3,5)):
        v4 = mdb.models['Model-1'].parts['AirInsideCarpet'].faces[i].getVertices()[0:3]   
mdb.models['Model-1'].parts['AirInsideCarpet'].PartitionCellByPlaneThreePoints(
    cells=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), point1=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v4[0]], point2=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v4[1]], point3=
    mdb.models['Model-1'].parts['AirInsideCarpet'].vertices[v4[2]])
### making sets 
tol = 1e-10
no_of_cells = len(mdb.models['Model-1'].parts['AirInsideCarpet'].cells)
normal_cells = []
for i in range (0,no_of_cells):
    if abs(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i].getSize() - a*a*d1) <= tol or abs(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i].getSize() - a*a*d3) <= tol or abs(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i].getSize() - a*a*d4) <= tol:
        normal_cells.append(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i])
mdb.models['Model-1'].parts['AirInsideCarpet'].Set(name='Set-normal', cells=part.CellArray(normal_cells)) 
all_hole_cells = []   
for n in range(1,N+1):
    L = a/(3**n)
    hole_cells = []
    for i in range(0,no_of_cells):
        if abs(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i].getSize() - L*L*d2) <= tol:
            hole_cells.append(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i])
            all_hole_cells.append(mdb.models['Model-1'].parts['AirInsideCarpet'].cells[i])
            mdb.models['Model-1'].parts['AirInsideCarpet'].Set(name='Set-hole-{n1}'.format(n1=n), cells=part.CellArray(hole_cells))
mdb.models['Model-1'].parts['AirInsideCarpet'].Set(name='Set-All-holes', cells=part.CellArray(all_hole_cells)) 
mdb.models['Model-1'].parts['AirInsideCarpet'].SetByBoolean(name='Full-Set', 
    sets=(mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Set-All-holes'], 
    mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Set-normal']))
### Materials and Section assignments 
B = 142000.000 # Bulk modulus
rho = 1.225 # density
for n in range(1,N+1):
    mdb.models['Model-1'].Material(name='Air-hole-{n1}'.format(n1=n))
    mdb.models['Model-1'].materials['Air-hole-{n1}'.format(n1=n)].Density(table=((rho, ), ))
    mdb.models['Model-1'].materials['Air-hole-{n1}'.format(n1=n)].AcousticMedium(acousticVolumetricDrag=OFF, bulkTable=((B, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(material='Air-hole-{n1}'.format(n1=n), name='AirSection-hole-{n1}'.format(n1=n), thickness=None)
    mdb.models['Model-1'].parts['AirInsideCarpet'].SectionAssignment(offset=0.0,offsetField='', offsetType=MIDDLE_SURFACE, region=mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Set-hole-{n1}'.format(n1=n)],sectionName='AirSection-hole-{n1}'.format(n1=n), thicknessAssignment=FROM_SECTION)
mdb.models['Model-1'].Material(name='Air-normal')
mdb.models['Model-1'].materials['Air-normal'].Density(table=((rho, ), ))
mdb.models['Model-1'].materials['Air-normal'].AcousticMedium(acousticVolumetricDrag=
    OFF, bulkTable=((B, ), ))
mdb.models['Model-1'].HomogeneousSolidSection(material='Air-normal', name='AirSection-normal', 
    thickness=None)
mdb.models['Model-1'].parts['AirInsideCarpet'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=
    mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Set-normal'], 
    sectionName='AirSection-normal', thicknessAssignment=FROM_SECTION)
### Meshing
mdb.models['Model-1'].parts['AirInsideCarpet'].setMeshControls(elemShape=TET, 
    regions=part.CellArray(all_hole_cells), technique=FREE)
mdb.models['Model-1'].parts['AirInsideCarpet'].setMeshControls(elemShape=TET, 
    regions=part.CellArray(normal_cells), technique=FREE)
mdb.models['Model-1'].parts['AirInsideCarpet'].setElementType(elemTypes=(
    ElemType(elemCode=C3D20R, elemLibrary=STANDARD), ElemType(elemCode=C3D15, 
    elemLibrary=STANDARD), ElemType(elemCode=C3D10, elemLibrary=STANDARD)), 
    regions=mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Full-Set'])
mdb.models['Model-1'].parts['AirInsideCarpet'].setElementType(elemTypes=(
    ElemType(elemCode=AC3D20, elemLibrary=STANDARD), ElemType(elemCode=AC3D15, 
    elemLibrary=STANDARD), ElemType(elemCode=AC3D10, elemLibrary=STANDARD)), 
    regions=mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Full-Set'])
### Meshing (in part module)
### meshing holes of carpet
no_of_faces = len(mdb.models['Model-1'].parts['AirInsideCarpet'].faces)
no_of_edges = len(mdb.models['Model-1'].parts['AirInsideCarpet'].edges)
for n in range(1,N+1):
    L_edge = a/(3**n)
    for i in range(0,no_of_edges):
        if abs(mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i].getSize()-L_edge) <= tol:
            mdb.models['Model-1'].parts['AirInsideCarpet'].seedEdgeBySize(constraint=FINER, deviationFactor=0.1, edges=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i:i+1], size=L_edge/7)
# meshing depth of carpet
for i in range(0,no_of_edges):
    if abs(mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i].getSize()-d2) <= tol:
        mdb.models['Model-1'].parts['AirInsideCarpet'].seedEdgeBySize(constraint=FINER, deviationFactor=0.1, edges=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i:i+1], size=d2/7)  
mdb.models['Model-1'].parts['AirInsideCarpet'].seedPart(deviationFactor=0.1, 
    minSizeFactor=0.1, size=0.015)
mdb.models['Model-1'].parts['AirInsideCarpet'].generateMesh()
### importing part in Assembly module
mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name=
    'AirInsideCarpet-1', part=mdb.models['Model-1'].parts['AirInsideCarpet']) 
### Creating steady state dynamic step
f1 = 100.000
f2 = 1700.000
mdb.models['Model-1'].SteadyStateDirectStep(frequencyRange=((f1, f2, 33, 
    1.0), ), name='InwardVolAccelStep', previous='Initial', scale=LINEAR)
mdb.models['Model-1'].TabularAmplitude(data=((f1, 1.0), (f2, 1.0)), name=
    'Amp-1', smooth=SOLVER_DEFAULT, timeSpan=STEP)  
mdb.models['Model-1'].FieldOutputRequest(createStepName='InwardVolAccelStep', 
    name='F-Output-1', variables=('POR', 'ACV'))    
no_of_faces = len(mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces) 
for i in range(0,no_of_faces):
    if mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces[i].getCentroid()[0] == (0,0,d1):
        area_face = mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces[i].getSize()
        mdb.models['Model-1'].rootAssembly.Set(name='Set-nodes', nodes=mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces[i].getNodes())
        mdb.models['Model-1'].InwardVolAccel(amplitude='Amp-1', createStepName='InwardVolAccelStep',magnitude=(area_face+0j), name='Load', region=mdb.models['Model-1'].rootAssembly.sets['Set-nodes'])
#mdb.models['Model-1'].setValues(waveFormulation=SCATTERED)
allNodes = mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].nodes
node1 = allNodes.getClosest((0,0,0.15),1)
node2 = allNodes.getClosest((0,0,0.2),1)
mdb.models['Model-1'].rootAssembly.Set(name='Set-node1', nodes=MeshNodeArray([node1]))
mdb.models['Model-1'].rootAssembly.Set(name='Set-node2', nodes=MeshNodeArray([node2]))
mdb.models['Model-1'].HistoryOutputRequest(createStepName='InwardVolAccelStep', 
    name='H-Output-1', rebar=EXCLUDE, region=mdb.models['Model-1'].rootAssembly.sets['Set-node1'], sectionPoints=DEFAULT, variables=('POR', 'ACV'))   
mdb.models['Model-1'].HistoryOutputRequest(createStepName='InwardVolAccelStep', 
    name='H-Output-2', rebar=EXCLUDE, region=mdb.models['Model-1'].rootAssembly.sets['Set-node2'], sectionPoints=DEFAULT, variables=('POR', 'ACV'))    
### Creating Job
mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
    explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
    memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
    multiprocessingMode=DEFAULT, name='Job-N{Ni}-{di}-TwoSpecimens'.format(Ni=N,di=int(d3*1000)), nodalOutputPrecision=SINGLE
    , numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='', type=
    ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
# end       