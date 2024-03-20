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
N = 4 # order of Sierpinski carpet
a = 0.10000 # edge length of starting square; give sufficient zeros to maintain accuracy
d = 0.30000 # Length of air cavity
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
mdb.models['Model-1'].parts['AirInsideCarpet'].BaseSolidExtrude(depth=d, 
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
mdb.models['Model-1'].parts['AirInsideCarpet'].PartitionFaceBySketch(faces=
    mdb.models['Model-1'].parts['AirInsideCarpet'].faces[face_index], sketch=mdb.models['Model-1'].sketches['__profile__'], sketchUpEdge=
    mdb.models['Model-1'].parts['AirInsideCarpet'].edges[edge_index])
del mdb.models['Model-1'].sketches['__profile__']
### Materials, Section assignments and meshes
mdb.models['Model-1'].Material(name='Air')
mdb.models['Model-1'].materials['Air'].Density(table=((1.225, ), ))
mdb.models['Model-1'].materials['Air'].AcousticMedium(acousticVolumetricDrag=
    OFF, bulkTable=((142000.000, ), ))
mdb.models['Model-1'].HomogeneousSolidSection(material='Air', name='AirSection'
    , thickness=None)
mdb.models['Model-1'].parts['AirInsideCarpet'].Set(cells=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), name='Set-1')
mdb.models['Model-1'].parts['AirInsideCarpet'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=
    mdb.models['Model-1'].parts['AirInsideCarpet'].sets['Set-1'], sectionName=
    'AirSection', thicknessAssignment=FROM_SECTION)
mdb.models['Model-1'].parts['AirInsideCarpet'].setMeshControls(elemShape=TET, 
    regions=
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), technique=FREE)
mdb.models['Model-1'].parts['AirInsideCarpet'].setElementType(elemTypes=(
    ElemType(elemCode=C3D20R, elemLibrary=STANDARD), ElemType(elemCode=C3D15, 
    elemLibrary=STANDARD), ElemType(elemCode=C3D10, elemLibrary=STANDARD)), 
    regions=(
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), ))
mdb.models['Model-1'].parts['AirInsideCarpet'].setElementType(elemTypes=(
    ElemType(elemCode=AC3D20, elemLibrary=STANDARD), ElemType(elemCode=AC3D15, 
    elemLibrary=STANDARD), ElemType(elemCode=AC3D10, elemLibrary=STANDARD)), 
    regions=(
    mdb.models['Model-1'].parts['AirInsideCarpet'].cells.getSequenceFromMask((
    '[#1 ]', ), ), ))
### Meshing (in part module)
### meshing holes of carpet
no_of_edges = len(mdb.models['Model-1'].parts['AirInsideCarpet'].edges)
tol = 1e-10
for n in range(1,N+1):
    L_edge = a/(3**n)
    for i in range(0,no_of_edges):
        if abs(mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i].getSize()-L_edge) <= tol:
            mdb.models['Model-1'].parts['AirInsideCarpet'].seedEdgeBySize(constraint=FINER, deviationFactor=0.1, edges=mdb.models['Model-1'].parts['AirInsideCarpet'].edges[i:i+1], size=L_edge/7)
mdb.models['Model-1'].parts['AirInsideCarpet'].seedPart(deviationFactor=0.1, 
    minSizeFactor=0.1, size=d/20)
mdb.models['Model-1'].parts['AirInsideCarpet'].generateMesh()
### imposing pressure BC (in Assembly module)
mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name=
    'AirInsideCarpet-1', part=mdb.models['Model-1'].parts['AirInsideCarpet'])
no_of_faces = len(mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces)
### Assigning each hole to a set
counter = 1
for n in range(1,N+1):
    L_edge = a/(3**n)
    for i in range(0,no_of_faces):
        if abs(mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces[i].getSize()-L_edge*L_edge) <= tol:
            mdb.models['Model-1'].rootAssembly.Set(faces=mdb.models['Model-1'].rootAssembly.instances['AirInsideCarpet-1'].faces[i:i+1], name='Set-hole-{co}'.format(co=counter))
            counter = counter+1
### Making a big set of all square holes
mdb.models['Model-1'].rootAssembly.Set(name='Set-hole-1A', objectToCopy=mdb.models['Model-1'].rootAssembly.sets['Set-hole-1'])
for i in range(1,(8**N-1)//7):
    mdb.models['Model-1'].rootAssembly.SetByBoolean(name='Set-hole-{I}A'.format(I=i+1), sets=(mdb.models['Model-1'].rootAssembly.sets['Set-hole-{I}'.format(I=i+1)],mdb.models['Model-1'].rootAssembly.sets['Set-hole-{I}A'.format(I=i)]))
### Creating eigenfrequency analysis step
mdb.models['Model-1'].FrequencyStep(maxEigen=5000.000, minEigen=0.000, name='EigenFreqStep', previous='Initial')
mdb.models['Model-1'].AcousticPressureBC(createStepName='EigenFreqStep', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, magnitude=0.0, name=
    'PressureReleaseBC', region=mdb.models['Model-1'].rootAssembly.sets['Set-hole-{M}A'.format(M=(8**N-1)//7)])
mdb.models['Model-1'].FieldOutputRequest(createStepName='EigenFreqStep', name=
    'F-Output-1', variables=('POR', ))
### Creating Job
mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
    explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
    memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
    multiprocessingMode=DEFAULT, name='Job-N{Ni}-{di}-EigenFrequencies'.format(Ni=N,di=int(d*1000)), nodalOutputPrecision=SINGLE
    , numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='', type=
    ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)