#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 31 11:35:15 2022

@author: tanguy
"""

import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaConstraint SofaDeformable SofaEngine SofaImplicitOdeSolver SofaLoader SofaOpenglVisual SofaPreconditioner SofaSimpleFem SofaSparseSolver SofaTopologyMapping')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', printLog='0')

    ################################################################################################################
    ################################################### Bunny ######################################################
    ################################################################################################################
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver', name='odesolver')
    bunny.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond=True, update_step='1')

    bunny.addObject('MeshVTKLoader', name='loader', filename=path+'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.addObject('TetrahedronSetTopologyModifier')

    bunny.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
    bunny.addObject('UniformMass', totalMass='0.5')
    bunny.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio='0.3',  youngModulus='10000')
    

    bunny.addObject('BoxROI', name='boxROI', box=[-5, -5.0, -5,  5, -4.5, 5], drawBoxes=True)
    bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')

    bunny.addObject('SparseLDLSolver', name='preconditioner')
    bunny.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')
    
    
#    constraints = bunny.addChild('Constraints')
#    constraints.addObject('MechanicalObject', name = "PulledPointSphericalMO", position = [-4.5, -2.54, 0.0])
#    constraints.addObject('CableConstraint', template='Vec3', name='CableSphericalSurface', pullPoint= [-4.4, -10, 3.1], value = 1.5, 
#                         indices='0', drawPoints = True)
#    constraints.addObject("BarycentricMapping")

	################################################################################################################
    ################################################ Constraints ###################################################
    ################################################################################################################    
    constraints = bunny.addChild('Constraints')
    constraints.addObject('MeshSTLLoader', name='loaderSufaceCable', filename=path+'Bunny.stl')
    constraints.addObject('MeshTopology', src='@loaderSufaceCable', name='topoSufaceCable')
    constraints.addObject('MechanicalObject')
    
    constraints.addObject('CableConstraint', template='Vec3', name='CableSphericalSurface', pullPoint= [-4.4, -10, 3.1], minForce=0, 
                         value = 1, centers = [[-4.5, -2.54, 0.0]],
                         surfaceTopology = "@topoSufaceCable",
                         method="geodesic", radii = [1.0], # 0.8
                         drawPoints = True, drawPulledAreas = True) 
    
    constraints.addObject('BarycentricMapping')    
    

	################################################################################################################
    ############################################### Visualization ##################################################
    ################################################################################################################
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping')
    bunnyVisu.addObject('OglModel', template='Vec3d', color='0.7 0.4 0.4 1')
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
