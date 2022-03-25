# -*- coding: utf-8 -*-
"""
Step 5: Adding a controller.
The controller will connect user actions to the simulated behaviour.
"""
import Sofa
from stlib3.scene import Scene
from tripod import Tripod
"""
The controller is implemented in this file: tripodcontroller.py
"""
from tripodcontroller import TripodController


class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        def onKeypressedEvent(self, key):
            print("Key Pressed")


def createScene(rootNode):

    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False, plugins=["SofaSparseSolver",'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid', 'SofaMiscMapping', 'SofaRigid', 'SofaGraphComponent', 'SofaBoundaryCondition', 'SofaGeneralAnimationLoop', 'SofaGeneralEngine'])
    scene.addMainHeader()
    scene.addObject('AttachBodyButtonSetting', stiffness=10)  # Set mouse spring stiffness
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = scene.Modelling.addChild(Tripod())

    scene.addObject(TripodController(name="TripodController",actuators=[tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    scene.Simulation.addChild(tripod.RigidifiedStructure)

    motors = scene.Simulation.addChild("Motors")
    for i in range(3):
        motors.addChild(tripod.getChild('ActuatedArm'+str(i)))

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.06
    scene.Simulation.addObject('MechanicalMatrixMapper',
                                 name="mmmFreeCenter",
                                 template='Vec3,Rigid3',
                                 object1="@RigidifiedStructure/DeformableParts/dofs",
                                 object2="@RigidifiedStructure/FreeCenter/dofs",
                                 nodeToParse="@RigidifiedStructure/DeformableParts/ElasticMaterialObject")

    for i in range(3):
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                   name="mmmDeformableAndArm" + str(i),
                                   template='Vec1,Vec3',
                                   object1="@Modelling/Tripod/ActuatedArm" + str(i) + "/ServoMotor/Articulation/dofs",
                                   object2="@Simulation/RigidifiedStructure/DeformableParts/dofs",
                                   skipJ2tKJ2=True,
                                   nodeToParse="@Simulation/RigidifiedStructure/DeformableParts/ElasticMaterialObject")


