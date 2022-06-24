import Sofa
from stlib3.scene import Scene
from actuated_finger import ActuatedFinger
from actuated_finger import FingerController
from fixing_box import FixingBox
from stlib3.physics.rigid import Sphere
from stlib3.scene.contactheader import ContactHeader


def createScene(rootNode):
    from stlib3.scene import Scene

    # Define the main architecture of the scene, with a node Modelling, Setting and Simulation
    # Define also the integration method as Euler implicit and the solver as Conjugate Gradient)
    scene = Scene(rootNode, gravity=[0.0, 0.0, -9.81],
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGeneralRigid', 'SofaRigid', 'SofaBoundaryCondition', 'SofaMeshCollision'],
                  iterative=False)
    scene.addMainHeader()
    ContactHeader(scene, alarmDistance=15e-3, contactDistance=0.5e-3, frictionCoef=0.8)

    # Setting the time step
    rootNode.dt = 0.01

    # Define the default view of the scene on SOFA
    scene.addObject('DefaultVisualManagerLoop')
    scene.VisualStyle.displayFlags = ["hideInteractionForceFields", "showForceFields",
                                      "showCollisionModels"]
    # Add a grid on the scene with squares 10mm/10mm
    rootNode.addObject("OglGrid", nbSubdiv=100, size=1)

    # Set up the pipeline for the collision computation
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10

    # Create one actuated finger
    actuatedFinger = ActuatedFinger()
    scene.Modelling.addChild(actuatedFinger)

    # Install an obstacle in the scene/object to grasp
    scene.Modelling.addChild('Obstacle')
    sphereObst = Sphere(scene.Modelling.Obstacle, translation=[30.0e-3, 0.0, 70.0e-3],
                        uniformScale=10e-3,
                        totalMass=0.032,
                        isAStaticObject=True)
    sphereObst.mass.showAxisSizeFactor = 1e-2
    sphereObst.mstate.name = 'dofs'
    # Fix the object in space
    fixSphere = FixingBox(scene.Modelling.Obstacle, sphereObst, translation=[30.0e-3, 0.0, 70.0e-3],
                          scale=[10e-3, 10e-3, 10e-3])
    scene.Modelling.Obstacle.FixingBox.BoxROI.drawBoxes = True

    # Add the simulated elements to the Simulation node
    scene.Simulation.addChild(actuatedFinger.RigidifiedStructure.DeformableParts)
    scene.Simulation.addChild(actuatedFinger.ActuatedArm)

    # Add a controller to output some performance metric during the simulation
    scene.addObject(FingerController(name='FingerController', objectDof=sphereObst.collision.MechanicalObject,
                                     actuator=scene.Modelling.ActuatedFinger.ActuatedArm, node=rootNode))

    # Temporary addition to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.12
    scene.Simulation.addObject('MechanicalMatrixMapper',
                               name="deformableAndFreeCenterCoupling",
                               template='Vec1,Vec3',
                               object1=actuatedFinger.ActuatedArm.ServoMotor.Articulation.dofs.getLinkPath(),
                               object2=actuatedFinger.RigidifiedStructure.DeformableParts.dofs.getLinkPath(),
                               nodeToParse=actuatedFinger.RigidifiedStructure.DeformableParts.ElasticMaterialObject.getLinkPath())

    return rootNode
