def EffectorGoal(attachedTo=None,
    name="Goal",
    position=None,
    template="Vec3d",
    translation=[0.0,0.0,0.0],
    rotation=[0.0,0.0,0.0],
    uniformScale=1.0,
    visuScale=0.1,
    ):

    """Creates and adds a effector goal. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to the given desired position.
    See examples in SoftRobots/docs/tutorials

    Args:
        name (str): Name of the effector goal.

        position: Location of the target position(s) of the effector(s).

        template:

        translation (vec3f):   Apply a 3D translation to the object.

        rotation (vec3f):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3f):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "Goal"
                    EulerImplicit,
                    CGLinearSolver,
                    MechanicalObject
            }

    """
    #  This create a new node in the scene. This node should be appended to the root node.
    goal = attachedTo.createChild(name)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a effector it is a set of positions specifying
    # ghe location of the effector
    goal.createObject('EulerImplicit', firstOrder="1")
    goal.createObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5)
    goal.createObject('MechanicalObject', template=template, position=position,
                        rotation=rotation, translation=translation, scale=uniformScale,
                        showObject="1", showObjectScale=visuScale, drawMode="1", showColor="255 255 255 255")

    return goal
