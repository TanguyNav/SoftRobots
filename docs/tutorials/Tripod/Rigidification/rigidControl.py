import Sofa
from rigidification import *
from math import sin,cos
import os
path2 = os.path.dirname(os.path.abspath(__file__))+'/'

class rigidControl(Sofa.PythonScriptController):

   ### Overloaded PythonScriptController callbacks
    def __init__(self, controllerNode, name, listRigid, forceFields, mecaObjet, nodeFEM, TransformBaseFrame, inverse):
        # note: this member aliases the sofa component `name` data, so you can't
        # put anything in here (in this case, strings only)
        self.name=name
        self.listRigid = listRigid
        self.forceFields = forceFields
        self.bwdInitcalled=0  
        self.mecaObjet = mecaObjet
        self.nodeFEM = nodeFEM
        self.O_H_Frames= TransformBaseFrame
        self.inverse= inverse
        
        
        if len(TransformBaseFrame)!=len(listRigid):
            print '********* WARNING : number of Transforms is not good' 


        
        ParentsNode=controllerNode.getParents()
        self.node=ParentsNode[0]
        
    def initGraph(self,controllerNode):                
        testNode = self.node.createChild('Bug_Todo_SuppressThisNode')
        
#    def bwdInitGraph(self,node):
        if (self.bwdInitcalled):
            return        ##the function has already been called (don't execute two times)
        
        self.bwdInitcalled = 1;
        
        numRigids = len(self.listRigid)
        numForceFields = len(self.forceFields)
        
        
        ###########
        ## - compute the indices pairs of the multimapping
        ###########
        self.numNodes = len(self.mecaObjet.position)
        self.indexPairs = fillIndexPairs_oneRigid(self.numNodes, self.listRigid)
        
        
        
        ###########
        ##---- creation of the rigid node ----
        ##########
        nodeParent = self.node.getParents()
        self.rigidNode = self.node.createChild('Rigid')
        
        ##for each rigid, compute the center of gravity

        num_points= 0;
        pos_rigid=[0]*numRigids
        posG_list=[0]*numRigids
        for r in range(numRigids):
            it=0;
            listN = self.listRigid[r];
            num_points = num_points+len(listN)        
            pos = [0]*len(listN)
                        
            for i in listN:
                pos[it] =  self.mecaObjet.position[i] 
                it = it+1

            posG = gravityCenter(pos)
            pos_rigid[r] = [self.O_H_Frames[r][0], self.O_H_Frames[r][1], self.O_H_Frames[r][2], self.O_H_Frames[r][3],self.O_H_Frames[r][4],self.O_H_Frames[r][5],self.O_H_Frames[r][6]]
            if (r==3):
                pos_rigid[r][0] = posG[0];
                pos_rigid[r][1] = posG[1];
                pos_rigid[r][2] = posG[2];
                
            
            posG_list[r] = [posG[0], posG[1], posG[2]]
            
        # creation of the MechanicalObject Rigid
        rigidMO=self.rigidNode.createObject('PointSetTopologyContainer', position=posG_list)
        rigidMO= self.rigidNode.createObject('MechanicalObject', template='Rigid',name='RigidFrames', position=pos_rigid, showObject='1', showObjectScale='10')
        
        # INVERSE
        PI=3.14159265359;
        if self.inverse:
            self.rigidNode.createObject('SlidingActuator', direction='0 0 0 1 0 0', indices='0',  maxPositiveDisp='0.5', maxNegativeDisp='0.5', maxDispVariation='0.05')
            self.rigidNode.createObject('SlidingActuator', direction=[0 ,0 ,0 ,cos(2*PI/3),-sin(2*PI/3) ,0], indices='1',  maxPositiveDisp='0.5', maxNegativeDisp='0.5', maxDispVariation='0.05')
            self.rigidNode.createObject('SlidingActuator', direction=[0 ,0 ,0 ,cos(4*PI/3),-sin(4*PI/3) ,0], indices='2',  maxPositiveDisp='0.5', maxNegativeDisp='0.5', maxDispVariation='0.05')
            #rigidM0=self.rigidNode.createObject('SlidingActuator', direction='0 1 0 0 0 0', indices='0')
            #rigidM0=self.rigidNode.createObject('SlidingActuator', direction='1 0 0 0 0 0', indices='0')
        else:
            self.rigidNode.createObject('PythonScriptController', filename=path2+'TripodController.py', classname='controller')
        
        rigidMO=self.rigidNode.createObject('RestShapeSpringsForceField', template='Rigid', points=[0], angularStiffness='1e6', stiffness='1e8')
        rigidMO=self.rigidNode.createObject('RestShapeSpringsForceField', template='Rigid', points=[1,2], angularStiffness='1e6', stiffness='1e8')
                
        ##########
        # mapping of effector point
        ##########
        self.effectorNode  = self.rigidNode.createChild('Effector')
        self.effectorNode.createObject('MechanicalObject', template='Vec3d', name="effector", position='0 0 10')
        self.effectorNode.createObject('RigidMapping',rigidIndexPerPoint='3')
        if self.inverse:
            self.effectorNode.createObject('PositionEffector', template='Vec3d', indices="0", effectorGoal="@../../../goal/goalMO.position", useDirections='1 1 1')        
        
        ###########
        # ---- mapping of the rigidifiedNodes
        ###########
        self.rigidifiedNode = self.rigidNode.createChild('Rigidified')
        
        ## position pour le MecaObject
        pos_rigidified = [0] * num_points
        it=0;
        rigidIndexPerPoint=[0]*num_points
        for r in range(numRigids):
            for i in self.listRigid[r]:
                pos_rigidified[it] = self.mecaObjet.position[i]
                rigidIndexPerPoint[it]=r;
                it = it+1
   
        self.rigidifiedNode.createObject('PointSetTopologyContainer', position=pos_rigidified)     
        self.rigidifiedMO= self.rigidifiedNode.createObject('MechanicalObject', template='Vec3d',name='RigidifiedNodes', position=pos_rigidified)
        rigidMap = self.rigidifiedNode.createObject('RigidMapping', name="rigidMap", input='@..',output='@.', globalToLocalCoords='true', rigidIndexPerPoint=rigidIndexPerPoint)
            


    

        
        ###########
        # ---- creation of the "free Nodes"
        ###########        
    
        self.freeNodes = self.node.createChild('FreeNodes')
        
        #find position
        pos_node_free = [ ];
        for i in range(self.numNodes):
            if self.indexPairs[2*i]==0:
                pos_node_free = pos_node_free + [self.mecaObjet.position[i]]
        
        #create a point topology and the mechanicalObject
        self.freeNodes.createObject('PointSetTopologyContainer', position=pos_node_free)        
        self.freeMO= self.freeNodes.createObject('MechanicalObject', template='Vec3d',name='FreeNodes', position=pos_node_free)
        

        
        
        
        ###########
        # ---- creation of the (multi-) mapped object
        ###########    
        # node and multi-mapped    
        
        ## remove the FEM node from its initial node and put it as a child
        self.node.removeChild(self.nodeFEM)
        self.freeNodes.addChild(self.nodeFEM)
        self.rigidifiedNode.addChild(self.nodeFEM)
        
        # create inputPath
        pathIn = '@../../..'+self.freeMO.getPathName() + ' ' + '@../../..'+self.rigidifiedMO.getPathName()
        # create the mapping 
        self.nodeFEM.createObject('SubsetMultiMapping', template='Vec3d,Vec3d', name='model_mapping', input=pathIn, output='@.', indexPairs= self.indexPairs)
        
        
        ###########
        # ---- creation of the mappedMatrixForcefield
        ###########    
        
        #self.node.createObject('RequiredPlugin', name='ModelOrderReduction', pluginName='ModelOrderReduction')
        
        
        test = self.node.createObject('MappedMatrixForceFieldAndMass', template='Vec3d,Rigid', object1='@./FreeNodes/FreeNodes', object2='@./Rigid/RigidFrames', mappedForceField='@'+self.forceFields[0].getPathName(), mappedMass='@'+self.forceFields[1].getPathName())
        
        test.init();

        
        print '-------------'       
