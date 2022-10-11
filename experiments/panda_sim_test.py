import time
import numpy as np
import math
import pybullet as pb

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 8 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
# restposes for null space
jointPositions=(0.8045609285966308, 0.525471701354679, -0.02519566900946519, -1.3925086098003587, 0.013443782914225877, 1.9178323512245277, -0.007207024243406651, 0.01999436579245478, 0.019977024051412193)
            # [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions

class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.p = bullet_client
        self.p.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)
        
        flags = self.p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        orn=[0, 0, 0, 1]
        self.panda = self.p.loadURDF("setup/robots/franka_panda/panda_1.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
        index = 0
        self.state = 0
        self.control_dt = 1./240.
        self.finger_target = 0.04
        self.gripper_height = 0.2
        #create a constraint to keep the fingers centered
        c = self.p.createConstraint(self.panda,
                          9,
                          self.panda,
                          10,
                          jointType=self.p.JOINT_GEAR,
                          jointAxis=[1, 0, 0],
                          parentFramePosition=[0, 0, 0],
                          childFramePosition=[0, 0, 0])
        self.p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
    
        for j in range(self.p.getNumJoints(self.panda)):
            self.p.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = self.p.getJointInfo(self.panda, j)
            #print("info=",info)
            jointType = info[2]
            ## Joint that moves linearly 
            if (jointType == self.p.JOINT_PRISMATIC):
                self.p.resetJointState(self.panda, j, jointPositions[index]) 
                index=index+1
            ## Joint that revolves
            if (jointType == self.p.JOINT_REVOLUTE):
                self.p.resetJointState(self.panda, j, jointPositions[index]) 
                index=index+1
        self.t = 0.
        self.armNames = [
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

        self.armJoints = self.get_id_by_name(self.armNames)
        self.armControlID = self.get_control_id_by_name(self.armNames)

        self.gripperNames = [
            "guide_joint_finger_left", #9
            "guide_joint_finger_right", #10
        ]
        self.gripperJoints = self.get_id_by_name(self.gripperNames)
        self.gripperControlID = self.get_control_id_by_name(self.gripperNames)
        pb.enableJointForceTorqueSensor(self.panda, self.gripperJoints[0])
        pb.enableJointForceTorqueSensor(self.panda, self.gripperJoints[1])

    def get_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        nbJoint = pb.getNumJoints(self.panda)
        jointNames = {}
        for i in range(nbJoint):
            name = pb.getJointInfo(self.panda, i)[1].decode()
            jointNames[name] = i

        return [jointNames[name] for name in names]

    def get_control_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        nbJoint = pb.getNumJoints(self.panda)
        jointNames = {}
        ctlID = 0
        for i in range(nbJoint):
            jointInfo = pb.getJointInfo(self.panda, i)
            name = jointInfo[1].decode("utf-8")

            # skip fixed joint
            if jointInfo[2] == 4:
                continue

            # skip base joint
            if jointInfo[-1] == -1:
                continue
            jointNames[name] = ctlID
            ctlID += 1

        return [jointNames[name] for name in names]

    def go(self, pos, ori=None, width=None, wait=False, gripForce=20):
        if ori is None:
            ori = self.ori

        if width is None:
            width = self.width

        ori_q = pb.getQuaternionFromEuler(ori)
        ## Calculate target position of each joint
        jointPose = self.p.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, ori_q, ll, ul, jr, rp, maxNumIterations=20)
        jointPose = np.array(jointPose)

        ## Set position for each joint
        for i in range(pandaNumDofs):
            self.p.setJointMotorControl2(self.panda, i, self.p.POSITION_CONTROL, jointPose[i], force=5 * 240.)

        ## Set position for both grippers
        self.p.setJointMotorControlArray(
            self.panda,
            self.gripperJoints,
            self.p.POSITION_CONTROL,
            targetPositions=[-width / 2, width / 2],
        )

        ## Can use self.p.getJointState for joint position

        self.pos = pos
        if ori is not None:
            self.ori = ori
        if width is not None:
            self.width = width

        if wait:
            last_err = 1e6
            while True:
                pb.stepSimulation()
                ee_pose = self.get_ee_pose()
                w = self.get_gripper_width()
                err = (
                        np.sum(np.abs(np.array(ee_pose[0]) - pos))
                        + np.sum(np.abs(np.array(ee_pose[1]) - ori_q))
                        + np.abs(w - width)
                )
                diff_err = last_err - err
                last_err = err

                if np.abs(diff_err) < self.tol:
                    break

        # print("FINISHED")


