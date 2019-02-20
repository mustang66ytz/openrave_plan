#!/usr/bin/env python

import numpy as np
import openravepy as orpy
import time
import math
import tf.transformations as tr

def build_rotation(axis, angle):
    rotation = None
    if axis == 'x':
        rotation = np.matrix([[1, 0, 0, 0],
                             [0, math.cos(-angle), math.sin(-angle), 0],
                             [0, -math.sin(-angle), math.cos(-angle), 0],
                             [0, 0, 0, 1]])
    if axis == 'y':
        rotation = np.matrix(
            [[math.cos(-angle), 0, -math.sin(-angle), 0],
             [0, 1, 0, 0],
             [math.sin(-angle), 0, math.cos(-angle), 0],
             [0, 0, 0, 1]]
        )
    if axis == 'z':
        rotation = np.matrix(
            [[math.cos(-angle), math.sin(-angle), 0, 0],
             [-math.sin(-angle), math.cos(-angle), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        )
    return rotation

# input: 4by4 rotational matrix needed to be rotated, rotational angles about x, y, and z axes
# output: the rotated 4by4 rotational matrix
def build_relative_rotation(original, x_rot, y_rot, z_rot):
    rotatex = build_rotation('x', x_rot)
    rotatey = build_rotation('y', y_rot)
    rotatez = build_rotation('z', z_rot)
    # apply the rotations here
    return original.dot(rotatex).dot(rotatey).dot(rotatez)


class PickPlace(object):

    def __init__(self):
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load('../osr_openrave/worlds/pick_and_place.env.xml')
        self.env.Load('../osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
        self.env.SetDefaultViewer()
        self.robot = self.env.GetRobot('denso_robotiq_85_gripper')
        self.manipulator = self.robot.SetActiveManipulator('gripper')
        self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())

        self.boxes = []

    def create_box(self, T, color = [0, 0.6, 0]):

        box = orpy.RaveCreateKinBody(self.env, '')
        box.SetName('box')
        box.InitFromBoxes(np.array([[0,0,0,0.035,0.03,0.005]]), True)
        g = box.GetLinks()[0].GetGeometries()[0]
        g.SetAmbientColor(color)
        g.SetDiffuseColor(color)
        box.SetTransform(T)
        self.env.Add(box, True)
        return box

    def load_rest(self):
        T = np.eye(4)
        container_center = np.array([0.4, 0.2, 0.195])
        # Destination
        T[:3, 3] = container_center + np.array([0, -0.5, 0])
        self.destination0 = self.create_box(T, color=[0, 0, 0.6])
        T[:3, 3] = container_center + np.array([0, -0.6, 0])
        self.destination1 = self.create_box(T, color=[0, 0, 0.6])
        # Generate random box positions

        nbox_per_layer = 2
        n_layer = 20
        h = container_center[2]

        for i in range(n_layer):
            nbox_current_layer = 0
            while nbox_current_layer < nbox_per_layer:
                theta = np.random.rand() * np.pi
                T[0, 0] = np.cos(theta)
                T[0, 1] = -np.sin(theta)
                T[1, 0] = np.sin(theta)
                T[1, 1] = np.cos(theta)
                T[0, 3] = container_center[0] + (np.random.rand() - 0.5) * 0.2
                T[1, 3] = container_center[1] + (np.random.rand() - 0.5) * 0.1
                T[2, 3] = h
                box = self.create_box(T)
                if self.env.CheckCollision(box):
                    self.env.Remove(box)
                else:
                    self.boxes.append(box)
                    nbox_current_layer += 1
            h += 0.011

    def motion_plan(self):

        # access the box from top to bottom
        for index, cur_box in reversed(list(enumerate(self.boxes))):
            cur_centroid = cur_box.ComputeAABB().pos()
            print "planning for the ", index, " box from the top"
            print "the position of the current box is: ", cur_centroid
            # modify the target pose of the object
            Tgrasp = cur_box.GetTransform()
            Tgrasp = np.array(build_relative_rotation(Tgrasp, 0, math.pi, math.pi / 2))

            ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                              iktype=orpy.IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            Tgrasp[:3, 3] = cur_centroid

            solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)

            if len(solutions) == 0:
                x_range = np.linspace(-0.0175, 0.0175, num=20)
                y_range = np.linspace(-0.015, 0.015, num=20)
                z_range = np.linspace(-0.025, 0.025, num = 10)

                count = 0
                found = False
                for cur_x in x_range:
                    if not found:
                        for cur_y in y_range:
                            if not found:
                                for cur_z in z_range:
                                    count = count+1
                                    print "adjusting...", count
                                    Tgrasp[:3, 3] = cur_centroid + [cur_x, cur_y, cur_z]
                                    solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)
                                    if len(solutions) > 0:
                                        print "find a solution... ", solutions
                                        found = True
                                        break
                            else:
                                break
                    else:
                        break

                if count == 4000:
                    count = 0
                    Tgrasp = cur_box.GetTransform()
                    Tgrasp = np.array(build_relative_rotation(Tgrasp, 0, math.pi, 0))
                    Tgrasp[:3, 3] = cur_centroid
                    for cur_x in x_range:
                        if not found:
                            for cur_y in y_range:
                                if not found:
                                    for cur_z in z_range:
                                        count = count + 1
                                        print "adjusting for new orientation...", count
                                        Tgrasp[:3, 3] = cur_centroid + [cur_x, cur_y, cur_z]
                                        solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)
                                        if len(solutions) > 0:
                                            print "find a solution... ", solutions
                                            found = True
                                            break
                                else:
                                    break
                        else:
                            break

            print "find ik solutions: ", solutions
            self.robot.SetActiveDOFValues(solutions[0])
            time.sleep(0.2)

            '''
            while len(solutions) == 0:
                replan_count = replan_count + 1
                print "replanning"
                # Tgrasp = np.array(build_relative_rotation(Tgrasp, 0, 0, math.pi/2))


                x_off = x_off - replan_count/100
                y_off = y_off - replan_count/100
                z_off = z_off - replan_count/100

                Tgrasp[:3, 3] = cur_centroid + [0.01, 0, 0]
                solutions = self.manipulator.FindIKSolutions(Tgrasp, 0)
            else:
                print solutions

            for solution in solutions:
                self.robot.SetActiveDOFValues(solution)
            '''


if __name__ == "__main__":
    # load the pick and place environment
    scene = PickPlace()
    # load the robot and the objects
    scene.load_rest()
    scene.motion_plan()

