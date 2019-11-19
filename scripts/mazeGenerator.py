#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Chirav Dave"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json

color_list = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.1725490196078432, 0.5562),
              (1.0, 0.9254901960784314, 0.8980392156862745),
              (0.6340, 0.9490196078431372, 0.6780392156862745), (1.0, 0.5764705882352941, 0.3980392156862745),
              (1.0, 1.0, 0.8980392156862745),
              (0.9764705882352941, 1.0, 0.8980392156862745), (0.9490196078431372, 1.0, 0.8980392156862745),
              (0.9254901960784314, 1.0, 0.8980392156862745),
              (0.8980392156862745, 1.0, 0.8980392156862745), (0.8980392156862745, 1.0, 0.9254901960784314),
              (0.8980392156862745, 0.9490196078431372, 1.0),
              (0.8980392156862745, 0.9254901960784314, 1.0), (0.8980392156862745, 0.8980392156862745, 1.0),
              (1.0, 0.8980392156862745, 0.8980392156862745),
              (1.0, 0.8980392156862745, 1.0), (1.0, 0.9019607843137255, 0.9490196078431372),
              (1.0, 0.8980392156862745, 0.9254901960784314)]


def copy_empty_world(root_path):
    f_in = open(root_path + '/worlds/empty_world.sdf', 'r')
    f_out = open(root_path + '/worlds/maze.sdf', 'w')
    for line in f_in:
        f_out.write(line)
    f_in.close()
    return f_out


def add_walls_description(f_out):
    for i in range(1, 5):
        f_out.write('<model name=\'wall{}\'>\n'.format(i))
        f_out.write(
            '<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
        f_out.write(
            '</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
        f_out.write(
            '<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
        f_out.write(
            '<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
        f_out.write(
            '<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')


def add_walls(f_out, length):
    scale = (length + 2) / 7.5
    wall_dimensions = [(-1, length / 2, -1.55905, scale, 1), (length / 2, length + 1, 0, scale, 1),
                       (length + 1, length / 2, -1.55905, scale, 1), (length / 2, -1, 0, scale, 1)]
    for i in range(4):
        f_out.write('<model name=\'wall{}\'>\n'.format(i + 1))
        f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1],
                                                                       wall_dimensions[i][2]))
        f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
        f_out.write('<link name=\'link\'>\n')
        f_out.write('<pose frame=\'\'>{} {} 0.42 -0 0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1],
                                                                          wall_dimensions[i][2]))
        f_out.write(
            '<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')


# geometry>/n<box>/n<size>0.245 0.16 0.03</size>/n</box>/n</geometry>
def add_dirt_description(f_out, coords):

    for z, i in enumerate(coords):
        x, y = i
        color=z+1
        f_out.write("<model name='dirt_{0}'>/n".format(z + 1))
        f_out.write("<link name='cover'>/n<pose frame=''>0 -0.000108 0.015405 0 -0 0</pose>/n<self_collide>0</self_collide>/n<kinematic>0</kinematic>/n")
        f_out.write("<gravity>1</gravity>/n<visual name='visual'>/n <geometry><cylinder><radius>0.1</radius><length>.3</length></cylinder></geometry>/n<material>/n<script>/n")
        f_out.write("<uri>model://dirt_1/materials/scripts/dirt_{0}.material</uri>/n<uri>model://dirt_1/materials/textures/cover{0}.jpg</uri>/n<name>dirt_{0}</name>/n</script>/n</material>/n<cast_shadows>1</cast_shadows>/n<transparency>0</transparency>/n</visual>/n<collision name='collision'>/n<laser_retro>0</laser_retro>/n<max_contacts>10</max_contacts>/n<geometry><cylinder><radius>0.1</radius><length>.3</length></cylinder></geometry>/n<surface>/n<contact>/n<ode/>/n</contact>/n<bounce/>/n<friction>/n<ode><mu>1000</mu><mu2>1000</mu2></ode>/n</friction>/n</surface>/n</collision>/n<inertial>/n<inertia>/n<ixx>0.00058</ixx>/n<ixy>0</ixy>/n<ixz>0</ixz>/n<iyy>0.00058</iyy>/n<iyz>0</iyz>/n<izz>0.00019</izz>/n</inertia>/n<mass>0.05</mass>/n</inertial>/n</link>/n<link name='page'>/n<pose frame=''>0 0.000608 0.015405 0 -0 0</pose>/n<visual name='visual'>/n<pose frame=''>0 0 0 0 -0 0</pose>/n<geometry><cylinder><radius>0.1</radius><length>.3</length></cylinder></geometry>/n<material>/n<lighting>1</lighting>/n<ambient>1 1 1 1</ambient>/n<diffuse>1 1 1 1</diffuse>/n<specular>0.01 0.01 0.01 1</specular>/n<emissive>0 0 0 1</emissive>/n<shader type='vertex'>/n<normal_map>__default__</normal_map>/n</shader>/n</material>/n<cast_shadows>1</cast_shadows>/n<transparency>0</transparency>/n</visual>/n<collision name='collision'>/n<laser_retro>0</laser_retro>/n<max_contacts>10</max_contacts>/n<pose frame=''>0 0 0 0 -0 0</pose>/n<geometry><cylinder><radius>0.1</radius><length>.3</length></cylinder></geometry>/n<surface>/n<contact>/n<ode/>/n</contact>/n<bounce/>/n<friction>/n<ode><mu>1000</mu>/n<mu2>1000</mu2>/n</ode>/n</friction>/n</surface>/n</collision>/n<self_collide>0</self_collide>/n<inertial>/n<inertia>/n<ixx>0.00058</ixx>/n<ixy>0</ixy>/n<ixz>0</ixz>/n<iyy>0.00058</iyy>/n<iyz>0</iyz>/n<izz>0.00019</izz>/n</inertia>/n<mass>0.05</mass>/n</inertial>/n<kinematic>0</kinematic>/n<gravity>1</gravity>/n</link>/n<static>0</static>/n<allow_auto_disable>1</allow_auto_disable>/n<pose frame=''>0.830691 0.858956 0 0 -0 0</pose>/n</model>".format(color, x, y))

    f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose frame=\'\'>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n<projection_type>perspective</projection_type>\n</camera>\n</gui>\n')


# change in dimenssion of the dirt is handled in this function.
def add_dirt(f_out, x, y, dirt_size_scale, dirtCounter):
    f_out.write("<model name='dirt_{0}'>\n".format(dirtCounter))
    f_out.write("<pose frame=''>{0} {1} -0.000405 -1e-06 1e-06 0</pose>\n".format(x, y))
    f_out.write("<scale>{0} {0} 1</scale>\n".format(dirt_size_scale))
    f_out.write("<link name='cover'>\n")
    f_out.write("<pose frame=''>{0} {1} 0.015 -1e-06 1e-06 0</pose>\n".format(x, y - 0.000108))
    f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0.017626 0.011511 -0.205341 -0.7674 1.17508 -0</acceleration>\n<wrench>0.017626 0.011511 -0.205341 0 -0 0</wrench>\n</link>\n<link name='page'>\n")
    f_out.write("<pose frame=''>{0} {1} 0.015 0 1e-06 0</pose>\n".format(x, y + 0.000608))
    f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>")




def dirt_dict_generator(dirts, dirtCounter, location, coord1, cord2):
    dirts["dirt_" + str(dirtCounter)]["loc"] = location
    dirts["dirt_" + str(dirtCounter)]["load_loc"] = []
    dirts["dirt_" + str(dirtCounter)]["load_loc"].append(coord1)
    dirts["dirt_" + str(dirtCounter)]["load_loc"].append(cord2)


def add_bloced_edges(x, y):
    blocked_list = []
    x_dec = 0.5
    y_dec = 0.5
    blocked_list.append((x - x_dec, y))
    blocked_list.append((x + x_dec, y))
    blocked_list.append((x, y - y_dec))
    blocked_list.append((x, y + y_dec))

    return blocked_list


def generate_blocked_edges(grid_dimension, no_of_dirts, seed, root_path, myscale=0.5):

        object_dict = {}
        dirts = {}
        np.random.seed(seed)
        blocked_edges = set()
        list_of_list_of_coords = []
        f_out = copy_empty_world(root_path)
        add_walls(f_out, grid_dimension * myscale)
        dirt_size_scale = 1
        dirtCounter = 1


        n_obstacles =no_of_dirts
        count = 1
        coords = []
        while (count <= n_obstacles):
            dirts["dirt_" + str(dirtCounter)] = {}
            x = myscale * np.random.randint(0, (grid_dimension + 1))

            y = myscale * np.random.randint(0, (grid_dimension + 1))
          #  print x,y
            flag = np.random.randint(0, 2)
           # print flag
            if (flag == 0 and ((x + myscale) <= grid_dimension * myscale ) and (
                    (x, y, x + myscale, y) not in blocked_edges)):
                blocked_edges.add((x, y, x + myscale, y))
            #    print '222'
                offset = np.random.uniform(0, 0.05 * myscale)
                coords.append((x + myscale / 2 + offset, y))
                dirt_dict_generator(dirts, dirtCounter, (x + myscale / 2 + offset, y), (x, y), (x + myscale, y))
                add_dirt(f_out, x + myscale / 2 + offset, y, dirt_size_scale, dirtCounter)
                count += 1

            elif (flag == 1 and ((y + myscale) <= grid_dimension * myscale ) and (
                    (x, y, x, y + myscale) not in blocked_edges)):
              #  print '2322'
                blocked_edges.add((x, y, x, y + myscale))
                offset = np.random.uniform(0, 0.05 * myscale)
                coords.append((x, y + myscale / 2 - offset))
                dirt_dict_generator(dirts, dirtCounter, (x, y + myscale / 2 - offset), (x, y), (x, y + myscale))
                add_dirt(f_out, x, y + myscale / 2 - offset, dirt_size_scale, dirtCounter)
                count += 1

            else:
                dirtCounter -= 1
            dirtCounter += 1



    	f_out.write('</state>')
        add_walls_description(f_out)

        add_dirt_description(f_out,coords)

        f_out.write('</world>\n</sdf>')
        f_out.close()

        object_dict["dirts"] = dirts
        with open(root_path + '/objects.json', 'w') as fp:
            json.dump(object_dict, fp)

        mazeInfo = [(0, grid_dimension, "EAST", myscale), blocked_edges]
        return object_dict, mazeInfo


if __name__ == "__main__":
	print "Doing Nothing!!!"
    # robot height = 0.4m
    # trolly height = 1m

  #  subject_count = 6
   # dirt_sizes = 2
    #dirt_count_of_each_size = 5
    #dirt_count_of_each_subject = dirt_count_of_each_size * dirt_sizes
    #dirt_count_list = [dirt_count_of_each_size] * subject_count * dirt_sizes
    #number_of_trollies = subject_count * 2
    #grid_size = max((((dirt_count_of_each_subject * subject_count) / 4) // 1) + 1, ((number_of_trollies / 4) * 7), 10)

    #root_path = "/home/ketan/catkin_ws/src/search"
    #print "************"
    #dirts, mazeInfo = generate_blocked_edges(grid_size, 4, 2,4, root_path, 0.5)

    # pprint.pprint(dirts)
    #print(mazeInfo)