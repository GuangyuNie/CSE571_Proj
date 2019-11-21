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

import numpy as np
import os, sys
from room_utils import *
from geometry_msgs.msg import Twist, Pose
class Maze:

	def copy_empty_world(self):
		parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
		#File containing empty world description
		f_in = open(parent_dir+'/worlds/empty_world.sdf', 'r')
		#File to save your maze
		f_out = open(parent_dir+'/worlds/maze.sdf', 'w')
		#Copying empty world description into output maze
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out
        def add_book_description(self,f_out, coords, color=2, bookCounter=1):
           for z, i in enumerate(coords):
              x, y = i
              f_out.write("<model name='book_{0}'>/n".format(bookCounter+1))
              f_out.write("<link name='cover'>/n<pose frame=''>0 -0.000108 0.015405 0 -0 0</pose>/n<self_collide>0</self_collide>/n<kinematic>0</kinematic>/n")
              f_out.write("<gravity>1</gravity>/n<visual name='visual'>/n<geometry>/n<box>/n<size>0.245 0.16 0.03</size>/n</box>/n</geometry>/n<material>/n<script>/n")
              f_out.write("<uri>model://book_1/materials/scripts/book_{0}.material</uri>/n<uri>model://book_1/materials/textures/cover{0}.jpg</uri>/n<name>book_{0}</name>/n</script>/n</material>/n<cast_shadows>1</cast_shadows>/n<transparency>0</transparency>/n</visual>/n<collision name='collision'>/n<laser_retro>0</laser_retro>/n<max_contacts>10</max_contacts>/n<geometry>/n<box>/n<size>0.245 0.16 0.03</size>/n</box>/n</geometry>/n<surface>/n<contact>/n<ode/>/n</contact>/n<bounce/>/n<friction>/n<ode><mu>1000</mu><mu2>1000</mu2></ode>/n</friction>/n</surface>/n</collision>/n<inertial>/n<inertia>/n<ixx>0.00058</ixx>/n<ixy>0</ixy>/n<ixz>0</ixz>/n<iyy>0.00058</iyy>/n<iyz>0</iyz>/n<izz>0.00019</izz>/n</inertia>/n<mass>0.05</mass>/n</inertial>/n</link>/n<link name='page'>/n<pose frame=''>0 0.000608 0.015405 0 -0 0</pose>/n<visual name='visual'>/n<pose frame=''>0 0 0 0 -0 0</pose>/n<geometry>/n<box>/n<size>0.24502 0.15862 0.028</size>/n</box>/n</geometry>/n<material>/n<lighting>1</lighting>/n<ambient>1 1 1 1</ambient>/n<diffuse>1 1 1 1</diffuse>/n<specular>0.01 0.01 0.01 1</specular>/n<emissive>0 0 0 1</emissive>/n<shader type='vertex'>/n<normal_map>__default__</normal_map>/n</shader>/n</material>/n<cast_shadows>1</cast_shadows>/n<transparency>0</transparency>/n</visual>/n<collision name='collision'>/n<laser_retro>0</laser_retro>/n<max_contacts>10</max_contacts>/n<pose frame=''>0 0 0 0 -0 0</pose>/n<geometry>/n<box>/n<size>0.245 0.16 0.03</size>/n</box>/n</geometry>/n<surface>/n<contact>/n<ode/>/n</contact>/n<bounce/>/n<friction>/n<ode><mu>1000</mu>/n<mu2>1000</mu2>/n</ode>/n</friction>/n</surface>/n</collision>/n<self_collide>0</self_collide>/n<inertial>/n<inertia>/n<ixx>0.00058</ixx>/n<ixy>0</ixy>/n<ixz>0</ixz>/n<iyy>0.00058</iyy>/n<iyz>0</iyz>/n<izz>0.00019</izz>/n</inertia>/n<mass>0.05</mass>/n</inertial>/n<kinematic>0</kinematic>/n<gravity>1</gravity>/n</link>/n<static>0</static>/n<allow_auto_disable>1</allow_auto_disable>/n<pose frame=''>0.830691 0.858956 0 0 -0 0</pose>/n</model>".format(color, x, y))
    	      bookCounter += 1
              f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose frame=\'\'>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n<projection_type>perspective</projection_type>\n</camera>\n</gui>\n')

#change in dimenssion of the book is handled in this function.
        def add_book(self,f_out, x, y, book_size_scale, bookCounter):
              f_out.write("<model name='book_{0}'>\n".format(bookCounter))
              f_out.write("<pose frame=''>{0} {1} -0.000405 -1e-06 1e-06 0</pose>\n".format(x, y))
              f_out.write("<scale>{0} {0} 1</scale>\n".format(book_size_scale))
              f_out.write("<link name='cover'>\n")
              f_out.write("<pose frame=''>{0} {1} 0.015 -1e-06 1e-06 0</pose>\n".format(x, y-0.000108))
              f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0.017626 0.011511 -0.205341 -0.7674 1.17508 -0</acceleration>\n<wrench>0.017626 0.011511 -0.205341 0 -0 0</wrench>\n</link>\n<link name='page'>\n")
              f_out.write("<pose frame=''>{0} {1} 0.015 0 1e-06 0</pose>\n".format(x, y+0.000608))
              f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>")

	#Method to add wall description surrounding the maze
	def add_walls_description(self, f_out):
		for i in range(1, 5):
			f_out.write('<model name=\'wall{}\'>\n'.format(i))
			f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
			f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
			f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
			f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
			f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')

	#Method to place walls around the maze
	def add_walls(self, f_out, length):
		scale = (length+2)/7.5
		wall_dimensions = [(-1, length/2, -1.55905, scale, 1), (length/2, length+1, 0, scale, 1), (length+1, length/2, -1.55905, scale, 1), (length/2, -1, 0, scale, 1)]
		for i in range(4):
			f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
			f_out.write('<link name=\'link\'>\n')
			f_out.write('<pose frame=\'\'>{} {} 0.42 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to add description for cans
	def add_can_description(self, f_out, coords):
		for i in coords:
			x, y = i
			f_out.write('<model name=\'can{}{}\'>\n'.format(x, y))
			f_out.write('<link name=\'link\'>\n<pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>39</mass>\n<inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n</inertial>\n<collision name=\'collision\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n')
			f_out.write('<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n<name>Beer/Diffuse</name>\n</script>\n</material>\n</visual>\n')
			f_out.write('<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>0.888525 -2.58346 0 0 -0 0</pose>\n</model>\n')
			f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose frame=\'\'>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n<projection_type>perspective</projection_type>\n</camera>\n</gui>\n')

	#Method to place cans inside the maze
	def add_can(self, f_out, x, y):
		f_out.write('<model name=\'can{}{}\'>\n'.format(x, y))
		f_out.write('<pose frame=\'\'>{} {} -2e-06 1e-06 0 -9.5e-05</pose>\n'.format(x, y))
		f_out.write('<scale>0.5 0.5 1</scale>\n<link name=\'link\'>\n')
		f_out.write('<pose frame=\'\'>{} {} 0.114998 1e-06 0 -9.5e-05</pose>\n'.format(x, y))
		f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -3.822 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to add description for the goal can
	def add_goal_description(self, f_out, coord):
		f_out.write('<model name=\'goal\'>\n<link name=\'link\'><pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>0.0005</mass><inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n')
		f_out.write('<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n</inertial>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n')
		f_out.write('<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n')
		f_out.write('<name>Beer/Diffuse</name>\n</script>\n<ambient>1 0 0 1</ambient>\n<diffuse>1 0 0 1</diffuse>\n<specular>0 0 0 1</specular>\n<emissive>0 0 0 1</emissive>\n<shader type=\'vertex\'>\n')
		f_out.write('<normal_map>__default__</normal_map>\n</shader>\n</material>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n</visual>\n')
		f_out.write('<collision name=\'collision\'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n')
		f_out.write('</cylinder>\n</geometry>\n<surface>\n<friction>\n<ode>\n<mu>1</mu>\n<mu2>1</mu2>\n<fdir1>0 0 0</fdir1>\n<slip1>0</slip1>\n<slip2>0</slip2>\n</ode>\n<torsional>\n<coefficient>1</coefficient>\n')
		f_out.write('<patch_radius>0</patch_radius>\n<surface_radius>0</surface_radius>\n<use_patch_radius>1</use_patch_radius>\n<ode>\n<slip>0</slip>\n</ode>\n</torsional>\n</friction>\n<bounce>\n')
		f_out.write('<restitution_coefficient>0</restitution_coefficient>\n<threshold>1e+06</threshold>\n</bounce>\n<contact>\n<collide_without_contact>0</collide_without_contact>\n<collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n')
		f_out.write('<collide_bitmask>1</collide_bitmask>\n<ode>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n<max_vel>0.01</max_vel>\n<min_depth>0</min_depth>\n</ode>\n')
		f_out.write('<bullet>\n<split_impulse>1</split_impulse>\n<split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n')
		f_out.write('<kd>1</kd>\n</bullet>\n</contact>\n</surface>\n</collision>\n</link>\n<static>0</static>\n<allow_auto_disable>1</allow_auto_disable>\n')
		f_out.write('<pose frame=\'\'>{} {} 0 0 -0 0</pose>\n</model>\n'.format(coord, coord))

        def add_trolly_description(self,f_out, coords):
	 subject_count = 0
	 for z, i in enumerate(coords):
		x, y = i

		scale = 0.03
		f_out.write("<model name='trolly_{0}\'>/n".format(z+1))
		f_out.write("<link name='link'>\n<pose frame=''>0 0 0 0 -0 0</pose>\n<inertial>\n<mass>1</mass>\n")
		f_out.write("<pose frame=''>0 0 0 0 -0 0</pose>\n<inertia>\n<ixx>1</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>1</iyy>\n<iyz>0</iyz>\n<izz>1</izz>\n</inertia>\n</inertial>\n")
		f_out.write("<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n<visual name='visual'>\n<geometry>\n")
		f_out.write("<mesh>\n<uri>model://bookcart/meshes/bookcart.dae</uri>\n<scale>{0} {0} 1</scale>\n</mesh>\n</geometry>\n<pose frame=''>0 0 0 0 -0 0</pose>\n".format(scale))
		f_out.write("<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n<material>\n<ambient>{0} {1} {2} 0</ambient>\n<diffuse>{0} {1} {2} 0</diffuse>\n<specular>0 0 0 1</specular>\n<emissive>0 0 0 1</emissive>\n".format(color_list[subject_count][0], color_list[subject_count][1], color_list[subject_count][2]))
		f_out.write("<script>\n<name>ModelPreview_1::link::visual_MATERIAL_</name>\n<uri>__default__</uri>\n</script>\n<shader type='vertex'>\n<normal_map>__default__</normal_map>\n</shader>\n</material>\n</visual>\n<collision name='collision_0'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose frame=''>0 0 0 0 -0 0</pose>\n<geometry>\n<mesh>\n<uri>model://bookcart/meshes/bookcart.dae</uri>\n<scale>{} {} 1</scale>\n</mesh>\n</geometry>\n<surface>\n<friction>\n<ode>\n<mu>1</mu>\n<mu2>1</mu2>\n<fdir1>0 0 0</fdir1>\n<slip1>0</slip1>\n<slip2>0</slip2>\n</ode>\n<torsional>\n<coefficient>1</coefficient>\n<patch_radius>0</patch_radius>\n<surface_radius>0</surface_radius>\n<use_patch_radius>1</use_patch_radius>\n<ode>\n<slip>0</slip>\n</ode>\n</torsional>\n</friction>\n<bounce>\n<restitution_coefficient>0</restitution_coefficient>\n<threshold>1e+06</threshold>\n</bounce>\n<contact>\n<collide_without_contact>0</collide_without_contact>\n<collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n<collide_bitmask>1</collide_bitmask>\n<ode>\n".format(scale,scale))
		f_out.write("<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n<max_vel>0.01</max_vel>\n<min_depth>0</min_depth>\n</ode>\n<bullet>\n<split_impulse>1</split_impulse>\n<split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n</bullet>\n</contact>\n</surface>\n</collision>\n</link>\n<static>1</static>\n<allow_auto_disable>1</allow_auto_disable>\n<pose frame=''>1.06777 -0.068202 0 0 -0 0</pose>\n</model>")

        def add_trolly(self,f_out, x, y, scale, trollies_count):
	 f_out.write("<model name='trolly_{0}'>".format(trollies_count))
	 f_out.write("<pose frame=''>{0} {1} 0 0 -0 0</pose>".format(x, y))
	 f_out.write("<scale>{0} {0} 0.35</scale>".format(scale))
	 f_out.write("<link name='link_20'>")
	 f_out.write("<pose frame=''>{0} {1} 0 0 -0 0</pose>".format(x, y))
	 f_out.write("<velocity>0 0 0 0 -0 0</velocity>")
	 f_out.write("<acceleration>0 0 0 0 -0 0</acceleration>")
	 f_out.write("<wrench>0 0 0 0 -0 0</wrench>")
	 f_out.write("</link>")
	 f_out.write("</model>")


	#Method to place the goal can
	def add_goal(self, f_out, coord):
		f_out.write('<model name=\'goal\'>\n')
		f_out.write('<pose frame=\'\'>{} {} -9e-06 -1e-06 -4e-06 0</pose>\n'.format(coord, coord))
		f_out.write('<scale>1 1 1</scale>\n<link name=\'goal::link\'>\n')
		f_out.write('<pose frame=\'\'>{} {} 0.114991 -1e-06 -4e-06 0</pose>\n'.format(coord, coord))
		f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to generate maze
        def generate_maze(self,grid_dimension, n_obstacles, seed, scale=0.5):
                
            	blocked_edges = set()
		coords = []
                trolliesCoords = []
                bookcords = []
                Player_cord = []
		f_out = self.copy_empty_world()
		self.add_walls(f_out, grid_dimension*scale)
                count = 0
                i = j = 0
                t_count = 0
                bookCounter = 0
                room = room_topology_generation()
                
                room = place_boxes_and_player(room, 3,False) 

                room_structure = np.copy(room)
                room_structure[room_structure == 5] = 1

                # Room structure represents the current state of the room including movable parts
                room_state = room.copy()
                room_state[room_state == 2] = 4

                best_room, score, box_mapping = reverse_playing(room_state, room_structure)
                room_state[best_room == 3] = 4
                

                #room_structure, room_state, box_mapping = generate_room()
                print(best_room)
                for i in range(best_room.shape[0]):
                     for j in range(best_room.shape[1]):
                         if(best_room[i,j] == 0):
                             scale = 0.5
                             x = scale*i
                             y = scale*j
                             blocked_edges.add((x, y, x+scale, y))
                             coords.append((x, y))
                             self.add_can(f_out, x, y)
                         elif(best_room[i,j] == 5):
                              #Player
                             scale = 0.5
                             x = scale*i
                             y = scale*j
	                     Player_cord = (x,y)
                             """
                         elif(best_room[i,j] == 2):
                             #Target
                             t_count += 1
                             size = "small"
                             scale = 0.5
                             x = scale*i
                             y = scale*j
                             trolliesCoords.append((x,y))
                             self.add_trolly(f_out,x,y,scale,t_count)
		             """
                         elif(best_room[i,j] == 3):
                             scale = 0.5
                             book_size_scale = 0.5
                             bookCounter += 1
                             x = scale*i
                             y = scale*j
                             bookcords.append((x,y))
                             self.add_book(f_out, x, y, book_size_scale, bookCounter)
 
                             '''
			     #flag is used to decide if we want to block the edge (x, y) and (x+1, y) or (x, y) and (x, y+1) 
			     flag = np.random.randint(0, 2)
			     if(flag == 0 and ((x+scale) <= grid_dimension*scale) and ((x, y, x+scale, y) not in blocked_edges)):
				blocked_edges.add((x, y, x+scale, y))
				#Adding obstacle in the middle with some offset value of the edge to be blocked
				offset = np.random.uniform(0, 0.07*scale)
				coords.append((x+scale/2+offset, y))
				self.add_can(f_out, x+scale/2+offset, y)
				
			     elif(flag == 1 and ((y+scale) <= grid_dimension*scale) and ((x, y, x, y+scale) not in blocked_edges)):
				blocked_edges.add((x, y, x, y+scale))
				#Adding obstacle in the middle with some offset value of the edge to be blocked
				offset = np.random.uniform(0, 0.07*scale)
				coords.append((x, y+scale/2-offset))
				self.add_can(f_out, x, y+scale/2-offset)
                       		self.add_goal(f_out, grid_dimension*0.5)
                                '''
		f_out.write('</state>')
		#self.add_goal_description(f_out, grid_dimension*0.5)
		self.add_walls_description(f_out)
		self.add_can_description(f_out, coords)
                self.add_trolly_description(f_out, trolliesCoords)
                self.add_book_description(f_out, bookcords)
		f_out.write('</world>\n</sdf>')
		f_out.close()
		maze = [(0, grid_dimension, 'EAST', scale), blocked_edges]
		return maze
                



