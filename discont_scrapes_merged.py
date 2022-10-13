"""
Simulate the scraping of a cube. 
(Continous visual trajectory with discontinous trajectory of audio - pause in the middle)
"""
import math
import numpy as np
from tdw.physics_audio.audio_material import AudioMaterial
from tdw.physics_audio.object_audio_static import ObjectAudioStatic
from tdw.controller import Controller
from tdw.tdw_utils import TDWUtils
from tdw.librarian import ModelLibrarian
from tdw.add_ons.third_person_camera import ThirdPersonCamera
from tdw.add_ons.audio_initializer import AudioInitializer
from tdw.add_ons.py_impact import PyImpact
from tdw.physics_audio.scrape_material import ScrapeMaterial
from tdw.backend.paths import EXAMPLE_CONTROLLER_OUTPUT_PATH
from pathlib import Path
from tdw.add_ons.physics_audio_recorder import PhysicsAudioRecorder
import psutil
from tdw.add_ons.interior_scene_lighting import InteriorSceneLighting
import argparse
from tdw.controller import Controller
from tdw.tdw_utils import TDWUtils
from tdw.add_ons.py_impact import PyImpact
from tdw.add_ons.audio_initializer import AudioInitializer
from tdw.add_ons.third_person_camera import ThirdPersonCamera
from tdw.backend.paths import EXAMPLE_CONTROLLER_OUTPUT_PATH
import time
import psutil
import os
from configparser import ConfigParser
import time 
from tdw.output_data import OutputData, Rigidbodies, StaticRigidbodies
from tdw.add_ons.object_manager import ObjectManager
from make_polynomial import get_poly_velocity2

class DiscontScrapesDemo(Controller):
    """
    Run a scraping simulation where there are two sliding cubes. The first cube moves in a continous path (this is the cube
    that can be seen from the camera viewpoints). The second cube moves in a discontinous path, with a small pause, in the
    middle of the path.  
    """
    def __init__(self, configs: dict):
        super().__init__()
        self.audio_device = configs["audio_device"]
        self.run_type = "cam_" + configs['cam_view'] + "_discontlen_" + configs['discont_len']
        
        pbased = bool(int(configs["physics_based"]))
        linearbool = bool(int(configs["linear_vel"]))
        shadowbool = bool(int(configs["shadow"]))
        self.lightx = bool(int(configs["lightx"]))
        self.linear_vel = linearbool

        self.run_type = "cam_" + configs['cam_view'] + "_discontlen_" + configs['discont_len'] + "_physics_" + str(pbased) + "_linear_" + str(linearbool) + "_shadow_" + str(shadowbool) + str(self.lightx)

        Controller.MODEL_LIBRARIANS["models_core.json"] = ModelLibrarian("models_core.json")
        Controller.MODEL_LIBRARIANS["models_flex.json"] = ModelLibrarian("models_flex.json")
        # visual material of tables
        visual_mat_table = ['b05_table_new', "willisau_varion_w3_table", 'glass_table']
        self.scrape_surface_model_name = visual_mat_table[int(configs['table1mat'])]
        self.scrape_surface2_model_name = visual_mat_table[int(configs['table2mat'])]
        # visual material of cube
        visual_mat_cube = ['ceramic_raw_striped', 'wood_beech_natural', 'metal_cast_iron']
        self.scrape_surface_cube1_name = visual_mat_cube[int(configs['cubemat'])]
        self.scrape_surface_cube2_name = visual_mat_cube[int(configs['cube2mat'])]
        self.cube_visual_material = self.scrape_surface_cube1_name
        self.cube_visual_material2 = self.scrape_surface_cube2_name
        # cube y-position depending on type
        cubey = {'b05_table_new': 0.2, "willisau_varion_w3_table": 0, 'glass_table':0}
        self.cube_posy = cubey[self.scrape_surface_model_name]
        # scale of the table depending on type
        table_scale ={'b05_table_new':{"x": 1, "y": 1.3, "z": 14},"willisau_varion_w3_table":{"x": 0.5, "y": 1, "z": 13}, 'glass_table':{"x": 0.8, "y": 1, "z": 12}}
        self.table1_scale = table_scale[self.scrape_surface_model_name]
        self.table2_scale = table_scale[self.scrape_surface2_model_name]
        impact_mat = ["plastic_hard_1", "wood_soft_1", "glass_1", "stone_4", "metal_1"]
        self.impact_mat1 = impact_mat[int(configs['scrape1'])]
        self.impact_mat2 = impact_mat[int(configs['scrape2'])]
        # scrape material used for sound - small/medium/large
        scrape_mat = [ScrapeMaterial.vinyl, ScrapeMaterial.bass_wood, ScrapeMaterial.ceramic]
        self.scrapemat1 = scrape_mat[int(configs['scrape1'])]
        self.scrapemat2 = scrape_mat[int(configs['scrape2'])]
        # get library records
        self.surface_record = Controller.MODEL_LIBRARIANS["models_core.json"].get_record(self.scrape_surface_model_name)
        self.surface_record2 = Controller.MODEL_LIBRARIANS["models_core.json"].get_record(self.scrape_surface2_model_name)
        # define applied force (only relevant when simulating physics)
        force = 0.5
        # mass of cubes (doesn't matter since object is teleported)
        masses = [0.01,1,100]
        self.cube_mass = masses[int(configs['mass'])]
        self.cube2_mass = masses[int(configs['secondmass'])]
        self.cube_bounciness = 0
        # visual cube scale 
        scale_dict = {0:{"x": 0.1, "y": 0.04, "z": 0.1}, 1: {"x": 0.5, "y": 0.3, "z": 0.5}}
        self.scale_factor_cube = scale_dict[int(configs['cube_size'])]
        # long or short scrape 
        self.scrape_length = int(configs['scrape_length'])
        # how long for the return : (skipping steps) on the return scrape
        self.waiter_time = float(configs['waiter_time'])
        self.shadow_present = int(configs['shadow'])
        self.obstacle_present = int(configs['obstacle'])
        cam_pos = [
            {"x": 1.2, "y": 1.4, "z": -2.86},
            {"x": 1, "y": 1.2, "z": 3.8},
            {"x": 0, "y": 1.4, "z": -3.5},
            {"x": 4.1, "y":3, "z": 0.3},
        ]
        cam_view = [
            {"x": -0.4, "y": 0.5, "z": 0},
            {"x": -0.4, "y": 0.5, "z": 0},
            {"x": 0, "y": 0.5, "z": 0},
            {"x": 0.7, "y":0.5, "z": 0.3}
        ]
        self.look_at = cam_view[int(configs['cam_view'])]
        self.look_at2 = self.look_at
        self.cam_point = cam_pos[int(configs['cam_view'])]
        self.cam_point2 = self.cam_point
        self.discont_len = int(configs['discont_len'])
        self.physics_based = int(configs['physics_based'])
        self.object_num = int(configs['object_num'])
        self.window_w = 1540
        self.window_h = 880
        self.capture_position = TDWUtils.get_expected_window_position(window_width=self.window_w,
                                                                      window_height=self.window_h,
                                                                      title_bar_height=int(configs["title_bar_height"]),
                                                                      monitor_index=int(configs["monitor_index"]))
        # Initialize PyImpact.
        rng = np.random.RandomState(0)
        self.py_impact = PyImpact(rng=rng)
        self.commands = []
        self.capture_path = EXAMPLE_CONTROLLER_OUTPUT_PATH.joinpath("crossmodal_illusions").joinpath(self.run_type + ".mp4")

    def initialize_scene(self):
        """
        Initialize audio, camera, and window settings
        """

        audio = AudioInitializer(avatar_id="a", framerate=60)
        print(f"Video will be saved to: {self.capture_path}")
        # set camera position depending on number of objects
        if self.object_num > 1:
            camera = ThirdPersonCamera(position=self.cam_point2,
                                       look_at=self.look_at2,
                                       avatar_id="a")
        else:
            camera = ThirdPersonCamera(position=self.cam_point,
                                       look_at=self.look_at,
                                       avatar_id="a")
        # Set a random number generator with a hardcoded random seed so that the generated audio will always be the same.
        # If you want the audio to change every time you run the controller, do this instead: `py_impact = PyImpact()`.
        rng = np.random.RandomState(0)

        if not self.lightx:
            hdri_skybox = "bergen_4k"
            interior_scene_lighting = InteriorSceneLighting(hdri_skybox=hdri_skybox)
            self.add_ons.extend([camera, audio,interior_scene_lighting])
        else:
            self.add_ons.extend([camera, audio])

        # add empty room,  skybox and other screen-size and light settings
        commands = [TDWUtils.create_empty_room(40, 40),
                    {"$type": "set_target_framerate",
                     "framerate": 60},
                    {"$type": "set_screen_size",
                     "width": self.window_w,
                     "height": self.window_h},
                    {"$type": "rotate_directional_light_by",
                     "angle": 175},
                    {"$type": "set_aperture",
                     "aperture": 8.0},
                    {"$type": "set_shadow_strength",
                     "strength": 1.0}]
                    
        if self.physics_based == False:
            commands.extend([
                {"$type": "simulate_physics",
                    "value": False},])
        self.resp = self.communicate(commands)
    


    def add_table(self, rank):
        """
        Place either one or two tables
        """
        if rank == 1:
            self.commands = self.get_add_physics_object(model_name=self.scrape_surface_model_name,
                                                    library="models_core.json",
                                                    object_id=self.surface_id,
                                                    kinematic=True,
                                                    scale_factor=self.table1_scale)
        else:
            self.commands.extend(self.get_add_physics_object(model_name=self.scrape_surface2_model_name,
                                                    library="models_core.json",
                                                    object_id=self.surface2_id,
                                                    kinematic=True,
                                                    scale_factor=self.table2_scale,
                                                    position={"x": self.surface_record.bounds["back"]["x"]+8, "y": 0, "z": 0}))
    
    def add_shadow_cube(self, zstart):
        """
        To get a physically implausible scene, we add a secondary cube
        so that the first cube and float and project a shadow. 
        """
        c_vis_mat = self.cube_visual_material
        self.commands.extend(self.get_add_physics_object(model_name="cube",
                                                        library="models_flex.json",
                                                        object_id=self.shadow_cube,
                                                        position={"x": 0,
                                                                "y": self.surface_record.bounds["top"]["y"]+ self.cube_posy,
                                                                "z": zstart+2.4+2.4},
                                                        scale_factor=self.scale_factor_cube,
                                                        default_physics_values=False,
                                                        mass=self.cube_mass,
                                                        bounciness=self.cube_bounciness))

        self.commands.extend([self.get_add_material(c_vis_mat, library="materials_low.json"),
                            {"$type": "set_visual_material",
                            "id": self.shadow_cube,
                            "material_name": c_vis_mat,
                            "object_name": "cube",
                            "material_index": 0}, 
                            ])

    def add_apple_obstacles(self):
        """
        To strengthen the physical implasuibility of the videos, we add two extra apples on the scene. 
        """
        self.commands.extend(self.get_add_physics_object(model_name="apple",
                                            position={"x": -0.3,
                                                "y": self.surface_record.bounds["top"]["y"]+ self.cube_posy,
                                                "z": self.surface_record.bounds["back"]["z"]+0.2-0.7},
                                            object_id=self.get_unique_id()))
        apple2 = self.get_unique_id()
        self.commands.extend(self.get_add_physics_object(model_name="apple",
                                    position={"x": -0.15,
                                        "y": self.surface_record.bounds["top"]["y"]+ self.cube_posy,
                                        "z": self.surface_record.bounds["back"]["z"]+0.2-0.65},
                                    object_id=apple2))
        self.commands.extend( [{"$type": "set_color",
            "color": {"r": 1.0, "g": 0, "b": 0, "a": 1.0},
            "id": apple2}])


    def add_cube(self, zstart, rank):
        """
        Place one or two cubes
        """
        c_vis_mat = self.cube_visual_material
        lift_cube = 0
        if self.shadow_present:
            lift_cube = 0.2

        cube_id = self.cube_id
        self.xpos = 0
        self.ypos = self.surface_record.bounds["top"]["y"]+ self.cube_posy+lift_cube
        cube_mass = self.cube_mass
        if rank == 2:
            cube_id = self.cube_id2
            cube_mass = self.cube2_mass
            self.xpos = self.surface_record.bounds["back"]["x"]+8
            self.ypos = self.surface_record2.bounds["top"]["y"]+self.cube_posy+0.2

        self.commands.extend(self.get_add_physics_object(model_name="cube",
                                                    library="models_flex.json",
                                                    object_id=cube_id,
                                                    position={"x": self.xpos,
                                                            "y": self.ypos,
                                                            "z": zstart},
                                                    scale_factor=self.scale_factor_cube,
                                                    default_physics_values=False,
                                                    mass=cube_mass,
                                                    dynamic_friction=0.2,
                                                    static_friction=0.2,
                                                    bounciness=self.cube_bounciness))    
    
        
        self.commands.extend([self.get_add_material(c_vis_mat, library="materials_low.json"),
                            {"$type": "set_visual_material",
                            "id": cube_id,
                            "material_name": c_vis_mat,
                            "object_name": "cube",
                            "material_index": 0}, 
                                ])
   
    def declare_objects(self):
        """
        Variables necessary for initializing scene objects
        """
        # add first table
        self.surface_id = self.get_unique_id()
        self.add_table(1)

        # add first cube
        self.cube_id = self.get_unique_id()    
        zstart = self.surface_record.bounds["back"]["z"]-1.5
        self.zstart = zstart
        if self.scrape_length: # long
            zstart = zstart - 2
        self.add_cube(zstart, 1)

        # add second cube for physically implausible conditions
        if self.shadow_present:
            self.shadow_cube = self.get_unique_id() 
            self.add_shadow_cube(zstart)

        # add apple obstcles for physically implausible conditions
        if self.obstacle_present:
            self.add_apple_obstacles()

        # add second table and cube if number of objects is 2
        if self.object_num > 1:
            self.surface2_id = self.get_unique_id()
            self.add_table(2)
            self.cube_id2 = self.get_unique_id()
            self.add_cube(zstart, 2)

    def place_objects_start_capture(self):
        """
        Extend commands with the screen capture and send object and recording commands
        """
        self.communicate([
             {"$type": "start_video_capture_windows",
                "output_path": str(self.capture_path.resolve()),
                "position": self.capture_position,
                "log_args": True,
                "audio_device": self.audio_device}
        ])

        self.communicate(self.commands)

    
    def teleport_motion(self, velocity, list_pos, velocity2, list_pos2):
        """
        Teleports one or two cubes along a specified trajectory with velocity profiles
        TODO: 
        - Implement changing material or mass in the middle for 2 discontinuities
        """
        impact_material = self.impact_mat1
        scrape_material = self.scrapemat1
        massofcube = self.cube_mass
        lift_cube = 0
        if self.shadow_present:
            lift_cube = 0.2

        for i,z in enumerate(list_pos2):
            contact_normals = []

            z2 = list_pos2[i]
            z = list_pos[i]
            zshadow = list_pos[len(list_pos)-i-1]
            # Three directional vectors perpendicular to the collision.
            
            for k in range(3):
                contact_normals.append(np.array([0, 1, 0]))
            

            s = self.py_impact.get_scrape_sound(velocity=np.array([0, 0, velocity2[i]]),
                                        contact_normals=contact_normals,
                                        primary_id=0,
                                        primary_material=impact_material,
                                        primary_amp=0.2,
                                        primary_mass=massofcube,
                                        secondary_id=1,
                                        secondary_material=impact_material,
                                        secondary_amp=0.5,
                                        secondary_mass=100,
                                        primary_resonance=0.2,
                                        secondary_resonance=0.1,
                                        scrape_material=scrape_material)
            
            # Teleport visual (and silent) cube
            self.communicate([{"$type": "teleport_object",
                            "position":
                                {"x": 0, "y": self.surface_record.bounds["top"]["y"]+self.cube_posy+lift_cube, "z": z},
                            "id": self.cube_id},
                        ])

            # If shadow condition, teleport secondary cube
            if self.shadow_present:
                self.communicate([{"$type": "teleport_object",
                                    "position":
                                        {"x": 0, "y": self.surface_record.bounds["top"]["y"]+self.cube_posy, "z": zshadow},
                                    "id": self.shadow_cube}])
            
            # Teleport second non-silent cube
            if self.object_num > 1:
                self.communicate([
                    {"$type": "teleport_object",
                        "position": {
                                    "x": self.surface_record.bounds["top"]["x"]+8, 
                                    "y": self.surface_record2.bounds["top"]["y"], 
                                    "z": z2},
                        "id": self.cube_id2},
                    {"$type": "play_audio_data",
                        "id": Controller.get_unique_id(),
                        "position": self.cam_point,
                        "wav_data": s.wav_str,
                        "num_frames": s.length}
                ])


    def teleport_objects(self):
        """
        Defines teleportation trajectories and calls on the teleport function twice
        (for back and forth motion)
        """
        rng = np.random.RandomState(0)
        # starting position of objects
        zstart = self.surface_record.bounds["back"]["z"]-1.5
        center = zstart+1.2+1.2
        end = zstart+2.4+2.4
        path_len = 60

        # declare position and velocity vectors for continous cube
        if self.linear_vel == 0:
            velocity = np.linspace(1.5,0.5,path_len)
        elif self.linear_vel == 1:
            # velocity = self.get_poly_velocity(path_len)
            velocity = get_poly_velocity2(path_len,0)
            # list_pos = get_poly_velocity2(path_len,0)
            # [print(x) for x in list_pos]
       

        list_pos = np.linspace(zstart,end,path_len)


        path_len2 = int(path_len/2 - math.ceil(self.discont_len/2))
        if self.linear_vel == 0:  
            pre_velocity2 = np.linspace(1.5,1,path_len2)
            between_vel = np.repeat([0.000001], self.discont_len)
            post_velocity2 = np.linspace(1,0.5,path_len2)
            velocity2 = np.hstack(( pre_velocity2,between_vel,post_velocity2)).ravel()       
        elif self.linear_vel == 1:
            vel_pathlen2 = int(path_len/2 - math.ceil(self.discont_len/2))
            # velocity2 = self.get_poly_velocity(vel_pathlen2)
            velocity2 = get_poly_velocity2(vel_pathlen2, self.discont_len)
            print("*****", np.size(velocity2))
        else:
            pre_velocity2 = np.linspace(1.5,0.3,path_len2)
            between_vel = np.repeat([0.000001], self.discont_len)
            post_velocity2 = np.linspace(1.5,0.3,path_len2)
            velocity2 = np.hstack(( pre_velocity2,between_vel,post_velocity2)).ravel()
    
         # declare position and velocity vectors for discontinous cube (add still frames in middle)
        pre_list_pos = np.linspace(zstart,center,path_len2)
        between_pos = np.repeat([center], self.discont_len)
        post_list_pos = np.linspace(center,end,path_len2)
        list_pos2 = np.hstack(( pre_list_pos,between_pos,post_list_pos)).ravel()
        

        # send the forward motion
        self.teleport_motion(velocity,list_pos,velocity2,list_pos2)

        # c.communicate({"$type": "step_physics", "frames": waiter_time})
        time.sleep(self.waiter_time)
        
        # send backward position
        self.teleport_motion(velocity,np.flip(list_pos),velocity2,np.flip(list_pos2))


    def terminate_scene(self):
        self.communicate({"$type": "stop_video_capture"})

        self.communicate({"$type": "terminate"})
    
    def apply_force_visual_audio_cube(self, visual_cubeid, audio_cubeid):
        rng = np.random.RandomState(0)
        self.cube_audio_material = AudioMaterial.wood_medium
        cube_audio = ObjectAudioStatic(name="cube",
                                        object_id=audio_cubeid,
                                        mass=self.cube_mass,
                                        bounciness=self.cube_bounciness,
                                        amp=0.2,
                                        resonance=0.25,
                                        size=1,
                                        material=self.cube_audio_material)
        # Reset PyImpact and add it to the list of add-ons so that it automatically generates audio.
        self.py_impact = PyImpact(rng=rng, static_audio_data_overrides={audio_cubeid: cube_audio}, initial_amp=0.9)
        self.add_ons.append(self.py_impact)
        

        forth_move_commands = [
                          {"$type": "apply_force_magnitude_to_object",
                           "magnitude": 0.12,
                           "id": audio_cubeid},
                           {"$type": "apply_force_magnitude_to_object",
                           "magnitude": 0.12,
                           "id": audio_cubeid},
        ]
        self.communicate(forth_move_commands)
        for i in range(40):
            self.communicate([])
        # Apply a lateral force to start scraping.
        # self.communicate(forth_move_commands)
        self.communicate({"$type": "apply_force_magnitude_to_object",
                                        "magnitude": 0.6,
                                        "id": visual_cubeid})

        for i in range(40):
            self.communicate([])

        self.communicate([{"$type": "apply_force_magnitude_to_object",
                           "magnitude": 0.15,
                           "id": audio_cubeid},
                           {"$type": "apply_force_magnitude_to_object",
                           "magnitude": 0.15,
                           "id": audio_cubeid},
                           
                           ])

        for i in range(120):
            self.communicate([])
        
        back_move_commands = [
                          {"$type": "apply_force_magnitude_to_object",
                           "magnitude": -0.15,
                           "id": audio_cubeid},
                           {"$type": "apply_force_magnitude_to_object",
                           "magnitude": -0.15,
                           "id": audio_cubeid},
        ]
        self.communicate(back_move_commands)
        for i in range(30):
            self.communicate([])
        # Apply a lateral force to start scraping.
        self.communicate(back_move_commands)
        self.communicate({"$type": "apply_force_magnitude_to_object",
                                        "magnitude": -0.6,
                                        "id": visual_cubeid})

        for i in range(50):
            self.communicate([])

        self.communicate([{"$type": "apply_force_magnitude_to_object",
                           "magnitude": -0.15,
                           "id": audio_cubeid},
                           {"$type": "apply_force_magnitude_to_object",
                           "magnitude": -0.15,
                           "id": audio_cubeid},
                           
                           ])
        
        for i in range(120):
            self.communicate([])
        
        # Remove PyImpact from the list of add-ons.
        self.add_ons.pop(-1)

    def apply_forces(self):

        self.apply_force_visual_audio_cube(self.cube_id, self.cube_id2)
         # Define audio for the cube.
     
         # Define audio for the cube.
     

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple argument parser")
    parser.add_argument("-c", action="store", dest="config_file", default="config.ini")
    result = parser.parse_args()
    directory_cfg = str(Path.cwd().joinpath(result.config_file).resolve())

    #Read config.ini file
    config_object = ConfigParser()
    config_object.read(directory_cfg)
    configs = config_object["all"]


    scrapesObj = DiscontScrapesDemo(configs)
    scrapesObj.initialize_scene()
    scrapesObj.declare_objects()
    scrapesObj.place_objects_start_capture()
    if int(configs["physics_based"]):
       scrapesObj.apply_forces()
    else:
        scrapesObj.teleport_objects()
    scrapesObj.terminate_scene()
