"""
"""
import pandas as pd
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

class TeleportController(Controller):
    def __init__(self, velocity_vector, position_vector):
        super().__init__()

        # name of output video
        self.run_type = "teleport_example_output"

        # necessary librarian imports
        Controller.MODEL_LIBRARIANS["models_core.json"] = ModelLibrarian("models_core.json")
        Controller.MODEL_LIBRARIANS["models_flex.json"] = ModelLibrarian("models_flex.json")
      
        # velocity for teleport
        self.velocity_vector = velocity_vector
        self.position_vector = position_vector

        # table parameters
        self.scrape_surface_model_name = 'quatre_dining_table'
        self.table1_scale = {"x": 1, "y": 1, "z": 1.5}
        self.surface_record = Controller.MODEL_LIBRARIANS["models_core.json"].get_record(self.scrape_surface_model_name)

        # cube parameters
        self.impact_mat1 = "glass_1"
        self.cube_visual_material = 'metal_cast_iron'
        self.scrape_mat =  ScrapeMaterial.ceramic
        self.scale_factor_cube = {"x": 0.10, "y": 0.04, "z": 0.10}
        self.cube_mass = 1
        self.cube_bounciness = 0 

        # camera parameters
        self.cam_point = {"x": 1, "y": 1.5, "z": 2}
        self.look_at = {"x": -0.4, "y": 0.5, "z": 0}

        # recording parameters
        self.window_w = 1540
        self.window_h = 880
        self.audio_device = "Stereo Mix (Realtek Audio)"
        self.capture_position = TDWUtils.get_expected_window_position(window_width=self.window_w,
                                                                      window_height=self.window_h,
                                                                      title_bar_height=12,
                                                                      monitor_index=0)
        # Initialize PyImpact.
        rng = np.random.RandomState(0)
        self.py_impact = PyImpact(rng=rng)
        self.commands = []
        self.capture_path = EXAMPLE_CONTROLLER_OUTPUT_PATH.joinpath("crossmodal_illusions").joinpath(self.run_type + ".mp4")
        
    def add_table(self):
        """
     
        """
       
        self.commands = self.get_add_physics_object(model_name=self.scrape_surface_model_name,
                                                library="models_core.json",
                                                object_id=self.surface_id,
                                                kinematic=True,
                                                scale_factor=self.table1_scale)

    def add_cube(self):
        """
        Place one or two cubes
        """
        c_vis_mat = self.cube_visual_material
        lift_cube = 0


        cube_id = self.cube_id
        self.xpos = 0
        self.ypos = self.surface_record.bounds["top"]["y"]+ lift_cube+0.05
        cube_mass = self.cube_mass
        zstart = self.zstart + 1

        self.commands.extend(self.get_add_physics_object(model_name="cube",
                                                    library="models_flex.json",
                                                    object_id=cube_id,
                                                    position={"x": self.xpos,
                                                            "y": self.ypos,
                                                            "z": zstart},
                                                    scale_factor=self.scale_factor_cube,
                                                    default_physics_values=False,
                                                    mass=cube_mass,
                                                    dynamic_friction=.02,
                                                    static_friction=.025,
                                                    bounciness=self.cube_bounciness))    
    
        
        self.commands.extend([self.get_add_material(c_vis_mat, library="materials_low.json"),
                            {"$type": "set_visual_material",
                            "id": cube_id,
                            "material_name": c_vis_mat,
                            "object_name": "cube",
                            "material_index": 0}, 
                                ])

    def declare_objects(self):
        self.surface_id = self.get_unique_id()
        self.add_table()

        zstart = self.surface_record.bounds["back"]["z"]-1
        self.zstart = zstart

        self.cube_id = self.get_unique_id()  
        self.add_cube()


    def place_objects_start_capture(self):
        """
        Extend commands with the screen capture and send object and recording commands
        """
        self.communicate([
             {"$type": "start_video_capture_windows",
                "output_path": str(self.capture_path.resolve()),
                "position": self.capture_position,
                "log_args": True,
                "audio_device": self.audio_device}, 
        ])

        self.resp = self.communicate(self.commands)

   


    def move_cube(self):

        
        self.list_pos = np.linspace(self.zstart+1, self.zstart + 2.5, len(self.position_vector.values))
        self.list_pos = np.arange(self.zstart+1,len(self.velocity_vector.values))*-0.005+1.5
        impact_material = self.impact_mat1
        scrape_material = self.scrape_mat
        massofcube = self.cube_mass
        lift_cube = 0


        for i in range(len(self.position_vector.values)):
            contact_normals = []

            
            # xyz = {"x":0, "y":self.surface_record.bounds["top"]["y"]+self.cube_posy+lift_cube, "z": tuple(self.position_vector.values[i])[2]}
            
            x = tuple(self.position_vector.values[i])[0]
            y = tuple(self.position_vector.values[i])[1]
            # y =
            z = tuple(self.position_vector.values[i])[2]
    
            # Three directional vectors perpendicular to the collision.
            
            for k in range(3):
                contact_normals.append(np.array([0, 1, 0]))
            

            s = self.py_impact.get_scrape_sound(velocity=np.array([tuple(self.velocity_vector.values[i])[0], tuple(self.velocity_vector.values[i])[1], tuple(self.velocity_vector.values[i])[2]]),
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
            
            # Teleport cube
            self.communicate([{"$type": "teleport_object",
                            "position":
                                {"x": x, "y": y, "z": z},
                            "id": self.cube_id},
                        ])
                        
    def initialize_scene(self):
        """
        Initialize audio, camera, and window settings
        """

        audio = AudioInitializer(avatar_id="a", framerate=60)
        print(f"Video will be saved to: {self.capture_path}")
        # set camera position depending on number of objects
      
        camera = ThirdPersonCamera(position=self.cam_point,
                                    look_at=self.look_at,
                                    avatar_id="a")
        # Set a random number generator with a hardcoded random seed so that the generated audio will always be the same.
        # If you want the audio to change every time you run the controller, do this instead: `py_impact = PyImpact()`.
        rng = np.random.RandomState(0)

        # if not self.lightx:
        hdri_skybox = "bergen_4k"
        interior_scene_lighting = InteriorSceneLighting(hdri_skybox=hdri_skybox)
        self.add_ons.extend([camera, audio,interior_scene_lighting])
        

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
                     "aperture": 100.0},
                    {"$type": "set_shadow_strength",
                     "strength": 1.0}]
                    
       
        commands.extend([
            {"$type": "simulate_physics",
                "value": False},])
        self.resp = self.communicate(commands)
    


    def reset_scene(self):
        self.communicate({"$type": "stop_video_capture"})

        self.communicate({"$type": "terminate"})



if __name__ == '__main__':
    # read csv file for velocity vector (physics simulated)
    velocity_vector = pd.read_csv('velocity_vector.csv')
    position_vector = pd.read_csv('position_vector.csv')
    print(len(velocity_vector.values))

    teleportObj = TeleportController(velocity_vector, position_vector)
    teleportObj.initialize_scene()
    teleportObj.declare_objects()
    teleportObj.place_objects_start_capture()

    
    
    teleportObj.move_cube()
    teleportObj.reset_scene()
