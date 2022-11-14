import numpy as np
from tdw.controller import Controller
from tdw.tdw_utils import TDWUtils
from tdw.librarian import ModelLibrarian
from tdw.add_ons.third_person_camera import ThirdPersonCamera
from tdw.add_ons.audio_initializer import AudioInitializer
from tdw.add_ons.py_impact import PyImpact
from tdw.add_ons.physics_audio_recorder import PhysicsAudioRecorder
from tdw.physics_audio.object_audio_static import ObjectAudioStatic
from tdw.physics_audio.audio_material import AudioMaterial
from tdw.backend.paths import EXAMPLE_CONTROLLER_OUTPUT_PATH
from time import time

""":field
Record scrape sounds.
"""

c = Controller(launch_build=True)
# Camera for first and second sequence
camera = ThirdPersonCamera(position={"x": 1.0, "y": 1.27, "z": -1.66},
# Camera for third sequence
#camera = ThirdPersonCamera(position={"x": 1.0, "y": 1.27, "z": -1.66},
                           look_at={"x": 0, "y": 0.5, "z": 0},
                           avatar_id="a")
audio = AudioInitializer(avatar_id="a")

# Set a random number generator with a hardcoded random seed so that the generated audio will always be the same.
# If you want the audio to change every time you run the controller, do this instead: `py_impact = PyImpact()`.
rng = np.random.RandomState(1)
py_impact = PyImpact(rng=rng)

#recorder = PhysicsAudioRecorder()
path = EXAMPLE_CONTROLLER_OUTPUT_PATH.joinpath("scrape")
print(f"Audio will be saved to: {path}")
c.add_ons.extend([camera, audio, py_impact])
# c.communicate(c.get_add_scene(scene_name="tdw_room"))
c.communicate(TDWUtils.create_empty_room(12, 12))
c.communicate({"$type": "rotate_directional_light_by", "angle": -10, "axis": "pitch", "index": 0})
# c.communicate(TDWUtils.create_empty_room(12, 12))
lib_core = ModelLibrarian("models_core.json")
lib_flex = ModelLibrarian("models_flex.json")
cube_mass = 1
cube_bounciness = 0.1
#force = 3
for scrape_surface_model_name in ["quatre_dining_table"]:
    surface_record = lib_core.get_record(scrape_surface_model_name)
    for cube_audio_material, cube_visual_material in zip([AudioMaterial.wood_medium, AudioMaterial.ceramic],
                                                         ["wood_beech_honey", "ceramic_raw_striped"]):
        for fric, force in zip([0.25,0.3,0.5],[2,3,6]):
            # Add the surface.
            surface_id = c.get_unique_id()
            cube_id = c.get_unique_id()
            commands = c.get_add_physics_object(model_name=scrape_surface_model_name,
                                                library="models_core.json",
                                                object_id=surface_id,
                                                kinematic=True,
                                                scale_factor={"x": 2, "y": 1, "z": 4})
            # Add the cube just above the top of the surface.
            commands.extend(c.get_add_physics_object(model_name="cube",
                                                     library="models_flex.json",
                                                     object_id=cube_id,
                                                     position={"x": 0,
                                                               "y": surface_record.bounds["top"]["y"],
                                                               "z": surface_record.bounds["back"]["z"] -0.1},
                                                     scale_factor={"x": 0.1, "y": 0.04, "z": 0.1},
                                                     scale_mass=False,
                                                     default_physics_values=False,
                                                     mass=cube_mass,
                                                     dynamic_friction=fric,
                                                     static_friction=fric,
                                                     bounciness=cube_bounciness))
            #cube_id2 = c.get_unique_id()

            # commands.extend(c.get_add_physics_object(model_name="cube",
            #                                          library="models_flex.json",
            #                                          object_id=cube_id2,
            #                                          position={"x": 0.5,
            #                                                    "y": surface_record.bounds["top"]["y"],
            #                                                    "z": surface_record.bounds["back"]["z"] },
            #                                          scale_factor={"x": 0.1, "y": 0.04, "z": 0.1},
            #                                          default_physics_values=False,
            #                                          mass=cube_mass,
            #                                          dynamic_friction=fric,
            #                                          static_friction=fric,
            #                                          bounciness=cube_bounciness))
            commands.extend([c.get_add_material(cube_visual_material, library="materials_low.json"),
                             {"$type": "set_visual_material",
                              "id": cube_id,
                              "material_name": cube_visual_material,
                              "object_name": "cube",
                              "material_index": 0},
                             {"$type": "set_aperture", "aperture": 5.0},
                             {"$type": "set_field_of_view", "field_of_view": 60, "avatar_id": "a"},
                             {"$type":"set_target_framerate","framerate": 60},
                             {"$type": "set_shadow_strength", "strength": 0.0},
                             {"$type": "set_render_quality", "render_quality":1},
                             {"$type": "set_screen_size", "width": 1920, "height": 1080}])
            # Define audio for the cube.
            cube_audio = ObjectAudioStatic(name="cube",
                                           object_id=cube_id,
                                           mass=cube_mass,
                                           bounciness=cube_bounciness,
                                           amp=0.8,
                                           resonance=0.25,
                                           size=1,
                                           material=cube_audio_material)

            # cube_audio2 = ObjectAudioStatic(name="cube",
            #                                object_id=cube_id2,
            #                                mass=cube_mass,
            #                                bounciness=cube_bounciness,
            #                                amp=0.2,
            #                                resonance=0.02,
            #                                size=1,
            #                                material=cube_audio_material)
            # Reset PyImpact.
            py_impact.reset(static_audio_data_overrides={cube_id: cube_audio}, initial_amp=0.9)
            # py_impact.reset(static_audio_data_overrides={cube_id: cube_audio2}, initial_amp=0.9)
            c.communicate(commands)
            #recorder.start(path=path.joinpath(f"{scrape_surface_model_name}_{cube_audio_material.name}_{fric}.wav"))
            # Apply a lateral force to start scraping.
            c.communicate({"$type": "apply_force_magnitude_to_object",
                           "magnitude": force,
                           "id": cube_id})
            # c.communicate({"$type": "apply_force_magnitude_to_object",
            #                "magnitude": force,
            #                "id": cube_id2})
            while not recorder.done:
                t0 = time()
                c.communicate([])
                dt = time() - t0
            # Destroy the objects to reset the scene.
            c.communicate([{"$type": "destroy_object",
                            "id": cube_id},
                           # {"$type": "destroy_object",
                           #  "id": cube_id2},
                           {"$type": "destroy_object",
                            "id": surface_id}])
c.communicate({"$type": "terminate"})
