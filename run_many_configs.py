from configparser import ConfigParser
import os
import time
#Get the configparser object
config_object = ConfigParser()

def physics_videos():
    lightx = 0
    shadow = 0
    linear_v = 1
    discont_l = 1
    physicsb = 1
    for lightx in range(2):
        for cam_v in range(4):
            # for discont_l in [2,4,6]:
                # if physicsb == 1:
                    
                #Assume we need 2 sections in the config file, let's call them USERINFO and SERVERCONFIG
                config_object["all"] = {
                    "lightx": lightx,
                    "linear_vel":linear_v,
                    "shadow" : shadow,
                    "obstacle" : shadow,
                    "scrape_length" : 0,
                    "cam_view" : cam_v,
                    "waiter_time" : 0.8,
                    "physics_based" : physicsb,
                    "discont_len" : discont_l,
                    "mass" : 1,
                    "secondmass" : 1,
                    "table1mat" : 1,
                    "object_num" : 2,
                    "high_def" : True ,
                    "continuity_obj1" : 0,
                    "continuity_obj2" : 0,
                    "record_obj1 " : False,
                    "record_obj2" : False,
                    "table2mat" : 2,
                    "scrape2" : 2,
                    "scrape1" : 2,
                    "cubemat" : 2,
                    "cube2mat" : 2,
                    "change_mass_mid" : False,
                    "change_mat_mid" : False,
                    "cube_size" : 0,
                    "audio_device":"Stereo Mix (Realtek Audio)",
                    "title_bar_height": 12,	
                    "monitor_index": 0,

                }



                #Write the above sections to config.ini file
                with open('config.ini', 'w') as conf:
                    config_object.write(conf)

                cmd =  'python discont_scrapes_merged.py -c config.ini'
                print(cmd)
                os.system(cmd)
                time.sleep(5)


linear_v = 3


def teleport_videos():
    physicsb = 0
    for shadow in range(2):
        for lightx in range(2):
            for cam_v in range(4):
                for discont_l in [2,4,6]: 
                    #Assume we need 2 sections in the config file, let's call them USERINFO and SERVERCONFIG
                    config_object["all"] = {
                        "lightx": lightx,
                        "linear_vel":linear_v,
                        "shadow" : shadow,
                        "obstacle" : shadow,
                        "scrape_length" : 0,
                        "cam_view" : cam_v,
                        "waiter_time" : 0.8,
                        "physics_based" : physicsb,
                        "discont_len" : discont_l,
                        "mass" : 1,
                        "secondmass" : 1,
                        "table1mat" : 1,
                        "object_num" : 2,
                        "high_def" : True ,
                        "continuity_obj1" : 0,
                        "continuity_obj2" : 0,
                        "record_obj1 " : False,
                        "record_obj2" : False,
                        "table2mat" : 2,
                        "scrape2" : 2,
                        "scrape1" : 2,
                        "cubemat" : 2,
                        "cube2mat" : 2,
                        "change_mass_mid" : False,
                        "change_mat_mid" : False,
                        "cube_size" : 0,
                        "audio_device":"Stereo Mix (Realtek Audio)",
                        "title_bar_height": 12,	
                        "monitor_index": 0,

                    }



                    #Write the above sections to config.ini file
                    with open('config.ini', 'w') as conf:
                        config_object.write(conf)

                    cmd =  'python discont_scrapes_merged.py -c config.ini'
                    print(cmd)
                    os.system(cmd)
                    time.sleep(5)

import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple argument parser")
    parser.add_argument("-physics", action="store", dest="call_physics", default=False)
    parser.add_argument("-teleport", action="store", dest="call_teleport", default=False)
    result = parser.parse_args()
    if result.call_physics:
        physics_videos()
    
    if result.call_teleport:
        teleport_videos()
