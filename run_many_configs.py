from configparser import ConfigParser
import os
import time
#Get the configparser object
config_object = ConfigParser()


for shadow in range(2):
    physicsb = 0
    for linear_v in range(2):
        for cam_v in range(4):
            for discont_l in [2,3,4,6,8]:
                #Assume we need 2 sections in the config file, let's call them USERINFO and SERVERCONFIG
                config_object["all"] = {
                    "linear_vel":linear_v,
                    "shadow" : shadow,
                    "obstacle" : 0,
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