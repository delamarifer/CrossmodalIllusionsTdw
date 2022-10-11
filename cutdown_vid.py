import os
import pathlib as path

imgs_dir2 = os.path.join(path.Path.home(), "Desktop/discont_illusion_testing/pre_videos/")
out_dir = os.path.join(path.Path.home(), "Desktop/discont_illusion_testing/videos/")
for vidfile in os.listdir(imgs_dir2):
     if os.path.join(imgs_dir2, vidfile).endswith(".mp4"):
        outfile = out_dir + vidfile
        if "shadow" in vidfile:
          cmd = "ffmpeg -ss 2.1 -i " + imgs_dir2 + vidfile + " -c:v libx264 -c:a aac " + outfile
        else:
          cmd = "ffmpeg -ss 0.5 -i " + imgs_dir2 + vidfile + " -c:v libx264 -c:a aac " + outfile
        os.system(cmd)
        print(cmd)

for vidfile in os.listdir(out_dir):
     if os.path.join(out_dir, vidfile).endswith(".mp4"):
        cmd = "ffmpeg -stream_loop 1 -i " +  os.path.join(out_dir, vidfile) + ' -c copy ' + os.path.join(out_dir) + "looped_" + vidfile
        os.system(cmd)
        print(cmd)