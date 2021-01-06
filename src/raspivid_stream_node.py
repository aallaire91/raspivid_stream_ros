# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import other libraries
import os
import time
import yaml
import numpy as np
import subprocess as sp


def set_config(filename,hostname):
    set_cfg_cmd = f"scp {filename} pi@{hostname}:/home/pi/raspivid_stream/stream_cfg.yml"
    os.system(set_cfg_cmd)


def load_config(filename):
    cfg_file = open(filename,'r')
    cfg = yaml.load(cfg_file,yaml.FullLoader)
    return cfg 


def start_stream(hostname):
    ssh_prefix = f"ssh pi@{hostname}"
    start_stream_cmd = ssh_prefix + " \'start_stream\'"
    os.system(start_stream_cmd)


def stop_stream(hostname):
    ssh_prefix = f"ssh pi@{hostname}"
    stop_stream_cmd = ssh_prefix + " \'stop_stream\'"
    os.system(stop_stream_cmd)


def start_decode_process(pipe):
    ffmpeg_cmd = ["ffmpeg",
	            "-i", pipe,
	            "-pix_fmt", "bgr24",
	            "-vcodec", 'rawvideo',
	            "-an", "-sn",
	            "-f", "image2pipe",
	            "-"]
    proc = sp.Popen(ffmpeg_cmd, stdout=sp.PIPE, bufsize=10**8) 
    time.sleep(1)
    return proc


def start_stream_server_process(pipe,port):
    rel_script_path = os.path.join("raspivid_stream","scripts","receive_stream_remote.sh")
    script_path = os.path.join(rospack.get_path('raspivid_stream_ros'),script_path)
    srv_cmd = ["/bin/bash",script_path,str(pipe),str(port)]
    proc = sp.Popen(srv_cmd)
    time.sleep(0.1)
    return proc


def raspivid_stream():

    rospy.init_node('raspivid_stream',anonymous=True)

    hostname = rospy.get_param('hostname')
    config_filename = rospy.get_param('config_file')

    pub = rospy.Publisher(f'{hostname}/image/',Image,queue_size=10)

    bridge = CvBridge()

    # load config file
    dev_cfg = load_config(config_filename)
    PIPE = os.path.join("/tmp/",hostname)
    PORT = int(dev_cfg["netcat"]["port"])
    WIDTH = int(dev_cfg["video"]["width"])
    HEIGHT = int(dev_cfg["video"]["height"])
    FPS = int(dev_cfg["video"]["fps"])

    # always stop stream first
    stop_stream()

    # set config file (i.e. send to device)
    set_config()

    # start stream server
    start_stream_server_process(PIPE,PORT)
    
    # start decoder
    dec = start_decode_process(PIPE)
 
    # start stream on rpi
    start_stream()

    r = rospy.Rate(FPS) 
    avg_dt = 0
    start = time.time()
    while not rospy.is_shutdown():

        image = np.fromstring(dec.stdout.read(int(WIDTH * HEIGHT * 3)),
                              dtype='uint8')  # read bytes of single frames
        if image.size > 0:
            dt = time.time() - start
            start = time.time()
            avg_dt += (dt - avg_dt) * 0.03

            image_cv = image.reshape((HEIGHT,WIDTH,3))
            pub.publish(bridge.cv2_to_imgmsg(image_cv))

	r.sleep()




if __name__ == '__main__':
  try:
    raspivid_stream()
  except rospy.ROSInterruptException:
    pass
