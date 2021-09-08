#!/usr/bin/env python3 

import os
import rospkg
import glob
import sys


def main(string):
    dir_name = rospkg.RosPack().get_path('mrpp_sumo')
    config_files = glob.glob(dir_name + '/config/{}*.yaml'.format(string))
    count = 0
    for conf in config_files:
        path = conf.split('/')
        name = path[-1].split('.')[0]
        print ('Processing {}'.format(name))
        os.system('python3 {}/scripts/post_process/sim_visualize.py {}'.format(dir_name, name))
        count += 1
        print ('{} Done {}'.format(count, conf))

if __name__ == '__main__':
    main(sys.argv[1])