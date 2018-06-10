#! /usr/bin/python3
from random import *
import os,sys
run = os.system

##BEGIN 这些需要修改

dataset = '20171229_nanian_130255'
#dataset = '20180103_nanian_103247'
target_dir = "/home/jcvb/work/PublicCourse/perception/fafafa" 
data_dir = "/home/jcvb/work/PublicCourse/data/%s"%(dataset)
config_dir = "/home/jcvb/work/PublicCourse/perception/config"

##END

run_cmd = "bazel run -c opt //perception:evaluation_viewer_main  -- --eval_result_file `bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir %s 2>&1 | grep folder | awk '{ print $4 }'`/pony_data.result"%(config_dir)
run_only_cmd = "bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir %s "%config_dir
clear_cmd1 = "rm  %s/label/VelodyneDevice32c/*"%(target_dir)
clear_cmd2 = "rm  %s/select/VelodyneDevice32c/*"%(target_dir)


mn = 488
mx = 488

def choose(x):
        run('cp %s/label/VelodyneDevice32c/%d.label %s/label/VelodyneDevice32c/'%(data_dir,x,target_dir))
        run('cp %s/select/VelodyneDevice32c/%d.txt %s/select/VelodyneDevice32c/'%(data_dir,x,target_dir))

def sample():
        x = randint(mn,mx)
        run('cp %s/label/VelodyneDevice32c/%d.label %s/label/VelodyneDevice32c/'%(data_dir,x,target_dir))
        run('cp %s/select/VelodyneDevice32c/%d.txt %s/select/VelodyneDevice32c/'%(data_dir,x,target_dir))

def clear():
        run(clear_cmd1)
        run(clear_cmd2)

if len(sys.argv) > 1 :
        if sys.argv[1] == 'all' or sys.argv[1] == 'a' :
                clear()
                for i in range(mn,mx+1):
                        choose(i)
                run(run_only_cmd)
        elif sys.argv[1] == 'sample' or sys.argv[1] == 's':
                clear()
                sample()
                run(run_only_cmd)
                run(run_cmd)
        else:
                clear()
                print('choose%d'%int(sys.argv[1]))
                choose(int(sys.argv[1]))
                run(run_cmd)
else:
        run(run_cmd)
