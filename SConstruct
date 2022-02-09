import os
import sys
import rtconfig

RTT_ROOT = os.path.normpath(os.getcwd() + '/vendor/rt-thread/')
sys.path = sys.path + [os.path.join(RTT_ROOT, 'tools')]
try:
    from building import *
except:
    print('Cannot found RT-Thread root directory, please check RTT_ROOT')
    print(RTT_ROOT)
    exit(-1)

TARGET = 'rtthread-gd32e1xx.' + rtconfig.TARGET_EXT

DefaultEnvironment(tools = [])
env = Environment(tools = ['mingw'],
    AS = rtconfig.AS,     ASFLAGS = rtconfig.AFLAGS,
    CC = rtconfig.CC,     CCFLAGS = rtconfig.CFLAGS,
    AR = rtconfig.AR,     ARFLAGS = '-rc',
    LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS,
    CPPPATH = ['./config']
)
env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

Export('RTT_ROOT')
Export('rtconfig')

# prepare building environment
objs = PrepareBuilding(env, RTT_ROOT, has_libcpu=False)

AddOption('--showgroup',
    dest = 'showgroup',
    action = 'store_true',
    default = False,
    help = 'show group'
)

if GetOption('showgroup'):
    g = open('.group', 'w')
    for Group in Projects:
        g.write(Group['name'])
        paths = [os.path.normpath(i) for i in Group['CPPPATH']]
        for path in paths:
            g.write(' ' + os.path.normpath(path))
        g.write('\n')
    g.close()
    exit(0)
    
# make a building
DoBuilding(TARGET, objs)
