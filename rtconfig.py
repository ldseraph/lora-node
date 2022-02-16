import os
import re

APP_NAME = ''

stream = os.popen('git describe --tags --dirty --match=v* 2> /dev/null || echo v0')
VERSION = stream.read().rstrip("\n")
stream.close()

rtconfig_file = './rtconfig.h'

if os.path.exists(rtconfig_file):
    f = open(rtconfig_file)
    rtconfig = f.read()
    APPS = re.findall(r"(?<=APP_COMPONENTS_)\S+",rtconfig)
    for app in APPS:
        APP_NAME = APP_NAME + app.lower() + '_'
else:
    APP_NAME = 'unknown_'

APP_NAME = APP_NAME + 'app_' + VERSION

EXEC_PATH  = './'
if os.path.exists('../gcc'):
    EXEC_PATH  = '../gcc/bin'

ARCH       = 'arm'
CPU        = 'cortex-m4'
CROSS_TOOL = 'gcc'
PLATFORM   = 'gcc'
BUILD      = 'release'

PREFIX = 'arm-none-eabi-'
CC = PREFIX + 'gcc'
AS = PREFIX + 'gcc'
AR = PREFIX + 'ar'
LINK = PREFIX + 'gcc'
TARGET_EXT = 'elf'
SIZE = PREFIX + 'size'
OBJDUMP = PREFIX + 'objdump'
OBJCPY = PREFIX + 'objcopy'
TARGET = APP_NAME + '.' +TARGET_EXT

DEVICE = ' -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter -fomit-frame-pointer -ffast-math -ftree-vectorize '
CFLAGS = DEVICE + ' -Dgcc -std=gnu99 ' # -D' + PART_TYPE
CPPFLAGS = DEVICE + ' -Dgcc -std=c++11 ' # -D' + PART_TYPE
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '
#  -u _printf_float
LFLAGS = DEVICE + ' -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -u Reset_Handler -T vendor/gd32e10x-bsp/linker/gd32_flash.ld -Wl,-cref,-Map='+APP_NAME+'.map '

CPATH = ''
LPATH = ''

if BUILD == 'debug':
    CFLAGS += ' -Og -gdwarf-2 -g'
    AFLAGS += ' -gdwarf-2'
else:
    CFLAGS += ' -O2 -Os'

POST_ACTION = OBJCPY + ' -O binary $TARGET '+ APP_NAME+'.bin && ' + OBJDUMP + ' -S $TARGET > ' + APP_NAME+'.objdump && ' + SIZE + ' $TARGET'
