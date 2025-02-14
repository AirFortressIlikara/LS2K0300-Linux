###
 # @Author: Ilikara 3435193369@qq.com
 # @Date: 2025-02-12 19:05:17
 # @LastEditors: Ilikara 3435193369@qq.com
 # @LastEditTime: 2025-02-13 22:14:22
 # @FilePath: /LS2K0300-Linux/cmd.sh
 # @Description: 
 # 
 # Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
###
#!/bin/bash

export PATH=/home/ilikara/loongson/gcc-13.2.0-loongarch64-linux-gnu/bin:$PATH 
make ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- -j $nproc
cksum vmlinux
