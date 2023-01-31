#!/usr/bin/env python3
# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from robomaster import robot, config
from rmtt_core import RoboMasterTelloTalent as rmtt

if __name__ == '__main__':
    config.LOCAL_IP_STR = rmtt.get_local_ip()
    print("Computer local ip:", config.LOCAL_IP_STR)


    tl_drone = robot.Drone()
    tl_drone.initialize()
    version = tl_drone.get_sdk_version()
    print("Drone SDK Version: {0}".format(version))
    # 切换飞行器WiFi模式为组网模式，指定路由器SSID和密码
    if version == None:
        print("Please check the connection !")
    else:
        n = len(sys.argv)
        if n == 3:
            ssid = sys.argv[1]
            password = sys.argv[2]
        else:
            ssid = "TIANBOT-be8-5G"
            password = "www.tianbot.com"

        is_configured = tl_drone.config_sta(ssid, password)
        if is_configured:
            print("Wifi configured to ssid: {0}, please switch TT to router mode".format(ssid))
        else:
            print("Failed to configure RMTT sta. Please check logger.")
        
        tl_drone.close()

    os._exit(0)
