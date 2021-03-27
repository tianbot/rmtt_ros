#!/usr/bin/env python3
# -*-coding:utf-8-*-
# Copyright (c) 2021 Tianbot.
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

import logging
import time
import robomaster
from robomaster import robot
from multi_robomaster import tool


if __name__ == '__main__':
    #robomaster.enable_logging_to_file()
    logger_name = "multi_robot"
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    robot_host_list={}
    robot_sn_dict={}

    client = tool.TelloClient()
    client._conn.local_ip = '0.0.0.0'
    client.start()

    robot_host_list=client.scan_multi_robot(1)
    for host in robot_host_list:
        proto = tool.TelloProtocol("sn?", host)
        client.send(proto)
        logger.info("send cmd")

    cur_time = time.time()
    while client.queue.qsize() < 1:
        if time.time() - cur_time > 10:
            raise Exception("get sn timeout")

    while not client.queue.empty():
        proto = client.queue.get()
        if proto.text is None:
            raise Exception("recv data is None")
        robot_sn_dict[proto.text] = proto.host
        time.sleep(0.1)
        logger.info("get host")


    client.close()
    # change the robot_num that you want to scan
    #multi_drone.initialize(robot_num=4)
    #drone_ip_list = multi_drone._get_sn(timeout=10)
    #for host in robot_host_list:
    #    print("scan result: host:{0} sn{1}".format(robot_host_list,robot_sn_dict))


    for sn in robot_sn_dict:
        print("scan result: sn:{0} host{1}".format(sn,robot_sn_dict[sn]))
    
