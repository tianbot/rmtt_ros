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

import logging, time, sys, getopt
import robomaster
from robomaster import robot
from multi_robomaster import tool


if __name__ == '__main__':
    help_str ='rmtt_scan_ip.py -n <num of drones>' 
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h:n:", ["help", "num=1"])
    except getopt.GetoptError:
        print(help_str)
        sys.exit(2)

    if not args:
        num = 1
        print('Searching for one drone...')
    elif not opts:
        print('Please use the following cmd to specify the number of drones: \n',
        '\033[95m'+help_str+'\033[0m')
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print(help_str)
            sys.exit()
        elif opt in ("-n", "--num"):
            num = int(arg)
        else:
            print(help_str)
            sys.exit(2)
    #robomaster.enable_logging_to_file()
    logger_name = "multi_robot"
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    robot_host_list={}
    robot_sn_dict={}

    client = tool.TelloClient()
    client._conn.local_ip = '0.0.0.0'
    client.start()

    robot_host_list=client.scan_multi_robot(num)

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

    for sn in robot_sn_dict:
        print("scan result: sn:{0} host{1}".format(sn,robot_sn_dict[sn]))
    
