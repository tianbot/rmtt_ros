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

# ROS
import rospy

from rmtt_core import RoboMasterTelloTalent

if __name__ == '__main__':
    robomaster_node_name = rospy.get_name()
    robomaster_node_namespace = rospy.get_namespace()
    rospy.loginfo('Node with namespace = {}, name = {} started'.format(
        robomaster_node_namespace, robomaster_node_name))
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    with RoboMasterTelloTalent() as rmtt_drone:
        rmtt_drone.run()
