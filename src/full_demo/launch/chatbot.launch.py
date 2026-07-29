# Copyright 2026 Aapo2001
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tts_node = Node(
        package='tts_package',
        executable='tts_node',
        name='tts_node',
        output='screen',
        additional_env={
            'CUDA_VISIBLE_DEVICES': '-1',
            'TF_CPP_MIN_LOG_LEVEL': '2',
        },
    )
    llm_node = Node(
        package='llm_package',
        executable='llm_node',
        name='llm_node',
        output='screen',
    )
    sst_node = Node(
        package='sst_package',
        executable='sst_node',
        name='sst_node',
        output='screen',
    )

    return LaunchDescription(
        [
            tts_node,
            llm_node,
            sst_node,
        ]
    )
