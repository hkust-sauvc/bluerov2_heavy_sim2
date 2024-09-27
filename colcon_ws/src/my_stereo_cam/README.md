export GZ_SIM_RESOURCE_PATH=/home/orca4/colcon_ws/src/my_stereo_cam/models:$GZ_SIM_RESOURCE_PATH

      <uri>model://stereo_camera</uri>
      <pose>0 10 0 0 0 0</pose>
    </include>
 <model name="block1">
            <pose>3 0.0 0.0 0 0 0</pose>
            <link name="link">
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.5 0.5 0.5</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>1 0 0 1</ambient> <!-- Red -->
                    </material>
                </visual>
            </link>
        </model>
