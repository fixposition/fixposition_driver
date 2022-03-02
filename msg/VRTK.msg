Header header
string pose_frame                               # frame of the pose values [pose, quaternion]
string kin_frame                                # frame of the kinematic values [linear/angular velocity, acceleration]
geometry_msgs/PoseWithCovariance pose           # position, orientation
geometry_msgs/TwistWithCovariance velocity      # linear, angular
geometry_msgs/Vector3 acceleration              # linear acceleration

uint16 fusion_status                            # field for the fusion status
uint16 imu_bias_status                          # field for the IMU bias status
uint16 gnss_status                              # field for the gnss status
uint16 wheelspeed_status                        # field for the wheelspeed status
string version                                  # Fixposition software version