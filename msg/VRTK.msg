Header header
string kin_frame                                # frame of the kinematic values [linear / angular velocity, acceleration]
geometry_msgs/PoseWithCovariance pose           # position, orientation
geometry_msgs/TwistWithCovariance velocity      # linear, angular
geometry_msgs/Vector3 acceleration              # linear acceleration
uint16 fusion_status                            # field for the fusion status
uint16 gnss_status                              # field for the gnss status
