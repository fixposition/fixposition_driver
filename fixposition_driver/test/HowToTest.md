# How to test using example data for Fixposition ROS / ROS2 Driver 

## we use `str2str` from `RTKLIB` to replay datastream on TCP port
  - Install RTKLIB: `sudo apt install rtklib`
  - The files are in `test/data`
    - `vrtk_output_1.txt` the tcp data stream
    - `vrtk_output_1.txt.tag` the timestamp file for the corresponding data stream
  - Replay the TCP stream:
    - `str2str -in file://vrtk2_output_1.txt::T -out tcpsvr://localhost:21000`
    - Adapt the IP and Port if needed
    - You can check with netcat `nc localhost 21000` to see if the data is properly replayed
    - For more details, see `str2str -h` 

## How to test
- Compile the ROS driver
- configure the ROS driver's `tcp.yaml` to the corresponding IP and port from above.
- Start the ROS driver with `roslaunch fixposition_driver tcp.launch`
- For Visualization, start RVIZ with the provided RVIZ configuration in `fixposition_driver/rviz/fixposition_driver.rviz`
- You shall see something similar to this:
  ![image](TestData_example.png)