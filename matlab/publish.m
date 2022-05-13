rosinit;
%% Intento de publicar sin usar el servicio, no logramos hacerlo funcionar
posPub = rospublisher("joint_trajectory","trajectory_msgs/JointTrajectory");
posMsg = rosmessage(posPub);


while true
    posMsg.Header.Stamp = rostime('now');
    posMsg.JointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"];
    joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    joint_send.Positions = [0, 0, 0, 0, 0];
    posMsg.Points = joint_send;
    posMsg.Points.TimeFromStart = rosduration(0.5);
    send(posPub,posMsg);
    disp('publicado 1')
    pause(1)
    posMsg.Header.Stamp = rostime('now');
    posMsg.JointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"];
    joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    joint_send.Positions = [0.25, 0, 0, 0, 1.3];
    posMsg.Points = joint_send;
    posMsg.Points.TimeFromStart = rosduration(0.5);
    send(posPub,posMsg);
    disp('publicado 2')
    pause(1)
end
rosshutdown;


