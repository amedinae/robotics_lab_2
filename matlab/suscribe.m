sub = rossubscriber('/dynamixel_workbench/joint_states'); % Nos subscribimos al servicio que deseamos
while(true)
    sub.LatestMessage.Position % Leemos la posición del servicio
end