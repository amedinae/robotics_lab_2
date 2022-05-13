rosinit; %Inicializamos el nodo master de ROS

motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Obtenemos el servicio
motorCommandMsg = rosmessage(motorSvcClient); % Creamos nuestro mensaje
motorCommandMsg.AddrName = "Goal_Position" ; %Definimos el parametro que vamos a modificar  

q1 = zeros(5,1) % Posición de home del robot
for i=1:length(q1)
    motorCommandMsg.Id = i; % Seteamos el ID de motor que queremos mover
    motorCommandMsg.Value = round(mapfun(q1(i),-150,150,0,1023)); % Seteamos la posición a la que se moveera el motor
    call(motorSvcClient,motorCommandMsg); % Llamamos al servicio enviando nuestro mensaje
    pause(1); % Delay de 1 segundo
end

q2 = [-90, 35, -55, -87, 45]; % Posición de objetivo del robot
for i=1:length(q2)
    motorCommandMsg.Id = i; % Seteamos el ID de motor que queremos mover
    motorCommandMsg.Value = round(mapfun(q2(i),-150,150,0,1023)); % Seteamos la posición a la que se moveera el motor
    call(motorSvcClient,motorCommandMsg); % Llamamos al servicio enviando nuestro mensaje
    pause(1); % Delay de 1 segundo   
end

% Función para mapear los valores de [-150,150] grados a [0,1023]
function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)  
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value-fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow)+ toLow;
end