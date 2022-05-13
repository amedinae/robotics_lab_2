lenghts = [40.75 106 106 85]; % Longitud de eslabones

% Creación de un arreglo de eslabones
L(1) = Link('revolute','d', lenghts(1), 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute','d', 0, 'a', lenghts(2), 'alpha', 0,'offset',pi/2);
L(3) = Link('revolute','d', 0, 'a', lenghts(3), 'alpha', 0);
L(4) = Link('revolute','d', 0, 'a', lenghts(4), 'alpha', 0);

robot = SerialLink(L); % Secrea un robot serial apartir del arreglo de eslabones
robot.tool=[0 0 1 0; 1 0 0 0;0 1 0 0;0 0 0 1]; % MTH que define la posición de la herramienta con respecto al ultimo eslabon

syms q1 q2 q3 q4 % Variables simbolicas para modelar el robot de forma parametrica

% Multiples configuraciones para el robot
q_1 = [0 0 0 0 0];
q_2 = [-20 -20 -20 -20 0];
q_3 = [30,-30, 30, -30, 0];
q_4 = [-90, 15, -55, 17, 0];
q_5 = [-90, 45, -55, 45, 10];

% Arreglo de configuraciones
targets = [q_1;q_2;q_3;q_4;q_5];
% Arreglo de variables simbolicas
q_s = [q1 q2 q3 q4];

% Calculo de matrices de transformación
MTH1 = prettify(L(1).A(q1));
MTH2 = prettify(L(2).A(q2));
MTH3 = prettify(L(3).A(q3));
MTH4 = prettify(L(4).A(q4));
MTH = simplify(MTH1*MTH2*MTH3*MTH4*robot.tool)
MTHd = double(subs(MTH,[q1 q2 q3 q4], q_0));
MTH2 = robot.fkine(q_s);
MTH2d = double(subs(MTH2,[q1 q2 q3 q4], q_0));
%%
rosinit %Inicializamos el nodo master de ROS
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');  %Obtenemos el servicio
motorCommandMsg = rosmessage(motorSvcClient); % Creamos nuestro mensaje
%%
motorCommandMsg.AddrName = "Goal_Position" ; %Definimos el parametro que vamos a modificar 
for i=1:length(targets) % Recorremos el arreglo de configuraciones 
    robot.plot(pi/180*targets(i,1:4),'notiles','noname','noa') % Ploteamos el robot para la configuración dada en Matlab
    disp(i);
    for j=1:length(targets(i)) % Recorremos los angulos dentro de cada configuración
        motorCommandMsg.Id = j; % Seteamos el ID de motor que queremos mover
        motorCommandMsg.Value = round(mapfun(targets(i,j),-150,150,0,1023)); % Seteamos la posición a la que se moveera el motor
        call(motorSvcClient,motorCommandMsg); % Llamamos al servicio enviando nuestro mensaje
        pause(1); % Delay de 1 segundo 
    end
    pause(2); % Delay de 2 segundos
end

% Función para mapear los valores de [-150,150] grados a [0,1023]
function output = mapfun(value,fromLow,fromHigh,toLow,toHigh)
    narginchk(5,5)
    nargoutchk(0,1)
    output = (value-fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow)+ toLow;
end

% Función para imprimir matrices de transformación homogenea
function MTH = prettify(x)
    MTH = x;
    for i = 1:numel(x);
        value = x(i);
        if isnumeric(value) & abs(value)<1e-10
            MTH(i) = 0;
        elseif abs(coeffs(value))<1e-10
            MTH(i) = 0;
        end
    end
end