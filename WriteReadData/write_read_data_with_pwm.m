
%%% Inicialización
clc;
clearvars -except out ts_position ts_position_1 ts_position_2 ts_position_3 ts_position_4 ts_position_5  

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

%%% Carga de librerías
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end


%%% Direcciones de la tabla de control
ADDR_PRO_TORQUE_ENABLE       = 64;         
ADDR_PRO_GOAL_PWM            = 100;
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_REALTIME_TICK       = 120;
ADDR_PRO_PRESENT_CURRENT     = 126;
ADDR_PRO_PRESENT_VELOCITY    = 128;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_PRESENT_VOLTAGE     = 144;
ADDR_PRO_PRESENT_TEMP        = 146;

%%% Versión de protocolo utilizada
PROTOCOL_VERSION            = 2.0;          % Es el usado por el modelo XM430-210W-R.

%%% Configuración
DXL_ID                      = 1;            % ID del motor (1 ó 2).
BAUDRATE                    = 56900;
DEVICENAME                  = 'COM3';       

TORQUE_ENABLE               = 1;            % Variable para habilitar el torque del motor.
TORQUE_DISABLE              = 0;            % Variable para deshabilitar el torque del motor.
DXL_MINIMUM_POSITION_VALUE  = 0;            % Posición mínima del motor.
DXL_MAXIMUM_POSITION_VALUE  = 4095;         % Posición máxima del motor.
DXL_MOVING_STATUS_THRESHOLD = 10;           % Umbral de movimiento.

DXL_MINIMUM_PWM_VALUE  = -885;              % Valor mínimo de la señal PWM.
DXL_MAXIMUM_PWM_VALUE  = 885;               % Valor máximo de la señal PWM.

ESC_CHARACTER               = 'e';          % Comando para salir del bucle de lectura de datos.

COMM_SUCCESS                = 0;            % Resultado si la comunicación tiene éxito
COMM_TX_FAIL                = -1001;        % Indicador de fallo de comunicación Tx

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

%if (exist('index')==0)
%    fprintf('Index has been initialized\n');
%    index = 1;
%end

dxl_comm_result = COMM_TX_FAIL;           % Communication result
%dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position
dxl_present_velocity = 0;                   % Present position
dxl_present_current=0;
dxl_present_voltage=0;
dxl_present_temp=0;

%%% Selección de la entrada PWM
dxl_goal_pwm = 200;




%%% Apertura del puerto

if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


%%% Baud rate = 59600 bps
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


%%% Habilitar el torque del motor
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%%% Inicialización de variables para el bucle.
i=1;
t_initial=now;
dxl_goal_pwm = 0;
initial_pos = 2060; %Posición inicial, hay que mirarla previamente en el programa R-Manager de Robotics.

%%% Bucle de control del motor.
while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
    
    
    %%% Control de tiempo para que el motor empiece siempre a moverse en el mismo instante.
    while 1
       initial_realtime_tick = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);
       fprintf('RealTIME:%03d\n', initial_realtime_tick);
       if (initial_realtime_tick<50)
           break;
       end
    end
    dxl_realtime_tick=initial_realtime_tick;
    
    switch opcion
        case 0
            while (dxl_realtime_tick<=10000) % Tiempo en milisegundos
                dxl_realtime_tick=read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);
                dxl_goal_pwm = -40;
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_PWM, typecast(int32(dxl_goal_pwm), 'uint32'));
                dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
                end

                dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
                if (i==1)
                    initial_pos=dxl_present_position;
                end

                position(i,1)=(initial_pos-dxl_present_position)*0.00153398078; %rads
                time(i,1)=datetime('now');

                fprintf('[ID:%03d] i:%03d GoalPWM:%03d   PresPos:%03d\n', DXL_ID, i, dxl_goal_pwm,  position(i,1));
                i=i+1;
                
            end
         
        case 1
             while (dxl_realtime_tick<=10000)  % Tiempo en milisegundos
                dxl_realtime_tick=read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);
                dxl_goal_pwm = -0.0032*dxl_realtime_tick-8;
                
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_PWM, typecast(int32(dxl_goal_pwm), 'uint32'));
                dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
                end

                dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
                if (i==1)
                    initial_pos=dxl_present_position;
                end

                position(i,1)=(initial_pos-dxl_present_position)*0.00153398078; %rads
                time(i,1)=datetime('now');

                fprintf('[ID:%03d] i:%03d GoalPWM:%03d   PresPos:%03d\n', DXL_ID, i, dxl_goal_pwm,  position(i,1));
                i=i+1;
                
             end
        case 2
            while (dxl_realtime_tick<=32000)  % Tiempo en milisegundos
                dxl_realtime_tick=read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);

                if((abs(dxl_realtime_tick-initial_realtime_tick)<=1000)||(abs(dxl_realtime_tick-initial_realtime_tick)>4000 && abs(dxl_realtime_tick-initial_realtime_tick)<=6000)||(abs(dxl_realtime_tick-initial_realtime_tick)>8000 && abs(dxl_realtime_tick-initial_realtime_tick)<=10000)||(abs(dxl_realtime_tick-initial_realtime_tick)>12000 && abs(dxl_realtime_tick-initial_realtime_tick)<=14000)||(abs(dxl_realtime_tick-initial_realtime_tick)>16000 && abs(dxl_realtime_tick-initial_realtime_tick)<=18000)||(abs(dxl_realtime_tick-initial_realtime_tick)>20000 && abs(dxl_realtime_tick-initial_realtime_tick)<=22000)||(abs(dxl_realtime_tick-initial_realtime_tick)>24000 && abs(dxl_realtime_tick-initial_realtime_tick)<=26000)||(abs(dxl_realtime_tick-initial_realtime_tick)>28000 && abs(dxl_realtime_tick-initial_realtime_tick)<=30000))
                    dxl_goal_pwm = 0;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>1000 && abs(dxl_realtime_tick-initial_realtime_tick)<=4000)
                    dxl_goal_pwm = -8;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>6000 && abs(dxl_realtime_tick-initial_realtime_tick)<=8000)
                    dxl_goal_pwm = -12;    
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>10000 && abs(dxl_realtime_tick-initial_realtime_tick)<=12000)
                    dxl_goal_pwm = -16;    
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>14000 && abs(dxl_realtime_tick-initial_realtime_tick)<=16000)
                    dxl_goal_pwm = -20;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>18000 && abs(dxl_realtime_tick-initial_realtime_tick)<=20000)
                    dxl_goal_pwm = -24;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>22000 && abs(dxl_realtime_tick-initial_realtime_tick)<=24000)
                    dxl_goal_pwm = -28;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>26000 && abs(dxl_realtime_tick-initial_realtime_tick)<=28000)
                    dxl_goal_pwm = -32;
                elseif (abs(dxl_realtime_tick-initial_realtime_tick)>30000 && abs(dxl_realtime_tick-initial_realtime_tick)<=32000)
                    dxl_goal_pwm = -36;
                end
                
                % Escritura de la entrada PWM
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_PWM, typecast(int32(dxl_goal_pwm), 'uint32'));
                dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
                end

                dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
             
                dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
                dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
                end

                position(i,1)=(initial_pos-dxl_present_position)*0.00153398078; %rads
                time(i,1)=datetime('now');

                fprintf('[ID:%03d] i:%03d GoalPWM:%03d   PresPos:%03d\n', DXL_ID, i, dxl_goal_pwm,  position(i,1));
                i=i+1;
            end
    end
end


%%% Detener el motor
dxl_goal_pwm = 0;

%%% Deshabilitar el torque del motor
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

%%% Cierre del puerto COM
closePort(port_num);

%%% Cierre de las librerías utilizadas
unloadlibrary(lib_name);

close all;


%%%% Envío de datos a Simulink
%%% Obtención de un vector de posiciones en el tiempo.
time.Format = 'HH:mm:ss.SSS';

d2s    = 24*3600;       % Convertir de días a segundos.
tspan  = zeros(length(time(:,1)),1);
for ii = 1:length(time)
    tspan(ii,1) = d2s*(datenum(time(ii,:))-datenum(time(1,:)));
end
end_time=tspan(length(time))+0.0320;
tspan=[tspan; end_time];


%%% Guardar los datos tomados en vectores de tiempo.
position=[initial_pos; position];
ts_position = timeseries(position,tspan);


%% Para leer todos los datos posibles del motor para una determinada entrada PWM constante 

%%% Inicialización
clc;
clearvars -except out ts_position ts_position_1 ts_position_2 ts_position_3 ts_position_4 ts_position_5  

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

%%% Carga de librerías
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end


%%% Direcciones de la tabla de control
ADDR_PRO_TORQUE_ENABLE       = 64;         
ADDR_PRO_GOAL_PWM            = 100;
ADDR_PRO_GOAL_POSITION       = 116;
ADDR_PRO_REALTIME_TICK       = 120;
ADDR_PRO_PRESENT_CURRENT     = 126;
ADDR_PRO_PRESENT_VELOCITY    = 128;
ADDR_PRO_PRESENT_POSITION    = 132;
ADDR_PRO_PRESENT_VOLTAGE     = 144;
ADDR_PRO_PRESENT_TEMP        = 146;

%%% Versión de protocolo utilizada
PROTOCOL_VERSION            = 2.0;          % Es el usado por el modelo XM430-210W-R.

%%% Configuración
DXL_ID                      = 1;            % ID del motor (1 ó 2).
BAUDRATE                    = 56900;
DEVICENAME                  = 'COM3';       

TORQUE_ENABLE               = 1;            % Variable para habilitar el torque del motor.
TORQUE_DISABLE              = 0;            % Variable para deshabilitar el torque del motor.
DXL_MINIMUM_POSITION_VALUE  = 0;            % Posición mínima del motor.
DXL_MAXIMUM_POSITION_VALUE  = 4095;         % Posición máxima del motor.
DXL_MOVING_STATUS_THRESHOLD = 10;           % Umbral de movimiento.

DXL_MINIMUM_PWM_VALUE  = -885;              % Valor mínimo de la señal PWM.
DXL_MAXIMUM_PWM_VALUE  = 885;               % Valor máximo de la señal PWM.

ESC_CHARACTER               = 'e';          % Comando para salir del bucle de lectura de datos.

COMM_SUCCESS                = 0;            % Resultado si la comunicación tiene éxito
COMM_TX_FAIL                = -1001;        % Indicador de fallo de comunicación Tx

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();


dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position
dxl_present_velocity = 0;                   % Present position
dxl_present_current=0;
dxl_present_voltage=0;
dxl_present_temp=0;


%%% Apertura del puerto

if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


%%% Baud rate = 59600 bps
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


%%% Habilitar el torque del motor
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%%% Inicialización de variables para el bucle.
i=1;
t_initial=now;
dxl_goal_pwm = 0;
initial_pos = 2060; 

%%% Bucle de control del motor.
while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
    
    
    %%% Control de tiempo para que el motor empiece siempre a moverse en el mismo instante.
    while 1
       initial_realtime_tick = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);
       fprintf('RealTIME:%03d\n', initial_realtime_tick);
       if (initial_realtime_tick<50)
           break;
       end
    end
    dxl_realtime_tick=initial_realtime_tick;
            
            
    while (dxl_realtime_tick<=10000)  % Tiempo en milisegundos.
        dxl_realtime_tick=read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_REALTIME_TICK);

        dxl_goal_pwm = -40;

        %%% Escritura de la entrada PWM
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_PWM, typecast(int32(dxl_goal_pwm), 'uint32'));
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        if (i==1)
            initial_pos=dxl_present_position;
        end
        dxl_present_velocity = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_VELOCITY);
        dxl_present_current = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_CURRENT);
        dxl_present_voltage = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_VOLTAGE);
        dxl_present_temp = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_TEMP);
        
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        position(i,1)=(initial_pos-dxl_present_position)*0.00153398078; %rads
        time(i,1)=datetime('now');
        velocity(i,1)=dxl_present_velocity*((0.229*pi)/30); % rad/s
        current(i,1)=dxl_present_current*2.69; %
        voltage(i,1)=dxl_present_voltage*0.1;
        temp(i,1)=dxl_present_temp;
        
        fprintf('[ID:%03d] i:%03d GoalPWM:%03d  PresVel:%03d  PresPos:%03d  PresCurr:%03d  PresVolt:%03d  PresTemp:%03d\n', DXL_ID, i, dxl_goal_pwm, dxl_present_velocity,  position(i,1), dxl_present_current, dxl_present_voltage,dxl_present_temp);

        %%% Trataminto de los datos tomados.
        
        
        
        

        i=i+1;

        %%% Realizar 20 iteraciones antes de salir del bucle
    %    if (i>length(out.signal_step.Data))
     %       break;
     %   end
    end
    
   
    
end
 %%% Detener el motor
    dxl_goal_pwm = 0;

%%% Deshabilitar el torque del motor
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

%%% Cierre del puerto COM
closePort(port_num);

%%% Cierre de las librerías utilizadas
unloadlibrary(lib_name);

close all;

%%% Envío de datos a Simulink

%%% Obtención de un vector de posiciones en el tiempo.
time.Format = 'HH:mm:ss.SSS';

d2s    = 24*3600;       % Convertir de días a segundos.
tspan  = zeros(length(time(:,1)),1);
for ii = 1:length(time)
    tspan(ii,1) = d2s*(datenum(time(ii,:))-datenum(time(1,:)));
end
end_time=tspan(length(time))+0.0320;
tspan=[tspan; end_time];


%%% Guardar los datos tomados en vectores de tiempo.
%initial_pos=0;
position=[initial_pos; position];
%velocity=[0; velocity];
%current=[0; current];
%voltage=[0; voltage];
%temp=[temp; temp(i-1)];

ts_position_4 = timeseries(position,tspan);
%ts_velocity = timeseries(velocity,tspan);
%ts_current = timeseries(current,tspan);
%ts_voltage = timeseries(voltage,tspan);
%ts_temp = timeseries(temp,tspan);

