clear all;
close all;
clc;

node = ros2node("/matlab_pose_subscriber");

turtle_pose_sub = ros2subscriber(node, "/robot_pose", "geometry_msgs/PoseStamped");
pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
cmd_vel_msg = ros2message(pub);

fprintf('Carregando controlador MPC...\n');
if ~isfile('drone_mpc.mat')
    error('Arquivo drone_mpc.mat não encontrado. Execute create_mpc_controller.m primeiro.');
end

load('drone_mpc.mat', 'mpcobj', 'identified_model', 'Ts', 'fit_x', 'fit_y', 'fit_z', 'fit_yaw');

fprintf('MPC carregado:\n');
fprintf('  - Ts = %.4f s\n', Ts);
fprintf('  - Qualidade: X=%.1f%%, Y=%.1f%%, Z=%.1f%%, Yaw=%.1f%%\n', fit_x, fit_y, fit_z, fit_yaw);
if fit_x < 50 || fit_y < 50 || fit_z < 50 || fit_yaw < 50
    warning('Modelo com qualidade baixa (FIT < 50%%). O desempenho pode ser ruim!');
end
fprintf('\n');

% Trajetória circular (raio = 3m, 16 pontos)
n_points = 16;
radius = 3;
theta = linspace(0, 2*pi, n_points+1);
theta = theta(1:end-1);  % Remover último ponto (igual ao primeiro)
goto_pose_x = radius * cos(theta);      % Posição X desejada
goto_pose_y = radius * sin(theta);      % Posição Y desejada
goto_pose_z = zeros(1, n_points);       % Posição Z desejada (altitude)
goto_pose_yaw = zeros(1, n_points);     % Yaw desejado (orientação)

% Inicializar estado do MPC
xmpc = mpcstate(mpcobj);

% Loop de recebimento
counter = 0;
goal_index = 1;
try
    while true
        % Receber mensagem (timeout de 1 segundo)
        [turtle_pose_msg, turtle_pose_status, statustext] = receive(turtle_pose_sub, 1);
        
        if turtle_pose_status
            % Posição atual e referência (4 dimensões: X, Y, Z, Yaw)
            current_pos = [turtle_pose_msg.pose.position.x; 
                          turtle_pose_msg.pose.position.y;
                          turtle_pose_msg.pose.position.z;
                          0];  % Yaw (será extraído do quaternion depois se necessário)
            
            ref = [goto_pose_x(goal_index); 
                   goto_pose_y(goal_index);
                   goto_pose_z(goal_index);
                   goto_pose_yaw(goal_index)];
            
            % MPC calcula velocidades (4 saídas: Vx, Vy, Vz, Vyaw)
            cmd_vel = mpcmove(mpcobj, xmpc, current_pos, ref);
            
            % Aplicar comandos
            cmd_vel_msg.linear.x = cmd_vel(1);
            cmd_vel_msg.linear.y = cmd_vel(2);
            cmd_vel_msg.linear.z = cmd_vel(3);
            cmd_vel_msg.angular.z = cmd_vel(4);

            fprintf('--- Mensagem %d ---\n', counter);
            fprintf('  Pos atual: (%.2f, %.2f, %.2f, %.2f) | Ref: (%.1f, %.1f, %.1f, %.1f)\n', ...
                current_pos(1), current_pos(2), current_pos(3), current_pos(4), ...
                ref(1), ref(2), ref(3), ref(4));
            fprintf('  Cmd MPC: Vx=%.3f, Vy=%.3f, Vz=%.3f, Vyaw=%.3f\n', ...
                cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.linear.z, cmd_vel_msg.angular.z);

            send(pub, cmd_vel_msg);
            
            counter = counter + 1;

            % Verificar se atingiu o waypoint (todas as 4 dimensões)
            tolerance_xy = 0.1;    % Tolerância para X e Y (metros)
            tolerance_z = 0.1;     % Tolerância para Z (metros)
            tolerance_yaw = 0.1;   % Tolerância para Yaw (radianos)
            
            if abs(current_pos(1) - ref(1)) < tolerance_xy && ...
               abs(current_pos(2) - ref(2)) < tolerance_xy && ...
               abs(current_pos(3) - ref(3)) < tolerance_z && ...
               abs(current_pos(4) - ref(4)) < tolerance_yaw
                fprintf('✓ Waypoint %d atingido!\n\n', goal_index);
                goal_index = goal_index + 1;
                if goal_index > length(goto_pose_x)
                    goal_index = 1;  % Reiniciar sequência de metas
                    
                end
            end

        else
            fprintf('Timeout: %s\n', statustext);
        end
    end
catch ME
    fprintf('\n!!! ERRO OCORREU !!!\n');
    fprintf('Mensagem: %s\n', ME.message);
    fprintf('Arquivo: %s\n', ME.stack(1).file);
    fprintf('Linha: %d\n', ME.stack(1).line);
    fprintf('\nEncerrando subscriber...\n');
end

% Limpar recursos
clear sub node;
fprintf('Subscriber finalizado.\n');
