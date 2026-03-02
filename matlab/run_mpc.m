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

load('drone_mpc.mat', 'mpcobj', 'identified_model', 'Ts', 'fit_x', 'fit_y');

fprintf('MPC carregado:\n');
fprintf('  - Ts = %.4f s\n', Ts);
fprintf('  - Qualidade do modelo: X=%.1f%%, Y=%.1f%%\n', fit_x, fit_y);
if fit_x < 50 || fit_y < 50
    warning('Modelo com qualidade baixa (FIT < 50%%). O desempenho pode ser ruim!');
end
fprintf('\n');

goto_pose_x = [3, -3, -3,  3];  % Posição X desejada
goto_pose_y = [3,  3, -3, -3];  % Posição Y desejada
xmpc = mpcstate(mpcobj);        % Estado interno do MPC

% Loop de recebimento
counter = 0;
goal_index = 1;
try
    while true
        % Receber mensagem (timeout de 1 segundo)
        [turtle_pose_msg, turtle_pose_status, statustext] = receive(turtle_pose_sub, 1);
        
        if turtle_pose_status
            % Posição atual e referência (apenas X e Y para robô 2D)
            current_pos = [turtle_pose_msg.pose.position.x; turtle_pose_msg.pose.position.y];
            ref = [goto_pose_x(goal_index); goto_pose_y(goal_index)];
            
            cmd_vel = mpcmove(mpcobj, xmpc, current_pos, ref);
            
            % Aplicar comandos
            cmd_vel_msg.linear.x = cmd_vel(1);
            cmd_vel_msg.linear.y = cmd_vel(2);

            fprintf('--- Mensagem %d ---\n', counter);
            fprintf('  Pos atual: (%.2f, %.2f) | Ref: (%.1f, %.1f)\n', ...
                current_pos(1), current_pos(2), ref(1), ref(2));
            fprintf('  Cmd MPC: Vx=%.4f, Vy=%.4f\n', cmd_vel_msg.linear.x, cmd_vel_msg.linear.y);

            send(pub, cmd_vel_msg);
            
            counter = counter + 1;

            % Verificar se atingiu o waypoint
            if abs(current_pos(1) - ref(1)) < 0.1 && abs(current_pos(2) - ref(2)) < 0.1
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
