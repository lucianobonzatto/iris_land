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

load('drone_mpc.mat', 'mpcobj', 'tau', 'K', 'Ts');

goto_pose_x = [5, -5, -5,  5];  % Posição X desejada
goto_pose_y = [5,  5, -5, -5];  % Posição Y desejada
xmpc = mpcstate(mpcobj);        % Estado interno do MPC

% Loop de recebimento
counter = 0;
goal_index = 1;
try
    while true
        % Receber mensagem (timeout de 1 segundo)
        [turtle_pose_msg, turtle_pose_status, statustext] = receive(turtle_pose_sub, 1);
        
        if turtle_pose_status
            % cmd_vel_msg.linear.x = round(goto_pose_x(goal_index) - turtle_pose_msg.pose.position.x, 2);
            % cmd_vel_msg.linear.y = round(goto_pose_y(goal_index) - turtle_pose_msg.pose.position.y, 2);

            % MPC calcula velocidades baseado na posição atual e referência
            current_pos = [turtle_pose_msg.pose.position.x, turtle_pose_msg.pose.position.y, 0, 0];
            ref = [goto_pose_x(goal_index), goto_pose_y(goal_index), 0, 0];
            cmd_vel_msg.linear.x = mpcmove(mpcobj, xmpc, current_pos, ref);
            cmd_vel_msg.linear.y = mpcmove(mpcobj, xmpc, current_pos, ref);

            cmd_vel = mpcmove(mpcobj, xmpc, current_pos, ref);

            cmd_vel_msg.linear.x = cmd_vel(1);
            cmd_vel_msg.linear.y = cmd_vel(2);

            % fprintf('  Turtle x = %.4f\n', turtle_pose_msg.pose.position.x);
            % fprintf('  Turtle y = %.4f\n', turtle_pose_msg.pose.position.y);
            % fprintf('  Goal x = %.4f\n', goto_pose_x(goal_index));
            % fprintf('  Goal y = %.4f\n', goto_pose_y(goal_index));
            fprintf('--- Mensagem %d ---\n', counter);
            fprintf('  Vel x = %.4f\n', cmd_vel_msg.linear.x);
            fprintf('  Vel y = %.4f\n', cmd_vel_msg.linear.y);

            send(pub, cmd_vel_msg);
            
            counter = counter + 1;
            if abs(cmd_vel_msg.linear.x) < 0.01 && abs(cmd_vel_msg.linear.y) < 0.01
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
    fprintf('\nEncerrando subscriber...\n');
end

% Limpar recursos
clear sub node;
fprintf('Subscriber finalizado.\n');
