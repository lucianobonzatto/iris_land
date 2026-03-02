clear all;
close all;
clc;

node = ros2node("/matlab_pose_subscriber");

turtle_pose_sub = ros2subscriber(node, "/robot_pose", "geometry_msgs/PoseStamped");
pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
cmd_vel_msg = ros2message(pub);

fprintf('Aguardando mensagens de pose...\n')

% Definir pose de destino diretamente (MODIFIQUE AQUI para onde quer ir)
goto_pose_x = 4.5;  % Posição X desejada
goto_pose_y = -1.0;  % Posição Y desejada

% Loop de recebimento
counter = 0;
try
    while true
        % Receber mensagem (timeout de 1 segundo)
        [turtle_pose_msg, turtle_pose_status, statustext] = receive(turtle_pose_sub, 1);
        
        if turtle_pose_status
            % Processar mensagem recebida
            fprintf('--- Mensagem %d ---\n', counter);

            cmd_vel_msg.linear.x = round(goto_pose_x - turtle_pose_msg.pose.position.x, 2);
            cmd_vel_msg.linear.y = round(goto_pose_y - turtle_pose_msg.pose.position.y, 2);


            fprintf('  Turtle x = %.4f\n', turtle_pose_msg.pose.position.x);
            fprintf('  Turtle y = %.4f\n', turtle_pose_msg.pose.position.y);
            fprintf('  Goal x = %.4f\n', goto_pose_x);
            fprintf('  Goal y = %.4f\n', goto_pose_y);
            fprintf('  Vel x = %.4f\n', cmd_vel_msg.linear.x);
            fprintf('  Vel y = %.4f\n', cmd_vel_msg.linear.y);

            send(pub, cmd_vel_msg);
            
            counter = counter + 1;
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
