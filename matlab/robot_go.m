clear all;
close all;
clc;

node = ros2node("/matlab_pose_subscriber");

turtle_pose_sub = ros2subscriber(node, "/robot_pose", "geometry_msgs/PoseStamped");
pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
cmd_vel_msg = ros2message(pub);

fprintf('Aguardando mensagens de pose...\n')

% Definir pose de destino diretamente (MODIFIQUE AQUI para onde quer ir)
goto_pose_x = [1, -1, -1,  1];  % Posição X desejada
goto_pose_y = [1,  1, -1, -1];  % Posição Y desejada

% Arrays para armazenar dados
Input_log = [];     % Comandos de velocidade
Output_log = [];    % Posições
Time_log = [];      % Timestamps
start_time = tic;   % Tempo inicial

% Loop de recebimento
counter = 0;
goal_index = 1;
try
    while true
        % Receber mensagem (timeout de 1 segundo)
        [turtle_pose_msg, turtle_pose_status, statustext] = receive(turtle_pose_sub, 1);
        
        if turtle_pose_status
            % Processar mensagem recebida
            fprintf('--- Mensagem %d ---\n', counter);

            if abs(goto_pose_x(goal_index) - turtle_pose_msg.pose.position.x) < 0.01
                cmd_vel_msg.linear.x = 0;
            elseif goto_pose_x(goal_index) > turtle_pose_msg.pose.position.x
                cmd_vel_msg.linear.x = 1;  % Move para frente
            else
                cmd_vel_msg.linear.x = -1; % Move para trás
                
            end


            if abs(goto_pose_y(goal_index) - turtle_pose_msg.pose.position.y) < 0.01
                cmd_vel_msg.linear.y = 0;
            elseif goto_pose_y(goal_index) > turtle_pose_msg.pose.position.y
                cmd_vel_msg.linear.y = 1;  % Move para direita
            else
                cmd_vel_msg.linear.y = -1; % Move para esquerda
            end

            
            send(pub, cmd_vel_msg);
            fprintf('  Vel x = %.4f\n', cmd_vel_msg.linear.x);
            fprintf('  Vel y = %.4f\n', cmd_vel_msg.linear.y);

            % Armazenar dados para salvar depois
            Input_log = [Input_log; cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, 0, 0];
            Output_log = [Output_log; turtle_pose_msg.pose.position.x, turtle_pose_msg.pose.position.y, 0, 0];
            Time_log = [Time_log; toc(start_time)];
            
            
            counter = counter + 1;
            if abs(cmd_vel_msg.linear.x) < 0.01 && abs(cmd_vel_msg.linear.y) < 0.01
                goal_index = goal_index + 1;
                if goal_index > length(goto_pose_x)
                    goal_index = 1;  % Reiniciar sequência de metas
                    fprintf('\n=== Sequência completa! Salvando dados... ===\n');
                    
                    Input = Input_log;
                    Output = Output_log;
                    Time = Time_log;
                    
                    filename = sprintf('mpc_run_data_%s.mat', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
                    save(filename, 'Input', 'Output', 'Time');
                    
                    fprintf('Dados salvos em: %s\n', filename);
                    fprintf('  - %d amostras coletadas\n', size(Input, 1));
                    fprintf('  - Duração: %.1f s\n', Time(end));
                    fprintf('Reiniciando sequência...\n\n');
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
