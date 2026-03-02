%% simulate_mpc.m
% Carrega e aplica o controlador MPC previamente criado
% Requisito: execute 'create_mpc_controller.m' primeiro

clear all;
close all;

%% Carregar controlador MPC
fprintf('Carregando controlador MPC...\n');
if ~isfile('drone_mpc.mat')
    error('Arquivo drone_mpc.mat não encontrado. Execute create_mpc_controller.m primeiro.');
end

load('drone_mpc.mat', 'mpcobj', 'tau', 'K', 'Ts');
fprintf('Controlador MPC carregado com sucesso!\n\n');

%% Parâmetros de simulação
sim_length = 150;
fprintf('Tempo de simulação: %.1f s (%d amostras)\n', sim_length*Ts, sim_length);

%% Definir trajetória de referência
ref = zeros(sim_length, 4);

% Trajetória em X
ref(1:50, 1) = 0;
ref(51:100, 1) = 5;    % Degrau para 5m
ref(101:end, 1) = 5;

% Trajetória em Y
ref(1:70, 2) = 0;
ref(71:end, 2) = 3;    % Degrau para 3m

% Altitude Z (constante)
ref(:, 3) = 2;

% Yaw (constante)
ref(:, 4) = 0;

%% Inicializar simulação
y = zeros(sim_length, 4);      % Posições medidas
u = zeros(sim_length, 4);      % Velocidades comandadas
x_state_sim = zeros(4, 1);     % Estado interno da planta
xmpc = mpcstate(mpcobj);       % Estado interno do MPC

%% Loop de simulação

fprintf('Simulando...\n');
for k = 2:sim_length
    % MPC calcula velocidades baseado na posição atual e referência
    u(k,:) = mpcmove(mpcobj, xmpc, y(k-1,:), ref(k,:));
    
    % Simular dinâmica da planta: velocidade -> posição
    for i = 1:4
        x_state_sim(i) = x_state_sim(i) + (Ts/tau(i)) * (-x_state_sim(i) + K(i,i)*u(k,i));
    end
    
    y(k,:) = x_state_sim';
end

%% Plotar resultados
figure('Position', [100 100 1200 800]);
sgtitle('Resultados da Simulação MPC', 'FontSize', 14, 'FontWeight', 'bold');

% Velocidades comandadas
vel_names = {'V_x', 'V_y', 'V_z', 'V_{yaw}'};
for i = 1:4
    subplot(4,2,i);
    plot(0:Ts:(sim_length-1)*Ts, u(:,i), 'r-', 'LineWidth', 1.5);
    ylabel([vel_names{i} ' (m/s)']);
    xlabel('Tempo (s)');
    grid on;
    title(['Velocidade Comandada: ' vel_names{i}]);
    ylim([min(u(:,i))-0.5, max(u(:,i))+0.5]);
end

% Posições
pos_names = {'X', 'Y', 'Z', 'Yaw'};
for i = 1:4
    subplot(4,2,4+i);
    plot(0:Ts:(sim_length-1)*Ts, ref(:,i), 'k--', 'LineWidth', 2, 'DisplayName', 'Referência');
    hold on;
    plot(0:Ts:(sim_length-1)*Ts, y(:,i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real');
    legend('Location', 'best');
    ylabel([pos_names{i} ' (m)']);
    xlabel('Tempo (s)');
    grid on;
    title(['Posição: ' pos_names{i}]);
end

%% Trajetória 3D
figure('Position', [150 150 800 600]);
plot3(ref(:,1), ref(:,2), ref(:,3), 'k--', 'LineWidth', 2.5, 'DisplayName', 'Referência');
hold on;
plot3(y(:,1), y(:,2), y(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Trajetória Real');

% Marcar pontos inicial e final
plot3(y(1,1), y(1,2), y(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Início');
plot3(y(end,1), y(end,2), y(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Fim');

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Location', 'best');
title('Trajetória 3D do Drone', 'FontSize', 12, 'FontWeight', 'bold');
view(45, 30);
axis equal;
