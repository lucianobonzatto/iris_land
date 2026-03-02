clear all;
length = 100;
Ts = 0.1;

%% Sistema SIMO: 1 entrada -> 4 saídas

% Gerar dados de identificação
rng(0);
Input = idinput(length, 'rbs', [0 1/30], [-5 5]); % 1 entrada

% Simular sistema SIMO
% Uma entrada (ex: velocidade) afeta 4 saídas com ganhos diferentes
% Cada saída responde de forma diferente à mesma entrada
K = [1.5; 0.8; 1.2; 0.6];  % Ganhos para [x, y, z, yaw]
tau = [0.3; 0.4; 0.2; 0.5]; % Constantes de tempo diferentes

Output = zeros(length, 4); % 4 saídas
x_state = zeros(4,1);

for k = 1:length
    % Cada saída tem sua própria dinâmica, mas mesma entrada
    for i = 1:4
        x_state(i) = x_state(i) + (Ts/tau(i)) * (-x_state(i) + K(i)*Input(k));
    end
    Output(k,:) = x_state' + 0.01*randn(1,4);
end

% %% Plotar Input vs Output
% figure('Position', [100 100 1200 800]);

% % Plot Input
% subplot(5,1,1);
% plot(1:length, Input, 'r-', 'LineWidth', 1.5);
% ylabel('Input (Velocidade)');
% xlabel('Samples');
% grid on;
% title('Entrada Única');

% % Plot Outputs
% output_names = {'X', 'Y', 'Z', 'Yaw'};
% for i = 1:4
%     subplot(5,1,i+1);
%     plot(1:length, Output(:,i), 'b-', 'LineWidth', 1.5);
%     ylabel(['Output: ' output_names{i}]);
%     xlabel('Samples');
%     grid on;
%     title(['Saída ' num2str(i) ': ' output_names{i}]);
% end

%% Identificação do modelo MIMO
data = iddata(Output, Input, Ts);
identified_model = ssest(data, 4); % Modelo de espaço de estados

%% Criar MPC MIMO
predictionHorizon = 20;
controlHorizon = 5;

mpc1 = mpc(identified_model, Ts, predictionHorizon, controlHorizon);

% Configurar nomes
mpc1.Model.Plant.InputName = {'Motor1'};
mpc1.Model.Plant.OutputName = {'X', 'Y', 'Z', 'Yaw'};
mpc1.Model.Plant.InputUnit = {'PWM'};
mpc1.Model.Plant.OutputUnit = {'m', 'm', 'm', 'deg'};

mpc1.ManipulatedVariables.ScaleFactor = 10;

% Configurar saídas
for i = 1:4
    mpc1.OutputVariables(i).ScaleFactor = 5;
end

% mpcDesigner(mpc1);

%% Simulação SIMO com MPC
Ref = zeros(length, 4); % Referências para [x, y, z, yaw]
Ref(26:50, 1) = 5;   % Mover em X
Ref(51:75, 2) = 5;   % Mover em Y
Ref(76:100, 3) = 5;  % Mover em Z

y = zeros(length, 4);
u = zeros(length, 1);  % Agora é 1 entrada apenas
x_state_sim = zeros(4,1);
xmpc = mpcstate(mpc1);

for k = 2:length
    u(k) = mpcmove(mpc1, xmpc, y(k-1,:), Ref(k-1,:));
    
    % Simular sistema SIMO
    for i = 1:4
        x_state_sim(i) = x_state_sim(i) + (Ts/tau(i)) * (-x_state_sim(i) + K(i)*u(k));
    end
    y(k,:) = x_state_sim';
end

%% Plotar resultados
figure('Position', [100 100 1200 800]);

% Subplot para controle (1 entrada)
subplot(5,1,1);
plot(1:length, u, 'r-', 'LineWidth', 1.5);
ylabel('Control (u)');
xlabel('Samples');
grid on;
title('Sinal de Controle (1 entrada)');

% Subplots para saídas (4 saídas)
output_names = {'X', 'Y', 'Z', 'Yaw'};
for i = 1:4
    subplot(5,1,i+1);
    plot(1:length, Ref(:,i), 'k--', 'LineWidth', 1.5);
    hold on;
    plot(1:length, y(:,i), 'b-', 'LineWidth', 1.5);
    legend('Ref', 'Output');
    ylabel(output_names{i});
    xlabel('Samples');
    grid on;
end