clear all;
length = 200;
Ts = 0.1;
predictionHorizon = 20;
controlHorizon = 5;

rng(0);
Input = zeros(length, 4);
Input(:,1) = idinput(length, 'rbs', [0 1/30], [-2 2]); % MV: Velocidade comandada X
Input(:,2) = idinput(length, 'rbs', [0 1/30], [-2 2]); % MV: Velocidade comandada Y
Input(:,3) = idinput(length, 'rbs', [0 1/30], [-1 1]); % MV: Velocidade comandada Z
Input(:,4) = idinput(length, 'rbs', [0 1/30], [-1 1]); % MV: Velocidade comandada Yaw

K = eye(4);  % Ganho unitário (velocidade -> posição)
tau = [0.5; 0.5; 0.4; 0.3]; % Constantes de tempo para [x, y, z, yaw]
Output = zeros(length, 4); % 4 saídas: [X, Y, Z, Yaw]
x_state = zeros(4,1);

% Simular dinâmica: velocidade -> posição (integração com inércia)
for k = 1:length
    for i = 1:4
        % Velocidade comandada afeta posição: tau*dx/dt + x = integral(vel)
        x_state(i) = x_state(i) + (Ts/tau(i)) * (-x_state(i) + K(i,i)*Input(k,i));
    end
    Output(k,:) = x_state' + 0.01*randn(1,4);
end

data = iddata(Output, Input(:,1:4), Ts);  % Apenas 4 entradas MV
identified_model = ssest(data, 4);
plant = setmpcsignals(identified_model, 'MV', [1 2 3 4]);
mpc1 = mpc(plant, Ts, predictionHorizon, controlHorizon);

% Configurar nomes
mpc1.Model.Plant.InputName = {'Vx_cmd', 'Vy_cmd', 'Vz_cmd', 'Vyaw_cmd'};
mpc1.Model.Plant.InputUnit = {'m/s', 'm/s', 'm/s', 'deg/s'};
mpc1.Model.Plant.OutputName = {'X', 'Y', 'Z', 'Yaw'};
mpc1.Model.Plant.OutputUnit = {'m', 'm', 'm', 'deg'};

for i = 1:4
    mpc1.ManipulatedVariables(i).Min = -5;
    mpc1.ManipulatedVariables(i).Max = 5;
    mpc1.ManipulatedVariables(i).ScaleFactor = 2;
    mpc1.OutputVariables(i).ScaleFactor = 5;
end

%% Simulação: Teste de seguimento de referência
sim_length = 150;
y = zeros(sim_length, 4);  % Posição atual do drone (medida)
u = zeros(sim_length, 4);  % Velocidades comandadas (saída do MPC)
ref = zeros(sim_length, 4); % Referência de posição

% Definir trajetória de referência
ref(1:50, 1) = 0;
ref(51:100, 1) = 5;    % Move para X=5m
ref(101:end, 1) = 5;

ref(1:70, 2) = 0;
ref(71:end, 2) = 3;    % Move para Y=3m

ref(:, 3) = 2;         % Altitude constante Z=2m
ref(:, 4) = 0;         % Yaw=0

% Inicializar estado da planta simulada
x_state_sim = zeros(4,1);
xmpc = mpcstate(mpc1);

for k = 2:sim_length
    % MPC calcula VELOCIDADES baseado em:
    % - Posição atual: y(k-1,:)
    % - Referência desejada: ref(k,:)
    u(k,:) = mpcmove(mpc1, xmpc, y(k-1,:), ref(k,:));
    
    % Simular dinâmica real do drone: velocidade -> posição
    for i = 1:4
        x_state_sim(i) = x_state_sim(i) + (Ts/tau(i)) * (-x_state_sim(i) + K(i,i)*u(k,i));
    end
    y(k,:) = x_state_sim' + 0.001*randn(1,4); % Posição medida (com ruído)
end

%% Plotar resultados
figure('Position', [100 100 1200 800]);

% Velocidades comandadas
for i = 1:4
    subplot(4,2,i);
    plot(1:sim_length, u(:,i), 'r-', 'LineWidth', 1.5);
    ylabel(['V_' num2str(i)]);
    xlabel('Amostra');
    grid on;
    title(['Velocidade Comandada ' num2str(i)]);
end

% Posições
pos_names = {'X', 'Y', 'Z', 'Yaw'};
for i = 1:4
    subplot(4,2,4+i);
    plot(1:sim_length, ref(:,i), 'k--', 'LineWidth', 1.5);
    hold on;
    plot(1:sim_length, y(:,i), 'b-', 'LineWidth', 1.5);
    legend('Referência', 'Posição Real');
    ylabel(pos_names{i});
    xlabel('Amostra');
    grid on;
    title(['Posição: ' pos_names{i}]);
end

sgtitle('MPC: Velocidade → Posição');
