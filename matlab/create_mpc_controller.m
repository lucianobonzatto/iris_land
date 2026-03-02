%% create_mpc_controller.m
% Cria e salva um controlador MPC baseado em modelo identificado
% Saída: arquivo 'drone_mpc.mat' contendo o controlador

clear all;
close all;

%% Parâmetros do sistema
Ts = 0.1;  % Tempo de amostragem
predictionHorizon = 20;
controlHorizon = 5;

%% Gerar dados de identificação
fprintf('Gerando dados de treinamento...\n');
length = 200;
rng(0);

% Sinais de entrada: velocidades comandadas
Input = zeros(length, 4);
Input(:,1) = idinput(length, 'rbs', [0 1/30], [-2 2]); % Vx
Input(:,2) = idinput(length, 'rbs', [0 1/30], [-2 2]); % Vy
Input(:,3) = idinput(length, 'rbs', [0 1/30], [-1 1]); % Vz
Input(:,4) = idinput(length, 'rbs', [0 1/30], [-1 1]); % Vyaw

% Simular planta: velocidade -> posição
K = eye(4);  % Ganho unitário
tau = [0.5; 0.5; 0.4; 0.3]; % Constantes de tempo [x, y, z, yaw]
Output = zeros(length, 4);
x_state = zeros(4,1);

for k = 1:length
    for i = 1:4
        x_state(i) = x_state(i) + (Ts/tau(i)) * (-x_state(i) + K(i,i)*Input(k,i));
    end
    Output(k,:) = x_state' + 0.01*randn(1,4);
end

%% Identificar modelo da planta
fprintf('Identificando modelo da planta...\n');
data = iddata(Output, Input, Ts);
identified_model = ssest(data, 4);

fprintf('Modelo identificado:\n');
fprintf('  Estados: %d\n', size(identified_model.A, 1));
fprintf('  Entradas: %d\n', size(identified_model.B, 2));
fprintf('  Saídas: %d\n', size(identified_model.C, 1));

%% Criar controlador MPC
fprintf('Criando controlador MPC...\n');
plant = setmpcsignals(identified_model, 'MV', [1 2 3 4]);
mpcobj = mpc(plant, Ts, predictionHorizon, controlHorizon);

% Configurar nomes e unidades
mpcobj.Model.Plant.InputName = {'Vx_cmd', 'Vy_cmd', 'Vz_cmd', 'Vyaw_cmd'};
mpcobj.Model.Plant.InputUnit = {'m/s', 'm/s', 'm/s', 'deg/s'};
mpcobj.Model.Plant.OutputName = {'X', 'Y', 'Z', 'Yaw'};
mpcobj.Model.Plant.OutputUnit = {'m', 'm', 'm', 'deg'};

% Configurar restrições
for i = 1:4
    mpcobj.ManipulatedVariables(i).Min = -5;
    mpcobj.ManipulatedVariables(i).Max = 5;
    mpcobj.ManipulatedVariables(i).RateMin = -2;
    mpcobj.ManipulatedVariables(i).RateMax = 2;
    mpcobj.ManipulatedVariables(i).ScaleFactor = 2;
end

% Configurar pesos nas saídas
for i = 1:4
    mpcobj.OutputVariables(i).ScaleFactor = 5;
end

% Pesos personalizados (ajuste conforme necessário)
mpcobj.Weights.OutputVariables = [1 1 2 0.5];  % Prioriza Z
mpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.05 0.05 0.05 0.05];

%% Salvar controlador e parâmetros
fprintf('Salvando controlador MPC...\n');
% Salvar também tau e K para usar na simulação
save('drone_mpc.mat', 'mpcobj', 'tau', 'K', 'Ts');

fprintf('Controlador MPC salvo em: drone_mpc.mat\n');
fprintf('Use load(''drone_mpc.mat'') para carregar.\n\n');

% Exibir informações
fprintf('Parâmetros do MPC:\n');
fprintf('  Horizonte de predição: %d\n', predictionHorizon);
fprintf('  Horizonte de controle: %d\n', controlHorizon);
fprintf('  Tempo de amostragem: %.2f s\n', Ts);
fprintf('  Entradas (MVs): %d\n', size(mpcobj.Model.Plant.B, 2));
fprintf('  Saídas: %d\n', size(mpcobj.Model.Plant.C, 1));
