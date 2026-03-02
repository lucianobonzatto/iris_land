%% create_mpc_controller.m
% Cria e salva um controlador MPC baseado em modelo identificado
% Saída: arquivo 'drone_mpc.mat' contendo o controlador

clear all;
close all;

%% Carregar dados reais coletados
fprintf('Carregando dados coletados...\n');

% Listar arquivos disponíveis
files = dir('mpc_run_data_*.mat');
if isempty(files)
    error(['Nenhum arquivo de dados encontrado!\n' ...
           'Execute robot_go.m primeiro para coletar dados.']);
end

% Usar o arquivo mais recente
[~, idx] = max([files.datenum]);
data_file = files(idx).name;
fprintf('Carregando: %s\n', data_file);
load(data_file); % Carrega: Input, Output, Time

% Calcular período de amostragem dos dados reais
Ts = mean(diff(Time));
fprintf('Período de amostragem calculado: Ts = %.4f s\n', Ts);

fprintf('Dados carregados:\n');
fprintf('  - %d amostras\n', size(Input, 1));
fprintf('  - Duração: %.1f s\n', Time(end));
fprintf('  - Entradas: %d canais\n', size(Input, 2));
fprintf('  - Saídas: %d canais\n', size(Output, 2));

%% Parâmetros do MPC
predictionHorizon = 20;
controlHorizon = 5;

%% Parâmetros do MPC
predictionHorizon = 20;
controlHorizon = 5;

%% Usar apenas canais X e Y (robô 2D)
Input_2D = Input(:, 1:2);   % Vx, Vy
Output_2D = Output(:, 1:2); % X, Y

%% Identificar modelo da planta
fprintf('\nIdentificando modelo da planta...\n');
data = iddata(Output_2D, Input_2D, Ts);
identified_model = ssest(data, 4); % 4 estados (2 para cada eixo)

fprintf('Modelo identificado:\n');
fprintf('  Estados: %d\n', size(identified_model.A, 1));
fprintf('  Entradas: %d\n', size(identified_model.B, 2));
fprintf('  Saídas: %d\n', size(identified_model.C, 1));

% Avaliar qualidade do modelo
fprintf('\nValidando modelo...\n');
% Criar vetor de tempo uniforme para lsim (requer espaçamento regular)
N = size(Input_2D, 1);
Time_uniform = (0:N-1)' * Ts;
y_pred = lsim(identified_model, Input_2D, Time_uniform);
fit_x = 100 * (1 - norm(Output_2D(:,1) - y_pred(:,1)) / norm(Output_2D(:,1) - mean(Output_2D(:,1))));
fit_y = 100 * (1 - norm(Output_2D(:,2) - y_pred(:,2)) / norm(Output_2D(:,2) - mean(Output_2D(:,2))));
fprintf('  Qualidade do ajuste (FIT):\n');
fprintf('    - Eixo X: %.2f%%\n', fit_x);
fprintf('    - Eixo Y: %.2f%%\n', fit_y);

if fit_x < 50 || fit_y < 50
    warning('Qualidade do modelo baixa! FIT < 50%%. Considere coletar mais dados ou com mais variação.');
end

%% Criar controlador MPC
fprintf('\nCriando controlador MPC...\n');
plant = setmpcsignals(identified_model, 'MV', [1 2]);
mpcobj = mpc(plant, Ts, predictionHorizon, controlHorizon);

% Configurar nomes e unidades
mpcobj.Model.Plant.InputName = {'Vx_cmd', 'Vy_cmd'};
mpcobj.Model.Plant.InputUnit = {'m/s', 'm/s'};
mpcobj.Model.Plant.OutputName = {'X', 'Y'};
mpcobj.Model.Plant.OutputUnit = {'m', 'm'};

% Configurar restrições
for i = 1:2
    mpcobj.ManipulatedVariables(i).Min = -5;
    mpcobj.ManipulatedVariables(i).Max = 5;
    mpcobj.ManipulatedVariables(i).RateMin = -2;
    mpcobj.ManipulatedVariables(i).RateMax = 2;
    mpcobj.ManipulatedVariables(i).ScaleFactor = 2;
end

% Configurar pesos nas saídas
for i = 1:2
    mpcobj.OutputVariables(i).ScaleFactor = 5;
end

% Pesos personalizados
mpcobj.Weights.OutputVariables = [1 1];  % Igual importância X e Y
mpcobj.Weights.ManipulatedVariables = [0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.05 0.05];

%% Salvar controlador e parâmetros
fprintf('\nSalvando controlador MPC...\n');
save('drone_mpc.mat', 'mpcobj', 'identified_model', 'Ts', 'fit_x', 'fit_y');

fprintf('Controlador MPC salvo em: drone_mpc.mat\n');
fprintf('Use load(''drone_mpc.mat'') para carregar.\n\n');

% Exibir informações
fprintf('Parâmetros do MPC:\n');
fprintf('  Horizonte de predição: %d\n', predictionHorizon);
fprintf('  Horizonte de controle: %d\n', controlHorizon);
fprintf('  Tempo de amostragem: %.4f s\n', Ts);
fprintf('  Entradas (MVs): %d\n', size(mpcobj.Model.Plant.B, 2));
fprintf('  Saídas: %d\n', size(mpcobj.Model.Plant.C, 1));
fprintf('  Qualidade do modelo: X=%.1f%%, Y=%.1f%%\n', fit_x, fit_y);

%% Visualizar validação do modelo
figure('Name', 'Validação do Modelo Identificado');

subplot(2,1,1);
plot(Time_uniform, Output_2D(:,1), 'b', 'LineWidth', 2); hold on;
plot(Time_uniform, y_pred(:,1), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Posição X (m)');
title(sprintf('Validação Eixo X (FIT = %.1f%%)', fit_x));
legend('Dados Reais', 'Modelo Identificado', 'Location', 'best');

subplot(2,1,2);
plot(Time_uniform, Output_2D(:,2), 'b', 'LineWidth', 2); hold on;
plot(Time_uniform, y_pred(:,2), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Posição Y (m)');
title(sprintf('Validação Eixo Y (FIT = %.1f%%)', fit_y));
legend('Dados Reais', 'Modelo Identificado', 'Location', 'best');

fprintf('\n✓ MPC criado com sucesso usando dados reais!\n');
