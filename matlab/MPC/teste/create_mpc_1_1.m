%clear all;
length = 100;
K = 2;            % Ganho do sistema
tau = 0.2;        % Constante de tempo
Ts = 0.1;         % Período de amostragem

%% Inputs

rng(0);
Input = idinput(length, 'rbs', [0 1/30], [-7 7]);

%% Outputs

Output_ol = zeros(length, 1);
x = Input(1);
for i = 1:length
    x = x + (Ts/tau) * (-x + K*Input(i));
    Output_ol(i) = x + 0.1*randn();
end

data = iddata(Output_ol,Input,'Ts',Ts);
%data_preprocessed = detrend(data);
data_preprocessed = data;

identified_model = ssest(data_preprocessed,2,'Ts',Ts);

%% comparação

% % Gráfico de comparação (figura 1)
% figure;
% compare(data_preprocessed,identified_model);

% % Examine open-loop system
% figure;
% step(identified_model);

% figure;
% subplot(2,1,1);
% plot(data.InputData);
% hold on;
% plot(data_preprocessed.InputData);
% legend('Input Original', 'Input Detrended');
% ylabel('Input');
% grid on;

% subplot(2,1,2);
% plot(data.OutputData);
% hold on;
% plot(data_preprocessed.OutputData);
% legend('Output Original', 'Output Detrended');
% ylabel('Output');
% xlabel('# of Samples');
% grid on;

%% MPC

predictionHorizon = 10;
controlHorizon = 5;

mpc1 = mpc(identified_model,Ts,predictionHorizon,controlHorizon);
%-->Converting linear model from System Identification Toolbox to state-space.
%-->"Weights.ManipulatedVariables" is empty. Assuming default 0.00000.
%-->"Weights.ManipulatedVariablesRate" is empty. Assuming default 0.10000.
%-->"Weights.OutputVariables" is empty. Assuming default 1.00000.

mpc1.Weights.ManipulatedVariablesRate = 1.0;
mpc1.Weights.ManipulatedVariables = 0.1;

% Configurar nomes e unidades dos sinais (usar Model.Plant)
mpc1.Model.Plant.InputName = {'pwm'};
mpc1.Model.Plant.OutputName = {'velocidade'};
mpc1.Model.Plant.InputUnit = {'int'};
mpc1.Model.Plant.OutputUnit = {'rpm'};

% Variáveis Manipuladas (u - controle)
mpc1.ManipulatedVariables.Min = -10;
mpc1.ManipulatedVariables.Max = 10;
mpc1.ManipulatedVariables.ScaleFactor = 20;
mpc1.ManipulatedVariables.Target = 0;

% Variáveis de Saída (y - medição)
mpc1.OutputVariables.ScaleFactor = 5;
mpc1.OutputVariables.Min = -Inf;
mpc1.OutputVariables.Max = Inf;

mpcDesigner(mpc1);

%% resultado simulado

%length = 2500;
%Ref = zeros(length,1);
%Ref(501:1000) = -10;
%Ref(1501:2000) = 10;

Ref = Input;

y = zeros(length,1);    % Saída do sistema
u = zeros(length,1);    % Sinal de controle
x_state = 0;            % Estado interno do sistema simulado

% Criar estado inicial do MPC
xmpc = mpcstate(mpc1);

for k = 2:length
    % Calcular ação de controle do MPC
    u(k) = mpcmove(mpc1, xmpc, y(k-1), Ref(k));
    
    % Aplicar controle ao sistema (mesma dinâmica usada na identificação)
    x_state = x_state + (Ts/tau) * (-x_state + K*u(k));
    y(k) = x_state;

end


% Plotar resultados
figure;
subplot(2,1,1);
plot(1:length, Ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(1:length, y, 'b-', 'LineWidth', 1.5);
legend('Referência', 'Saída');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
plot(1:length, u, 'r-', 'LineWidth', 1.5);
ylabel('Controle');
xlabel('Amostras');
grid on;
