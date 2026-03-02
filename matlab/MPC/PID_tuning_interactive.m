function PID_tuning_interactive()
% Carregar dados
t_in_data = 0:0.01:10;
step_data = 2 * ones(size(t_in_data));
ramp_data = t_in_data;
seno_data = sin(2*pi*0.1*t_in_data);
step2_data = (t_in_data>=0)*1 + (t_in_data>=3)*1 + (t_in_data>=6)*(-2);

% Carregar arquivo SID
data = load('DATA.sid', '-mat');
tfx = data.Model{3};

% Escolher entrada
in_data = step2_data;

% Calcular valores iniciais dos ganhos usando pidtune
PID_auto = pidtune(tfx, 'PID');
Kp_init = PID_auto.Kp;
Ki_init = PID_auto.Ki;
Kd_init = PID_auto.Kd;

% Criar figura com sliders
fig = uifigure('Name', 'PID Tuning Interativo', 'Position', [100 100 800 600]);

% Criar painel para sliders
panel = uipanel(fig, 'Position', [10 10 200 580]);

% Slider Kp
lbl_kp = uilabel(panel, 'Position', [10 520 180 22], 'Text', 'Kp = 1.00');
slider_kp = uislider(panel, 'Position', [10 500 180 3], ...
    'Limits', [0 20], 'Value', Kp_init, ...
    'ValueChangedFcn', @(src, event) updatePlot());

% Slider Ki
lbl_ki = uilabel(panel, 'Position', [10 420 180 22], 'Text', 'Ki = 0.10');
slider_ki = uislider(panel, 'Position', [10 400 180 3], ...
    'Limits', [0 5], 'Value', Ki_init, ...
    'ValueChangedFcn', @(src, event) updatePlot());

% Slider Kd
lbl_kd = uilabel(panel, 'Position', [10 320 180 22], 'Text', 'Kd = 0.01');
slider_kd = uislider(panel, 'Position', [10 300 180 3], ...
    'Limits', [0 2], 'Value', Kd_init, ...
    'ValueChangedFcn', @(src, event) updatePlot());

% Labels para informações do sistema
lbl_info = uilabel(panel, 'Position', [10 200 180 80], ...
    'Text', 'Ajuste os ganhos...', 'WordWrap', 'on');

% Criar figura maior para 4 subplots
fig.Position = [100 100 1000 700];

% Criar 4 eixos para os gráficos
ax1 = uiaxes(fig, 'Position', [230 380 350 280]);
ax2 = uiaxes(fig, 'Position', [600 380 350 280]);
ax3 = uiaxes(fig, 'Position', [230 50 350 280]);
ax4 = uiaxes(fig, 'Position', [600 50 350 280]);

% Plotar inicial
updatePlot();

    function updatePlot()
        % Obter valores dos sliders
        Kp = slider_kp.Value;
        Ki = slider_ki.Value;
        Kd = slider_kd.Value;
        
        % Atualizar labels
        lbl_kp.Text = sprintf('Kp = %.2f', Kp);
        lbl_ki.Text = sprintf('Ki = %.2f', Ki);
        lbl_kd.Text = sprintf('Kd = %.3f', Kd);
        
        % Criar PID e sistema em malha fechada
        PID = pid(Kp, Ki, Kd);
        sys_closed = feedback(PID * tfx, 1);
        
        % Calcular informações
        try
            info = stepinfo(sys_closed);
            info_text = sprintf('Rise Time: %.2fs\nSettle Time: %.2fs\nOvershoot: %.1f%%', ...
                info.RiseTime, info.SettlingTime, info.Overshoot);
        catch
            info_text = 'Sistema instável';
        end
        lbl_info.Text = info_text;
        
        % Exibir ganhos no console
        fprintf('Kp = %.2f, Ki = %.2f, Kd = %.3f\n', Kp, Ki, Kd);
        
        % Plot 1: Degrau
        [out1, t1] = lsim(sys_closed, step_data, t_in_data);
        plotResponse(ax1, t_in_data, step_data, t1, out1, 'Degrau (Amplitude 2)');
        
        % Plot 2: Rampa
        [out2, t2] = lsim(sys_closed, ramp_data, t_in_data);
        plotResponse(ax2, t_in_data, ramp_data, t2, out2, 'Rampa');
        
        % Plot 3: Senoide
        [out3, t3] = lsim(sys_closed, seno_data, t_in_data);
        plotResponse(ax3, t_in_data, seno_data, t3, out3, 'Senoide (0.5 Hz)');
        
        % Plot 4: Múltiplos Degraus
        [out4, t4] = lsim(sys_closed, step2_data, t_in_data);
        plotResponse(ax4, t_in_data, step2_data, t4, out4, 'Múltiplos Degraus');
    end

    function plotResponse(ax, t_in, in_signal, t_out, out_signal, titleText)
        cla(ax);
        plot(ax, t_in, in_signal, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Entrada');
        hold(ax, 'on');
        plot(ax, t_out, out_signal, 'r', 'LineWidth', 1.2, 'DisplayName', 'Saída');
        hold(ax, 'off');
        title(ax, titleText);
        xlabel(ax, 'Tempo (s)');
        ylabel(ax, 'Amplitude');
        legend(ax, 'show', 'Location', 'best');
        grid(ax, 'on');
    end
end
