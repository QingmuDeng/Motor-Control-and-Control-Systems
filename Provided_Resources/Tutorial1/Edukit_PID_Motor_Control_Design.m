%               Edukit_PID_Motor_Control_Design.m
%
%
%                 Edukit Rotary Inverted Pendulum
%
%               PID Controller Design for Rotor Control
%
% This system is developed for design of controllers of Rotor Position
% Pendulum angle or other Pendulum properties are not included.
%
%
% This includes the steps of:
%
% 1) Determining initial values of Proportional, Integral, and Derivative
% 2) Presenting analysis of stability by Nyquist and Bode diagram analysis
% 3) Computation of Sensitivity Functions for performance analysis
%
% Both Proportional-Derivative (PD) and Proportional-Integral-Derivative
% (PID) control system designs are supported.
%

clear all
close all

fprintf('Select Output Data Directory for Edukit Data and Figures\r\n');
selpath = uigetdir;
selpath;
if isequal(selpath,0)
    disp('User selected Cancel');
else
    disp(['User selected ', selpath]);
end

% Figure position parameters

startx = 0.6;
starty = 0.5;
endx = 0.39;
endy = 0.5;
endy_triple = 0.8;

global exit_state;
exit_state = 0;

%
% Motor Plant Transfer Function of Rotor Angle response to acceleration
% command input
%


% Rotor Plant Transfer Function from control
% input (degree units) to rotor position (degree units).
%

%
% Optional Plant Design Selection
%


plant_design_select = "";

plant_design_select = input('Enter "y" for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) ','s');
if plant_design_select == "y" || plant_design_select == "Y"
    Wn = input('Enter Wn coefficient (0 < Wn < 10) : ');
    
    Plant_Gain = 1;
    if Wn < 0 || Wn > 10
        disp(sprintf('Wn Frequency Limits: (0 <= Wn <= 10) '));
        return
    end
    
    a = 1;
    b = Wn;
    c = 0;
    
end

if plant_design_select ~= "y"
    a = 1;
    b = 0;
    c = 0;
end

num_rotor = [0, 0, a];
den_rotor = [1, b, c];

Grotor = tf(num_rotor, den_rotor);

bode_opts = bodeoptions;
bode_opts.PhaseMatching = 'on';
bode_opts.PhaseMatchingFreq = 0.0;
bode_opts.Title.FontSize = 12;
bode_opts.Title.FontWeight = 'Bold';
bode_opts.XLabel.FontSize = 12;
bode_opts.YLabel.FontSize = 12;
bode_opts.TickLabel.FontSize = 12;

figure('units', 'normalized', 'Position',[startx starty endx endy]);
bode(Grotor,bode_opts)
title("Rotor Plant Open Loop Bode Plot")
next_plot; if exit_state == 1 return; end;

t = 0:0.001:10;
figure('units', 'normalized', 'Position',[startx starty endx endy]);
impulse(Grotor,t)
set(findall(gca,'Type','line'),'LineWidth',1)
title("Rotor Plant Open Loop Impulse Response")
next_plot; if exit_state == 1 return; end;

figure('units', 'normalized', 'Position',[startx starty endx endy]);
pzplot(Grotor)
set(findall(gca,'Type','line'),'LineWidth',2)
title("Poles and Zeroes of Rotor Plant");
next_plot; if exit_state == 1 return; end;

disp(sprintf("Rotor Plant Poles"));
disp(pole(Grotor));

disp(sprintf("Rotor Plant Zeroes"));
if length(zero(Grotor)) == 0
  disp(sprintf("Inf"));
end
if length(zero(Grotor)) > 0
  disp(zero(Grotor));
end



%
% Derivative Low Pass Filter definition,  The Edukit system includes
% a low pass filter operating on the derivative computation.  This may
% be disabled by setting Derivative_Low_Pass_Enable = 0;

Derivative_Low_Pass_Enable = 1;

disp(sprintf('\n\nRotor Controller Tuning Start With Impulse Response, Nyquist Analysis and Gain and Phase Margin and Sensitivity Function Guidance\n\n'));

answer = questdlg('Select PD or PID Design','PD', 'PD','PID','PD');
switch answer
    case 'PD'
        select_PID = 1;
    case 'PID'
        select_PID = 2;
end


while ( 1 )
    
    close all
    
    prompt = {'Enter Derivative Time Constant'};
    dlgtitle = 'Input';
    dims = [1 40];
    definput = {'1'};
    opts.Interpreter = 'tex';
    try
        user_entry = inputdlg(prompt,dlgtitle,dims,definput,opts);
        Tdr = str2num(cell2mat(user_entry(1)));
    catch
        disp(sprintf('No entry - assigning a value of 1'));
        Tdr = 1;
    end
    
    if select_PID == 2
        prompt = {'Enter Integral Time Constant'};
        dlgtitle = 'Input';
        dims = [1 40];
        definput = {'5'};
        opts.Interpreter = 'tex';
        try
            user_entry = inputdlg(prompt,dlgtitle,dims,definput,opts);
            Tir = str2num(cell2mat(user_entry(1)));
        catch
            disp(sprintf('No entry - assigning a value of 5'));
            Tir = 5;
        end
    end
    
    prompt = {'Enter Gain'};
    dlgtitle = 'Input';
    dims = [1 40];
    definput = {'1'};
    opts.Interpreter = 'tex';
    try
        user_entry = inputdlg(prompt,dlgtitle,dims,definput,opts);
        k = str2num(cell2mat(user_entry(1)));
    catch
        disp(sprintf('No entry - assigning a value of 11'));
        k = 1;
    end
    
    Kprotor = k;
    
    if select_PID == 1
        Kirotor = 0;
    else
        Kirotor = Kprotor/Tir;
    end
    
    Kdrotor = Kprotor*Tdr;
    
    %
    % The Rotor Control PID controller will include a 50 Hz low pass filter for
    % derivative computation.
    %
    
    Krotor_pid_fc = 50;
    Tf_rotor = 1/(2*pi*Krotor_pid_fc);
    
    %
    % Note that Suspended Mode Angle definitions require
    % negative gain values.  Thus, the Krotor controller
    % is defined with negative gains.
    %
    
    if  Derivative_Low_Pass_Enable == 0
        Krotor = pid(Kprotor, Kirotor, Kdrotor);
    end
    
    if  Derivative_Low_Pass_Enable == 1
        Krotor = pid(Kprotor, Kirotor, Kdrotor, Tf_rotor);
    end
    
    
    str = sprintf('Rotor PID Controller Gains: Kp = %0.02f, Ki = %0.02f, Kd = %0.02f', Kprotor, Kirotor, Kdrotor);
    disp(str);
    
    figure('units', 'normalized', 'Position',[startx starty endx endy]);
    bode(Grotor,Krotor,Krotor*Grotor,bode_opts)
    set(findall(gca,'Type','line'),'LineWidth',2)
    legend('Rotor Plant', 'Rotor Controller', 'Rotor Controller and Rotor Plant')
    title(sprintf('Motor Controller K = %0.2f Bode Plot', Kprotor));
    next_plot; if exit_state == 1 return; end;
    
    %
    % Grotor_cl is the closed loop transfer function for the rotor angle
    % feedback control system
    %
    
    
    Grotor_cl = feedback(Grotor*Krotor,1);
    
    figure('units', 'normalized', 'Position',[startx starty endx endy_triple]);
    subplot(3,1,1);
    step(Grotor_cl,10);
    title('Closed Loop Rotor Angle Step Response');
    subplot(3,1,2);
    nyquist(Grotor*Krotor);
    axis([-8 1 -4 4])
    subplot(3,1,3);
    margin(minreal(Grotor*Krotor));
    next_plot; if exit_state == 1 return; end;
    
    disp(sprintf("Poles of Trotor*Krotor (Poles of Open Loop Transfer Function):"));
    disp(pole(Krotor*Grotor));
    
    %
    % Evaluate Nyquist Stability Criterion:
    % P is number of poles of 1 + Krotor*Grotor in the right half plane
    % Stability is achieved if
    % N = P is the number of CCW encirclements of s = -1 + 0i
    
    % Count Close Loop Poles in RHP of Krotor*Grotor
    
    rotor_poles= pole(Krotor*Grotor);
    num_rhp_poles = 0;
    for i = 1:length(rotor_poles)
        if real(rotor_poles(i)) > 0
            num_rhp_poles = num_rhp_poles + 1;
        end
    end
    
    num_ccw_encirclements = num_rhp_poles;
    disp(sprintf('Nyquist Criterion: Number of CCW Encirclements of s = -1 : %i', num_ccw_encirclements));
    
    [Gm, Pm, Wcg, Wcp] = margin(minreal(Grotor*Krotor));
    disp(sprintf("Gain margin: %0.02f, Phase margin: %0.02f", Gm, Pm));
    disp(sprintf("Gain cross over frequency: %0.02f, Phase cross over frequency: %0.02f", Wcg, Wcp));
    
    % Settling time estimate
    ST = 0.02;
    t = 0:0.001:10;
    [step_response,t] = step(Grotor_cl,t);
    S = stepinfo(step_response, t, 'SettlingTimeThreshold',ST);
    
    disp(sprintf("Rotor Angle Step Time to Peak is %0.2f seconds at value of %0.2f with Overshoot of %0.2f Percent and Settling Time to Tolerance of 2 Percent of %0.2f seconds", S.PeakTime, S.Peak, S.Overshoot, S.SettlingTime));
    
    
    %
    % Compute maxima of Sensitivity Functions
    %
    
    [mag,phase,wout] = bode(1/(1 + Grotor*Krotor));
    Ms = max(mag);
    disp(sprintf("Maximum of Sensitivity Function: %0.02f", Ms));
    
    [mag,phase,wout] = bode(Grotor*Krotor/(1 + Grotor*Krotor));
    CSs = max(mag);
    disp(sprintf("Maximum of Complimentary Sensitivity Function: %0.02f", CSs));
    
    [mag,phase,wout] = bode(Krotor/(1 + Grotor*Krotor));
    NSs = max(mag);
    disp(sprintf("Maximum of Noise Sensitivity Function: %0.02f", NSs));
    
    [mag,phase,wout] = bode(Grotor/(1 + Grotor*Krotor));
    LDs = max(mag);
    disp(sprintf("Maximum of Load Disturbance Sensitivity Function: %0.04f", LDs));
    
    %
    % Generate and display figures
    %
    
    figure_display_option = '';
    answer = questdlg('Enter Yes to Display Sensitivity Function Figures', 'Yes', 'Yes', 'Continue', 'Yes');
    switch answer
        case 'Yes'
            figure_display_option = 'y';
            close all;
        case 'Continue'
            close all;
    end
    
    
    if figure_display_option == 'y'
        
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        subplot(2,1,1)
        t = 0:0.001:10;
        step_opt = stepDataOptions('StepAmplitude', 1);
        step(Grotor_cl,t,step_opt)
        title("Rotor Angle Control System Step Response");
        subplot(2,1,2)
        impulse(Grotor_cl,t)
        title("Rotor Angle Control System Impulse Response")
        next_plot; if exit_state == 1 return; end;
        
        w = 0.01:0.01:100;
        
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        [mag,phase,w] = bode(Grotor*Krotor/(1 + Grotor*Krotor), w);
        mag = squeeze(mag);
        semilogx(w, 20*log10(mag),'k');
        xlim([0.1 100])
        xlabel("Frequency (rad/sec)");
        ylabel("Complimentary Sensitivity Function Magnitude (dB)");
        title("Complimentary Sensitivity Function");
        next_plot; if exit_state == 1 return; end;
        
        
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        [mag,phase,w] = bode(1/(1 + Grotor*Krotor), w);
        mag = squeeze(mag);
        semilogx(w, 20*log10(mag),'k');
        xlim([0.1 100])
        xlabel("Frequency (rad/sec)");
        ylabel("Sensitivity Function Magnitude (dB)");
        title("Sensitivity Function");
        next_plot; if exit_state == 1 return; end;
        
        
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        [mag,phase,w] = bode(Grotor/(1 + Grotor*Krotor), w);
        mag = squeeze(mag);
        semilogx(w, 20*log10(mag),'k');
        xlim([0.1 100])
        title("Load Disturbance Sensitivity Function");
        ylabel("Load Disturbance Sensitivity Function Magnitude (dB)");
        xlabel("Frequency (rad/sec)");
        title("Load Disturbance Sensitivity Function");
        next_plot; if exit_state == 1 return; end;
        
        
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        [mag,phase,w] = bode(Krotor/(1 + Grotor*Krotor), w);
        mag = squeeze(mag);
        semilogx(w, 20*log10(mag),'k');
        xlim([0.1 100])
        title("Noise Sensitivity Function");
        ylabel("Noise Sensitivity Function Magnitude (dB)");
        xlabel("Frequency (rad/sec)");
        next_plot; if exit_state == 1 return; end;
        
        
        step_increment = 0.002;
        step_start = -1;
        step_end = 10;
        t = 0:step_increment:step_end;
        [step_resp, t] = step(Grotor*Krotor/(1 + Grotor*Krotor),t);
        S = stepinfo(step_resp,t);
        t = step_start:step_increment:step_end - step_increment;
        
        for m = 1:(step_end - step_start)/step_increment
            if t(m) < 0
                step_input(m) = 0;
                step_plot(m) = 0;
            else
                step_input(m) = 1;
                step_plot(m) = step_resp(m + step_start/step_increment);
            end
        end
        
        [diff,index] = min(abs(t-S.SettlingTime));
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        plot(t, step_plot,'k');
        hold on
        plot(S.PeakTime, S.Peak,'*g');
        hold on
        plot(S.SettlingTime, step_plot(index) ,'*r');
        hold on
        plot(t, step_input,'--k');
        xlim([-1 10]);
        title("Rotor Angle Control System Step Response");
        legend("Rotor Angle Response", "Rotor Angle Reference", "First Peak", "Settling Time");
        ylabel("Rotor Angle Amplitude");
        xlabel("Sample Time (seconds)");
        next_plot; if exit_state == 1 return; end;
        
        step_increment = 0.002;
        step_start = -1;
        step_end = 10;
        t = 0:step_increment:step_end;
        [step_resp, t] = step(Grotor/(1 + Grotor*Krotor),t);
        S = stepinfo(step_resp,t);
        
        disp(sprintf("Load Disturbance Time to Peak is %0.2f seconds at value of %0.2f with Overshoot of %0.2f Percent and Settling Time to Tolerance of 2 Percent of %0.2f seconds", S.PeakTime, S.Peak, S.Overshoot, S.SettlingTime));
        
        t = step_start:step_increment:step_end - step_increment;
        for m = 1:(step_end - step_start)/step_increment
            if t(m) < 0
                step_input(m) = 0;
                step_plot(m) = 0;
            else
                step_input(m) = 1;
                step_plot(m) = step_resp(m + step_start/step_increment);
            end
        end
        
        [diff,index] = min(abs(t-S.SettlingTime));
        fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        yyaxis left
        plot(t, step_plot,'k');
        ylabel("Sensitivity Function Amplitude");
        hold on
        plot(S.PeakTime, S.Peak,'*g');
        hold on
        plot(S.SettlingTime,step_plot(index) ,'*r');
        hold on
        yyaxis right
        plot(t, step_input,'--k');
        ylabel("Step Input Amplitude");
        xlim([-1 10]);
        title("Load Disturbance Rejection Sensitivity Function Step Response");
        legend("Load Sensitivity Function Response", "First Peak", "Settling Time", "Load Step Input Reference");
        xlabel("Sample Time (seconds)");
        next_plot; if exit_state == 1 return; end;
        
    end
    
    answer = questdlg('Enter Yes to Exit and Save Figures or Continue to proceed with design','Yes', 'Yes','Continue','Yes');
    switch answer
        case 'Yes'
            break;
        case 'Continue'
            close all;
    end
    
end

%
% Generate and save figures
%


fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
[mag,phase,w] = bode(Grotor*Krotor/(1 + Grotor*Krotor),  w);
mag = squeeze(mag);
semilogx(w, 20*log10(mag),'k');
xlim([0.1 100])
xlabel("Frequency (rad/sec)");
ylabel("Complimentary Sensitivity Function Magnitude (dB)");
title("Complimentary Sensitivity Function");
fig_str = 'Complimentary_Sensitivity_Function_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;


fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
[mag,phase,w] = bode(1/(1 + Grotor*Krotor),  w);
mag = squeeze(mag);
semilogx(w, 20*log10(mag),'k');
xlim([0.1 100])
xlabel("Frequency (rad/sec)");
ylabel("Sensitivity Function Magnitude (dB)");
title("Sensitivity Function");
fig_str = 'Sensitivity_Function_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

step_increment = 0.001;
step_start = -1;
step_end = 10;
t = 0:step_increment:step_end;
[step_resp, t] = step(Grotor/(1 + Grotor*Krotor),t);
S = stepinfo(step_resp,t);


t = step_start:step_increment:step_end - step_increment;
for m = 1:(step_end - step_start)/step_increment
    if t(m) < 0
        step_input(m) = 0;
        step_plot(m) = 0;
    else
        step_input(m) = 1;
        step_plot(m) = step_resp(m + step_start/step_increment);
    end
end

[diff,index] = min(abs(t-S.SettlingTime));
fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
yyaxis left
plot(t, step_plot,'k');
ylabel("Sensitivity Function Amplitude");
hold on
plot(S.PeakTime, S.Peak,'*g');
hold on
plot(S.SettlingTime,step_plot(index) ,'*r');
hold on
yyaxis right
plot(t, step_input,'--k');
ylabel("Step Input Amplitude");
xlim([-1 10]);
title("Load Disturbance Rejection Sensitivity Function Step Response");
legend("Load Sensitivity Function Response", "Load Step Input Reference", "First Peak", "Settling Time");
xlabel("Sample Time (seconds)");
fig_str = 'Load_Disturbance_Sensitivity_Step_Response_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
[mag,phase,w] = bode(Grotor/(1 + Grotor*Krotor),  w);
mag = squeeze(mag);
semilogx(w, 20*log10(mag),'k');
xlim([0.1 100])
title("Load Disturbance Sensitivity Function");
ylabel("Load Disturbance Sensitivity Function Magnitude (dB)");
xlabel("Frequency (rad/sec)");
title("Load Disturbance Sensitivity Function");
fig_str = 'Load_Disturbance_Sensitivity_Function_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
[mag,phase,w] = bode(Grotor/(1 + Grotor*Krotor),  w);
mag = squeeze(mag);
semilogx(w, 20*log10(mag),'k');
xlim([0.1 100])
title("Load Disturbance Sensitivity Function");
ylabel("Load Disturbance Sensitivity Function Magnitude (dB)");
xlabel("Frequency (rad/sec)");
title("Load Disturbance Sensitivity Function");
fig_str = 'Load_Disturbance_Sensitivity_Function_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
[mag,phase,w] = bode(Krotor/(1 + Grotor*Krotor),  w);
mag = squeeze(mag);
semilogx(w, 20*log10(mag),'k');
xlim([0.1 100])
title("Noise Sensitivity Function");
ylabel("Noise Sensitivity Function Magnitude (dB)");
xlabel("Frequency (rad/sec)");
fig_str = 'Noise_Sensitivity_Function_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

step_increment = 0.001;
step_start = -1;
step_end = 10;
t = 0:step_increment:step_end;
[step_resp, t] = step(Grotor*Krotor/(1 + Grotor*Krotor),t);
S = stepinfo(step_resp,t);
t = step_start:step_increment:step_end - step_increment;

for m = 1:(step_end - step_start)/step_increment
    if t(m) < 0
        step_input(m) = 0;
        step_plot(m) = 0;
    else
        step_input(m) = 1;
        step_plot(m) = step_resp(m + step_start/step_increment);
    end
end

[diff,index] = min(abs(t-S.SettlingTime));
fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
fig = figure('units', 'normalized', 'Position',[startx starty endx endy]);
plot(t, step_plot,'k');
hold on
plot(S.PeakTime, S.Peak,'*g');
hold on
plot(S.SettlingTime, step_plot(index) ,'*r');
hold on
plot(t, step_input,'--k');
xlim([-1 10]);
title("Rotor Angle Control System Step Response");
legend("Rotor Angle Response", "First Peak", "Settling Time", "Rotor Angle Reference");
ylabel("Rotor Angle Amplitude");
xlabel("Sample Time (seconds)");
fig_str = 'Rotor_Angle_Step_Response_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
fig_save_filename = fullfile(selpath,fig_save_filename);
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);

disp(sprintf("Rotor Angle Step Time to Peak is %0.2f seconds at value of %0.2f with Overshoot of %0.2f Percent and Settling Time to Tolerance of 2 Percent of %0.2f seconds", S.PeakTime, S.Peak, S.Overshoot, S.SettlingTime));


function next_plot(hObject,callbackdata,handles)
global exit_state;
answer = questdlg('Enter Yes to Continue to Next Plot','Yes','Yes','Quit','Yes');
switch answer
    case 'Yes'
        close all;
    case 'Quit'
        exit_state = 1;
end
end



