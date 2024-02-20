%               Edukit_PID_Motor_Control_Design_Octave.m
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

pkg load control

set(0, "defaulttextfontsize", 12)  % title
set(0, "defaultaxesfontsize", 12)  % axes labels
set(0, "defaulttextfontname", "Helvita")
set(0, "defaultaxesfontname", "Helvitica")
set(0, "defaultlinelinewidth", 2)

global exit_state;
exit_state = 0;

function next_plot(hObject,callbackdata,handles)
  global exit_state;
  answer = questdlg('Enter Yes to Continue to Next Plot','Yes','Yes','Quit','Yes');
  switch answer
    case 'Yes'
      close all;
    case 'Quit'
      close all;
      exit_state = 1;
    end
  end
  
  function last_plot(hObject,callbackdata,handles)
    global exit_state;
    answer = questdlg('Enter Yes to Exit','Yes','Yes','Quit','Yes');
    switch answer
      case 'Yes'
        close all;
        exit_state = 1;
      case 'Quit'
        close all;
        exit_state = 1;
      end
    end
    
    
    startx = 0.6;
    starty = 0.5;
    endx = 0.39;
    endy = 0.5;
    
    ver = version;
    if compare_versions('6.1.0', ver, "<=") == 0
      disp(sprintf('This System Requires Octave Version 6.1.0 or greater'));
      mh = msgbox(sprintf('This System Requires Octave Version 6.1.0 or greater', 'replace'));
      return;
    end
    
    h=msgbox('Select Output Data Directory for Figures','Title');
    delete(findobj(h,'string','OK'));
    delete(findobj(h,'style','frame'));
    pause(2);
    close (h)
    
    fprintf('Select Output Data Directory for Figures');
    selpath = uigetdir;
    if isequal(selpath,0)
      disp('User selected Cancel');
    else
      disp(['User selected ', selpath]);
    end
    
    
% Rotor Plant Transfer Function from control
% input (degree units) to rotor position (degree units).
%

%
% Optional Plant Design Selection
%


plant_design_select = "";

plant_design_select = input('Enter "y" for Rotor Plant Design Grotor = 1/(s^2 + Wn*s) ','s');
if strcmp(plant_design_select, "y") == 1 || strcmp(plant_design_select, "Y") == 1
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

if strcmp(plant_design_select, "y") == 0 && strcmp(plant_design_select, "Y") == 0
    a = 1;
    b = 0;
    c = 0;
end

num_rotor = [0, 0, a];
den_rotor = [1, b, c];

Grotor = tf(num_rotor, den_rotor);

figure('units', 'normalized', 'Position',[startx starty endx endy]);
bode(Grotor)
title("Rotor Plant Open Loop Bode Plot")
next_plot; if exit_state == 1 return; end;

t = 0:0.001:10;
figure('units', 'normalized', 'Position',[startx starty endx endy]);
impulse(Grotor,t)
set(findall(gca,'Type','line'),'LineWidth',1)
title("Rotor Plant Open Loop Impulse Response")
next_plot; if exit_state == 1 return; end;

figure('units', 'normalized', 'Position',[startx starty endx endy]);
pzmap(Grotor)
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
    try
        user_entry = inputdlg(prompt,dlgtitle,dims,definput);
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
        try
            user_entry = inputdlg(prompt,dlgtitle,dims,definput);
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
    try
        user_entry = inputdlg(prompt,dlgtitle,dims,definput);
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
    bode(Grotor,Krotor,Krotor*Grotor)
    set(findall(gca,'Type','line'),'LineWidth',2)
    legend('Rotor Plant', 'Rotor Controller', 'Rotor Controller and Rotor Plant')
    title(sprintf('Motor Controller K = %0.2f Bode Plot', Kprotor));
    next_plot; if exit_state == 1 return; end;
    
    %
    % Grotor_cl is the closed loop transfer function for the rotor angle
    % feedback control system
    %
    
    
    Grotor_cl = feedback(Grotor*Krotor,1);
    
    figure('units', 'normalized', 'Position',[startx starty endx endy]);
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
    
        %
        % Rotor Angle Response Computation
        %
        
        % Step signal generation
        
        
        step_increment = 0.002;
        step_width = 0.004;
        step_start = -1;
        step_end = 10;
        t = step_start:step_increment:step_end;
        
        for i = 1:length(t)
          if t(i) < 0
            rotor_step(i) = 0;
          end
          if t(i) >= 0
            rotor_step(i) = 1;
          end
        end
        
        
        % Define settling time vector
        
        Sample_Period = step_increment;
        for i = 1:length(t)
          sample_settling_time(i) = (i - 1)*Sample_Period;
        end
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Input_Offset = 1;
        
        % Compute Rotor Step Settling Time
        
        Settling_Time_Tolerance_Value = 0.02;
        
        [rotor_angle_norm, t] = lsim(minreal(Grotor*Krotor/(1 + Grotor*Krotor)), rotor_step, t - step_start);
        t = t + step_start;
        
        Rotor_Angle_Avg_Peak = 0;
        for i = 1:length(t)
          if Rotor_Angle_Avg_Peak < rotor_angle_norm(i)
            Rotor_Angle_Avg_Peak = rotor_angle_norm(i);
            Rotor_Angle_Avg_Peak_Index = i;
          end
        end
        
        Rotor_Angle_Avg_Peak_Time = sample_settling_time(Rotor_Angle_Avg_Peak_Index - Input_Offset) + step_start;
        
        Settling_Time_Tolerance = Settling_Time_Tolerance_Value * Rotor_Angle_Avg_Peak;
        
        Input_Offset = 1;
        rotor_angle_norm_final_state = rotor_angle_norm(length(t));
        i = length(t);
        while i > Input_Offset
          if abs(rotor_angle_norm(i) - rotor_angle_norm_final_state) > Settling_Time_Tolerance
            Settling_Time = t(i - Input_Offset + 1);
            Settling_Time_Value = rotor_angle_norm(i);
            Settling_Time_Index = i;
            break;
          end
          i = i - 1;
        end
        
        Rotor_Angle_Overshoot = Rotor_Angle_Avg_Peak;
        
        disp(sprintf("%s Time to Peak is %0.2f seconds at value of %0.2e Rotor Step Overshoot %0.2e Settling Time to Tolerance of %0.1f Percent of %0.2f seconds", 'Rotor Angle', Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, Rotor_Angle_Overshoot, 100*Settling_Time_Tolerance_Value, Settling_Time));
        
        
        fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        plot(sample_settling_time + step_start, rotor_angle_norm,'k');
        hold on
        plot(sample_settling_time(Rotor_Angle_Avg_Peak_Index - 1) + step_start, Rotor_Angle_Avg_Peak,'*g');
        hold on
        plot(sample_settling_time(Settling_Time_Index - 1)  + step_start, Settling_Time_Value,'*r');
        xlim([-1 10]);
        xlabel('Sample Time (sec)');
        ylabel('Rotor Angle Response to Rotor Command Step');
        legend('Rotor Step Response', 'Rotor Step Peak', 'Rotor Settling Time', 'location', 'SouthEast');
        title_str = sprintf('Rotor Angle Response to Rotor Command Step');
        title(title_str);
        next_plot; if exit_state == 1 return; end;
    
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
        step(Grotor_cl,t)
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
        
        
       
        %
        % Load Disturbance Sensitivity Function Response Computation
        %
        
        
        % Step signal generation
        
        
        step_increment = 0.002;
        step_width = 0.004;
        step_start = -1;
        step_end = 10;
        t = step_start:step_increment:step_end;
        
        for i = 1:length(t)
          if t(i) < 0
            rotor_step(i) = 0;
          end
          if t(i) >= 0
            rotor_step(i) = 1;
          end
        end
        
        
        % Define settling time vector
        
        Sample_Period = step_increment;
        for i = 1:length(t)
          sample_settling_time(i) = (i - 1)*Sample_Period;
        end
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Settling_Time_Tolerance_Value = 0.02;
        
        [load_dist_norm, t] = lsim(minreal(Grotor/(1 + Grotor*Krotor)), rotor_step, t - step_start);
        t = t + step_start;
        
        Load_Disturbance_Sensitivity_Function_Peak = 0;
        for i = 1:length(t)
          if abs(Load_Disturbance_Sensitivity_Function_Peak) < abs(load_dist_norm(i))
            Load_Disturbance_Sensitivity_Function_Peak = load_dist_norm(i);
            Load_Disturbance_Sensitivity_Function_Peak_Index = i;
          end
        end
        
        Load_Disturbance_Step_Response_Peak_Time = sample_settling_time(Load_Disturbance_Sensitivity_Function_Peak_Index - Input_Offset) + step_start;
        
        Settling_Time_Tolerance = Settling_Time_Tolerance_Value * Load_Disturbance_Sensitivity_Function_Peak;
        
        Input_Offset = 1;
        load_dist_norm_final_state = load_dist_norm(length(t));
        i = length(t);
        while i > Input_Offset
          if abs(load_dist_norm(i) - load_dist_norm_final_state) > Settling_Time_Tolerance
            Settling_Time = t(i - Input_Offset + 1);
            Settling_Time_Value = load_dist_norm(i);
            Settling_Time_Index = i;
            break;
          end
          i = i - 1;
        end
        
        Load_Disturbance_Step_Response_Overshoot = Load_Disturbance_Sensitivity_Function_Peak;
        
        disp(sprintf("%s Time to Peak is %0.2f seconds at value of %0.2e Overshoot %0.2e Settling Time to Tolerance of %0.1f Percent of %0.2f seconds", 'Load Disturbance', Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, 100*Settling_Time_Tolerance_Value, Settling_Time));
        
        
        fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        plot(sample_settling_time + step_start, load_dist_norm,'k');
        hold on
        plot(sample_settling_time(Load_Disturbance_Sensitivity_Function_Peak_Index - 1) + step_start, Load_Disturbance_Sensitivity_Function_Peak,'*g');
        hold on
        plot(sample_settling_time(Settling_Time_Index - 1)  + step_start, Settling_Time_Value,'*r');
        xlim([-1 10]);
        xlabel('Sample Time (sec)');
        ylabel("Step Input Amplitude");
        xlim([-1 10]);      
        title("Load Disturbance Rejection Sensitivity Function Step Response");
        legend("Load Sensitivity Function Response", "First Peak", "Settling Time");
        xlabel("Sample Time (seconds)");
        legend('Load Disturbance Step Response', 'Load Disturbance Step Peak', 'Load Disturbance Settling Time', 'location', 'SouthEast');
        disp(sprintf("Load Disturbance Time to Peak is %0.2f seconds at value of %0.2f with Overshoot of %0.2f Percent and Settling Time to Tolerance of 2 Percent of %0.2f seconds", Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, Settling_Time));
        disp(sprintf("%0.2f\t%0.2f\t%0.2f\t%0.2f", Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, Settling_Time));
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
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;


        %
        % Load Disturbance Sensitivity Function Response Computation
        %
        
        
        % Step signal generation
        
        
        step_increment = 0.002;
        step_width = 0.004;
        step_start = -1;
        step_end = 10;
        t = step_start:step_increment:step_end;
        
        for i = 1:length(t)
          if t(i) < 0
            rotor_step(i) = 0;
          end
          if t(i) >= 0
            rotor_step(i) = 1;
          end
        end
        
        
        % Define settling time vector
        
        Sample_Period = step_increment;
        for i = 1:length(t)
          sample_settling_time(i) = (i - 1)*Sample_Period;
        end
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Settling_Time_Tolerance_Value = 0.02;
        
        [load_dist_norm, t] = lsim(minreal(Grotor/(1 + Grotor*Krotor)), rotor_step, t - step_start);
        t = t + step_start;
        
        Load_Disturbance_Sensitivity_Function_Peak = 0;
        for i = 1:length(t)
          if abs(Load_Disturbance_Sensitivity_Function_Peak) < abs(load_dist_norm(i))
            Load_Disturbance_Sensitivity_Function_Peak = load_dist_norm(i);
            Load_Disturbance_Sensitivity_Function_Peak_Index = i;
          end
        end
        
        Load_Disturbance_Step_Response_Peak_Time = sample_settling_time(Load_Disturbance_Sensitivity_Function_Peak_Index - Input_Offset) + step_start;
        
        Settling_Time_Tolerance = Settling_Time_Tolerance_Value * Load_Disturbance_Sensitivity_Function_Peak;
        
        Input_Offset = 1;
        load_dist_norm_final_state = load_dist_norm(length(t));
        i = length(t);
        while i > Input_Offset
          if abs(load_dist_norm(i) - load_dist_norm_final_state) > Settling_Time_Tolerance
            Settling_Time = t(i - Input_Offset + 1);
            Settling_Time_Value = load_dist_norm(i);
            Settling_Time_Index = i;
            break;
          end
          i = i - 1;
        end
        
        Load_Disturbance_Step_Response_Overshoot = Load_Disturbance_Sensitivity_Function_Peak;
        
        disp(sprintf("%s Time to Peak is %0.2f seconds at value of %0.2e Overshoot %0.2e Settling Time to Tolerance of %0.1f Percent of %0.2f seconds", 'Load Disturbance', Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, 100*Settling_Time_Tolerance_Value, Settling_Time));
        
        
        fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        plot(sample_settling_time + step_start, load_dist_norm,'k');
        hold on
        plot(sample_settling_time(Load_Disturbance_Sensitivity_Function_Peak_Index - 1) + step_start, Load_Disturbance_Sensitivity_Function_Peak,'*g');
        hold on
        plot(sample_settling_time(Settling_Time_Index - 1)  + step_start, Settling_Time_Value,'*r');
        xlim([-1 10]);
        xlabel('Sample Time (sec)');
        ylabel("Step Input Amplitude");
        xlim([-1 10]);      
        title("Load Disturbance Rejection Sensitivity Function Step Response");
        legend("Load Sensitivity Function Response", "First Peak", "Settling Time");
        xlabel("Sample Time (seconds)");
        legend('Load Disturbance Step Response', 'Load Disturbance Step Peak', 'Load Disturbance Settling Time', 'location', 'SouthEast');
        disp(sprintf("Load Disturbance Time to Peak is %0.2f seconds at value of %0.2f with Overshoot of %0.2f Percent and Settling Time to Tolerance of 2 Percent of %0.2f seconds", Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, Settling_Time));
        disp(sprintf("%0.2f\t%0.2f\t%0.2f\t%0.2f", Load_Disturbance_Step_Response_Peak_Time, Load_Disturbance_Sensitivity_Function_Peak, Load_Disturbance_Step_Response_Overshoot, Settling_Time));
 

 fig_str = 'Load_Disturbance_Sensitivity_Step_Response_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
print(fig_display_step,fig_save_filename,'-dpng')
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
print(fig,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);
close;

        %
        % Rotor Angle Response Computation
        %
        
        % Step signal generation
        
        
        step_increment = 0.002;
        step_width = 0.004;
        step_start = -1;
        step_end = 10;
        t = step_start:step_increment:step_end;
        
        for i = 1:length(t)
          if t(i) < 0
            rotor_step(i) = 0;
          end
          if t(i) >= 0
            rotor_step(i) = 1;
          end
        end
        
        
        % Define settling time vector
        
        Sample_Period = step_increment;
        for i = 1:length(t)
          sample_settling_time(i) = (i - 1)*Sample_Period;
        end
        
        Input_Offset = 1;
        
        % Compute Step Settling Time
        
        Input_Offset = 1;
        
        % Compute Rotor Step Settling Time
        
        Settling_Time_Tolerance_Value = 0.02;
        
        [rotor_angle_norm, t] = lsim(minreal(Grotor*Krotor/(1 + Grotor*Krotor)), rotor_step, t - step_start);
        t = t + step_start;
        
        Rotor_Angle_Avg_Peak = 0;
        for i = 1:length(t)
          if Rotor_Angle_Avg_Peak < rotor_angle_norm(i)
            Rotor_Angle_Avg_Peak = rotor_angle_norm(i);
            Rotor_Angle_Avg_Peak_Index = i;
          end
        end
        
        Rotor_Angle_Avg_Peak_Time = sample_settling_time(Rotor_Angle_Avg_Peak_Index - Input_Offset) + step_start;
        
        Settling_Time_Tolerance = Settling_Time_Tolerance_Value * Rotor_Angle_Avg_Peak;
        
        Input_Offset = 1;
        rotor_angle_norm_final_state = rotor_angle_norm(length(t));
        i = length(t);
        while i > Input_Offset
          if abs(rotor_angle_norm(i) - rotor_angle_norm_final_state) > Settling_Time_Tolerance
            Settling_Time = t(i - Input_Offset + 1);
            Settling_Time_Value = rotor_angle_norm(i);
            Settling_Time_Index = i;
            break;
          end
          i = i - 1;
        end
        
        Rotor_Angle_Overshoot = Rotor_Angle_Avg_Peak;
        
        disp(sprintf("%s Time to Peak is %0.2f seconds at value of %0.2e Rotor Step Overshoot %0.2e Settling Time to Tolerance of %0.1f Percent of %0.2f seconds", 'Rotor Angle', Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, Rotor_Angle_Overshoot, 100*Settling_Time_Tolerance_Value, Settling_Time));
        
        
        fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
        plot(sample_settling_time + step_start, rotor_angle_norm,'k');
        hold on
        plot(sample_settling_time(Rotor_Angle_Avg_Peak_Index - 1) + step_start, Rotor_Angle_Avg_Peak,'*g');
        hold on
        plot(sample_settling_time(Settling_Time_Index - 1)  + step_start, Settling_Time_Value,'*r');
        xlim([-1 10]);
        xlabel('Sample Time (sec)');
        ylabel('Rotor Angle Response to Rotor Command Step');
        legend('Rotor Step Response', 'Rotor Step Peak', 'Rotor Settling Time', 'location', 'SouthEast');
        title_str = sprintf('Rotor Angle Response to Rotor Command Step');
        title(title_str);
fig_str = 'Rotor_Angle_Step_Response_Fig';
fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
print(fig_display_step,fig_save_filename,'-dpng')
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);

