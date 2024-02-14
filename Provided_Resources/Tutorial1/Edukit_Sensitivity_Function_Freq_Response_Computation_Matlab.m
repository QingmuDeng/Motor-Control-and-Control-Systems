%
%                        STMicroelectronics
%
%            Edukit Sensitivity Function Frequency Computation
%
%                         Matlab Version 2.0
%                         April 11, 2021
%
% ************************* Description *****************************
%
% Compute Sensitivity Function Frequency Response from Edukit

% time series data.
%
%
%               For Additional Information Please See
% Please see: https://sites.google.com/view/ucla-st-motor-control/home
%
%***************************** Usage ********************************
%
%
% The Edukit system includes the capability for introduction of a
% step signal stimulus for probing of the control system response
% for each of the four Sensitivity Functions.

% Frequency response for each Sensitivity Function may then be
% determined by computing the ratio of the Fourier Transform of
% both the system response and the system stimulus for each
% frequency component.
%
%
% Dataset must include a time series including cycles with
% step signal period of 16384 samples
%
% A total of 40,000 samples are required to ensure that a sample
% count of 32,768 with margin is available.
%
% Sample aquisition is performed by the Edukit Real Time Control
% System Workbench with its Data Log feature
%
% The Workbench stores *.mat format files with filename indicating
% the Sensitivity Function measurement
%
% The Sensitivity Function data file includes the
% identifier Sensitivity_Function__<Date>_<Time>.
%
% The Noise Sensitivity Function data file includes the
% identifier Noise_Sensitivity_Function__<Date>_<Time>.
%
% The Load Disturbance Sensitivity Function data file includes the
% identifier Disturbance_Sensitivity_Function__<Date>_<Time>.
%
% The Complimentary Sensitivity Function data file includes the
% identifier Step_Respone_Rotor_Angle_(degrees)__<Date>_<Time>.
%
% Acquisition procedure for Sensitivity Function data requires
% the selection of at least three measurement sweeps by the
% Workbench system.
%
% If a large number of measurement sweeps is selected, then
% signal averaging is applied by this system to improve signal to
% noise ratio of measured frequency response
%
%

clear all
close all

% Figure position parameters

startx = 0.6;
starty = 0.5;
endx = 0.39;
endy = 0.5;

global exit_state;
exit_state = 0;

fprintf('Select Input Data with Sensitivity Function Data\r\n');

[file,path] = uigetfile('*.mat');
if isequal(file,0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path,file)]);
end
fname = strcat(path,file);
load(fname)

fprintf('Select Output Data Directory for Sensitivity Function Figure\r\n');
selpath = uigetdir;
if isequal(selpath,0)
    disp('User selected Cancel');
else
    disp(['User selected ', selpath]);
end


disp(sprintf(" Measurement: %s", data_display));
disp(sprintf(" Kpend"));
disp(sprintf(" Kp\tKi\tKd"));
disp(sprintf("%0.2f\t%0.2f\t%0.2f", pend_p, pend_i,pend_d));
disp(sprintf(" Krotor"));
disp(sprintf(" Kp\tKi\tKd"));
disp(sprintf("%0.2f\t%0.2f\t%0.2f", rotor_p,rotor_i,rotor_d));

if length(rotor_command) < 32
    disp('Insufficient samples available: Require Two Measurement Sweeps of 40 Seconds');
    return;
end

NFFT = 32768;
Input_Offset = 1000;
% Selecting 1 permits viewing of each sweep during signal averaging
Display_Cycles = 0;

% Define interval for integration of control effort total energy

Total_Control_Effort_Integration_Interval = 5;

% Define settling limit for decay of Control Effort after impulse

Control_Effort_Settling_Limit = 50;


% Start signal acquisition and averaging

cycle_count = 1;
j = 1;
i = 1;

i = Input_Offset;
while i < length(rotor_command)
    if rotor_command(i - 1) < 0 && rotor_command(i) > rotor_command(i - 1)
        sample_start(j) = i - 1;
        sample_end(j) = sample_start(j) + NFFT - 1;
        i = i + NFFT/2;
        if sample_end(j) > length(rotor_command)
            break
        end
        for k = sample_start(j) - Input_Offset:sample_end(j) - Input_Offset
            rotor_command_sum(j,k-sample_start(j) + Input_Offset + 1) = rotor_command(k);
            rotor_angle_sum(j,k-sample_start(j) + Input_Offset + 1) = rotor_angle(k);
            pendulum_angle_sum(j,k-sample_start(j) + Input_Offset + 1) = pendulum_angle(k);
            control_effort_sum(j,k-sample_start(j) + Input_Offset + 1) = motor_position_target(k);
        end
        if Display_Cycles == 1
            figure('units', 'normalized', 'Position',[startx starty endx endy]);
            plot(rotor_angle(sample_start(j):sample_end(j)));
            hold on
            plot(rotor_command(sample_start(j):sample_end(j)));
            xlabel('Sample Time (seconds)')
            ylabel('Rotor Angle (degrees)');
            legend('Rotor Angle', 'Rotor Track Signal')
            next_plot; if exit_state == 1 return; end;
            
            figure('units', 'normalized', 'Position',[startx starty endx endy]);
            yyaxis left;
            plot(pendulum_angle(sample_start(j):sample_end(j)));
            xlabel('Sample Time (seconds)')
            ylabel('Pendulum Angle (degrees)');
            hold on
            plot(rotor_angle(sample_start(j):sample_end(j)));
            yyaxis right;
            xlabel('Sample Time (seconds)')
            ylabel('Rotor Angle (degrees)');
            legend('Pendulum Angle', 'Rotor Angle')
            next_plot; if exit_state == 1 return; end;
            
            figure('units', 'normalized', 'Position',[startx starty endx endy]);
            yyaxis left;
            plot(motor_position_target(sample_start(j):sample_end(j)));
            xlabel('Sample Time (seconds)')
            ylabel('Pendulum Angle (degrees)');
            hold on
            yyaxis right;
            plot(rotor_command(sample_start(j):sample_end(j)));
            xlabel('Sample Time (seconds)')
            ylabel('Rotor Angle (degrees)');
            legend('Control Signal', 'Rotor Angle')
            next_plot; if exit_state == 1 return; end;
            
        end
        j = j + 1;
    end
    i = i + 1;
end

cycle_count = j - 1;

for i = 1:cycle_count
    if i == 1
        rotor_command_avg = rotor_command_sum(i,:);
        rotor_angle_avg = rotor_angle_sum(i,:);
        pendulum_angle_avg = pendulum_angle_sum(i,:);
        control_effort_avg = control_effort_sum(i,:);
    end
    if i > 1
        rotor_command_avg = rotor_command_avg + rotor_command_sum(i,:);
        rotor_angle_avg = rotor_angle_avg + rotor_angle_sum(i,:);
        pendulum_angle_avg = pendulum_angle_avg + pendulum_angle_sum(i,:);
        control_effort_avg = control_effort_avg + control_effort_sum(i,:);
    end
end

rotor_command_avg = rotor_command_avg/cycle_count;
rotor_angle_avg = rotor_angle_avg/cycle_count;
pendulum_angle_avg = pendulum_angle_avg/cycle_count;
control_effort_avg = control_effort_avg/cycle_count;

start_index = 1;

rotor_command_avg = rotor_command_avg(start_index:NFFT + start_index - 1 );
rotor_angle_avg = rotor_angle_avg(start_index:NFFT + start_index - 1);
pendulum_angle_avg = pendulum_angle_avg(start_index:NFFT + start_index - 1);
control_effort_avg = control_effort_avg(start_index:NFFT + start_index - 1);

if rotor_angle_avg(1) > 0
    rotor_command_avg = -rotor_command_avg;
    rotor_angle_avg = -rotor_angle_avg;
    pendulum_angle_avg = -pendulum_angle_avg;
    control_effort_avg = -control_effort_avg;
end

% Compute time vector

for i = 1:NFFT
    cycle_sample_time(i) = (i - 1)*Sample_Period;
end

figure('units', 'normalized', 'Position',[startx starty endx endy]);
plot(cycle_sample_time, rotor_command_avg);
hold on
plot(cycle_sample_time, rotor_angle_avg);
xlabel('Sample Time (seconds)');
ylabel('Rotor Angle (degrees)');
legend('Rotor Tracking Command', 'Rotor Angle');
title_str = sprintf('%s Step Response',data_display);
title(title_str);

next_plot; if exit_state == 1 return; end;
figure('units', 'normalized', 'Position',[startx starty endx endy]);
plot(cycle_sample_time, pendulum_angle_avg);
xlabel('Sample Time (seconds)');
ylabel('Pendulum Angle (degrees)');
title_str = sprintf('Pendulum Angle Control System Step Response');
title(title_str);
next_plot; if exit_state == 1 return; end;


% Compute Peak Time, Overshoot, and Settling Time

NFFT = 32768;
Final_State_Interval_Start = round(0.8 * (NFFT)/2);
Final_State_Interval_End = round(0.95 * (NFFT)/2);

Initial_State_Amplitude_Average_Rotor = mean(rotor_angle_avg(1:Input_Offset));
Initial_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(1:Input_Offset));
Final_State_Amplitude_Average_Rotor = mean(rotor_angle_avg(Final_State_Interval_Start:Final_State_Interval_End));
Final_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(Final_State_Interval_Start:Final_State_Interval_End));

Amplitude_Norm_Rotor =  abs(Initial_State_Amplitude_Average_Rotor_Command - Final_State_Amplitude_Average_Rotor_Command);
rotor_angle_norm = ((rotor_angle_avg - Initial_State_Amplitude_Average_Rotor)/Amplitude_Norm_Rotor);
rotor_angle_norm_final_state = mean(rotor_angle_norm(Final_State_Interval_Start:Final_State_Interval_End));

Initial_State_Amplitude_Average_Pendulum = mean(pendulum_angle_avg(1:Input_Offset));
Initial_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(1:Input_Offset));
Final_State_Amplitude_Average_Pendulum = mean(pendulum_angle_avg(Final_State_Interval_Start:Final_State_Interval_End));
Final_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(Final_State_Interval_Start:Final_State_Interval_End));

Amplitude_Norm_Pendulum =  abs(Initial_State_Amplitude_Average_Rotor_Command - Final_State_Amplitude_Average_Rotor_Command);
pendulum_angle_norm = -((pendulum_angle_avg - Initial_State_Amplitude_Average_Pendulum)/Amplitude_Norm_Rotor);
pendulum_angle_norm_final_state = -mean(pendulum_angle_norm(Final_State_Interval_Start:Final_State_Interval_End));


for i = 1:NFFT/2
    sample_settling_time(i) = (i - 1)*Sample_Period;
end

if data_display == "Disturbance Sensitivity Function" && pend_p > 0
    rotor_angle_norm = -rotor_angle_norm;
end

Rotor_Angle_Avg_Peak = 0;
for i = 1:NFFT/2
    if Rotor_Angle_Avg_Peak < abs(rotor_angle_norm(i)) && data_display == "Noise Sensitivity Function"
        Rotor_Angle_Avg_Peak = abs(rotor_angle_norm(i));
        Rotor_Angle_Avg_Peak_Index = i;
    end
    if Rotor_Angle_Avg_Peak < rotor_angle_norm(i) && rotor_p < 0 && data_display ~= "Noise Sensitivity Function"
        Rotor_Angle_Avg_Peak = rotor_angle_norm(i);
        Rotor_Angle_Avg_Peak_Index = i;
    end
    if Rotor_Angle_Avg_Peak < rotor_angle_norm(i) && rotor_p > 0 && data_display ~= "Noise Sensitivity Function"
        Rotor_Angle_Avg_Peak = rotor_angle_norm(i);
        Rotor_Angle_Avg_Peak_Index = i;
    end
end

Rotor_Angle_Avg_Peak = rotor_angle_norm(Rotor_Angle_Avg_Peak_Index);

Rotor_Angle_Avg_Peak_Time = sample_settling_time(Rotor_Angle_Avg_Peak_Index - Input_Offset);

Settling_Time_Tolerance = 0.02;

if data_display == "Step Response Rotor Angle (degrees)"
    Settling_Time_Tolerance = 0.02;
end

if data_display == "Disturbance Sensitivity Function" && pend_p < 0
    Settling_Time_Tolerance = 0.02;
end

if data_display == "Disturbance Sensitivity Function" && pend_p > 0
    Settling_Time_Tolerance = 0.1;
end

if data_display == "Noise Sensitivity Function" && pend_p > 0
    Settling_Time_Tolerance = 0.1;
end

i = Final_State_Interval_End;
while i > Input_Offset
    if abs(rotor_angle_norm(i) - rotor_angle_norm_final_state) > abs(Settling_Time_Tolerance * Rotor_Angle_Avg_Peak)
        Settling_Time = sample_time(i - Input_Offset);
        Settling_Time_Value = rotor_angle_norm(i);
        Settling_Time_Index = i;
        break;
    end
    i = i - 1;
end

Overshoot = 100*(Rotor_Angle_Avg_Peak - rotor_angle_norm_final_state)/rotor_angle_norm_final_state;

if data_display == "Step Response Rotor Angle (degrees)"
    disp(sprintf("%s Time to Peak is %0.1f seconds at value of %0.2f with Overshoot of %0.1f Percent and Settling Time to Tolerance of %0.0f Percent of %0.1f seconds", data_display, Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, Overshoot, 100*Settling_Time_Tolerance, Settling_Time));
    disp(sprintf("Final State Value is %0.2f", rotor_angle_norm_final_state));
    disp(sprintf("%0.1f\t%0.2f\t%0.1f\t%0.0f\t%0.1f\t%0.2f",Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, Overshoot, 100*Settling_Time_Tolerance, Settling_Time, rotor_angle_norm_final_state));

end

if data_display ~= "Step Response Rotor Angle (degrees)"
    disp(sprintf("%s Time to Peak is %0.1f seconds at value of %0.2f and Settling Time to Tolerance of %0.0f Percent of %0.1f seconds", data_display, Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, 100*Settling_Time_Tolerance, Settling_Time));
    disp(sprintf("Final State Value is %0.2f", rotor_angle_norm_final_state));
    disp(sprintf("%0.1f\t%0.2f\t%0.1f\t%0.1f\t%0.2f",Rotor_Angle_Avg_Peak_Time, Rotor_Angle_Avg_Peak, 100*Settling_Time_Tolerance, Settling_Time, rotor_angle_norm_final_state));
end

fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
plot(sample_settling_time - sample_settling_time(Input_Offset), rotor_angle_norm(1:1 + NFFT/2 - 1),'k');
hold on
plot(sample_settling_time(Rotor_Angle_Avg_Peak_Index - 1) - sample_settling_time(Input_Offset), Rotor_Angle_Avg_Peak,'*g');
hold on
plot(sample_settling_time(Settling_Time_Index - 1) - sample_settling_time(Input_Offset), Settling_Time_Value,'*r');
xlabel("Sample Time (seconds)");
ylabel("Rotor Angle Response");
xlim([-1 10]);
legend("Rotor Angle Response", "Peak Time", "Settling Time");

if data_display == "Disturbance Sensitivity Function"
    ylabel("Disturbance Rejection Sensitivity Function Step Response");
    fig_str_step = "Disturbance Rejection Sensitivity Function Step Response";
    title_str = sprintf('Disturbance Rejection Sensitivity Function Step Response');
    title(title_str);
    
end
if data_display == "Sensitivity Function"
    ylabel("Sensitivity Function Step Response");
    fig_str_step = "Sensitivity Function Step Response";
    title_str = sprintf('Sensitivity Function Step Response');
    title(title_str);
end
if data_display == "Step Response Rotor Angle (degrees)"
    ylabel("Rotor Angle Step Response");
    fig_str_step = "Rotor Angle Step Response";
    title_str = sprintf('Rotor Angle Step Response');
    title(title_str);
end
if data_display == "Noise Sensitivity Function"
    ylabel("Noise Sensitivity Function Step Response");
    fig_str_step = "Noise Sensitivity Function Step Response";
    title_str = sprintf('Noise Sensitivity Function Step Response');
    title(title_str);
end


fig_save_filename = sprintf('%s_%s.png', fig_str_step, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
print(fig_display_step,save_filename,'-dpng')
saveas(gcf, save_filename);
fig_save_filename = sprintf('%s_%s.fig', fig_str_step, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);


% Compute Maximum values of Control Effort and Control Effort Energy
% for data acquired from Rotor Angle Step


if data_display == "Step Response Rotor Angle (degrees)"
    
    Input_Offset = 1000;
    
    Final_State_Interval_Start = round(Input_Offset + 0.8 * (NFFT)/2);
    Final_State_Interval_End = round(Input_Offset + 0.95 * (NFFT)/2);
    
    Initial_State_Amplitude_Average_Control_Effort = mean(control_effort_avg(1:Input_Offset));
    Initial_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(1:Input_Offset-2));
    Final_State_Amplitude_Average_Control_Effort = mean(control_effort_avg(Final_State_Interval_Start:Final_State_Interval_End));
    Final_State_Amplitude_Average_Rotor_Command = mean(rotor_command_avg(Final_State_Interval_Start:Final_State_Interval_End));
    
    
    Amplitude_Rotor_Command = max(rotor_command_avg) - min(rotor_command_avg);
    
    control_effort_norm = ((control_effort_avg - Final_State_Amplitude_Average_Control_Effort)/Amplitude_Rotor_Command);
    control_effort_norm_final_state = -mean(control_effort_norm(Final_State_Interval_Start:Final_State_Interval_End));
    
    for i = 1:NFFT
        sample_settling_time(i) = (i - 1)*Sample_Period;
    end
    
    Control_Effort_Avg_Peak = 0;
    for i = 1:NFFT
        if Control_Effort_Avg_Peak < control_effort_norm(i) && rotor_p < 0
            Control_Effort_Avg_Peak = control_effort_norm(i);
            Control_Effort_Avg_Peak_Index = i;
            Control_Effort_Max_Time = sample_settling_time(i);
        end
        if Control_Effort_Avg_Peak > control_effort_norm(i) && rotor_p >= 0
            Control_Effort_Avg_Peak = control_effort_norm(i);
            Control_Effort_Avg_Peak_Index = i;
            Control_Effort_Max_Time = sample_settling_time(i);
        end
    end
    
    Control_Effort_Avg_Peak_Time = sample_settling_time(1 + Control_Effort_Avg_Peak_Index - Input_Offset);
    
    Control_Effort_Power_Peak = Control_Effort_Avg_Peak^2;
    
    Control_Effort_Total= 0;
    
    i = 1;
    while i < Final_State_Interval_End
        Control_Effort_Total= Control_Effort_Total+ Sample_Period*abs(control_effort_norm(i) - control_effort_norm_final_state);
        if abs(control_effort_norm(i)) < abs(Control_Effort_Avg_Peak)/Control_Effort_Settling_Limit && sample_settling_time(i) > sample_settling_time(Control_Effort_Avg_Peak_Index) + Total_Control_Effort_Integration_Interval
            break
        end
        i = i + 1;
    end
    
    
    disp(sprintf("Control Effort Max: %0.2f; Control Effort Max Power: %0.2e at time of %0.2f seconds with Control Effort Total of %0.2f", Control_Effort_Avg_Peak, Control_Effort_Power_Peak, Control_Effort_Avg_Peak_Time, Control_Effort_Total));
    
    fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
    plot(sample_settling_time - sample_settling_time(Input_Offset), control_effort_norm(1:NFFT),'k');
    hold on
    plot(sample_settling_time(Control_Effort_Avg_Peak_Index) - sample_settling_time(Input_Offset), Control_Effort_Avg_Peak,'*g');
    xlim([-0.1 0.4]);
    xlabel('Sample Time (sec)');
    ylabel('Control Effort Response');
    title_str = sprintf('Control Effort Response to Rotor Angle Reference Step');
    title(title_str);
    legend("Control Effort Step Response", "Peak Time");
    
    fig_str_impulse_control_effort = "Control Effort Response to Rotor Angle Reference Step";
    fig_save_filename = sprintf('%s_%s.png', fig_str_impulse_control_effort, datestr(now,'_mm-dd-yyyy_HH-MM'));
    save_filename = fullfile(selpath,fig_save_filename);
    print(fig_display_step,save_filename,'-dpng')
    saveas(gcf, save_filename);
    fig_save_filename = sprintf('%s_%s.fig', fig_str_impulse_control_effort, datestr(now,'_mm-dd-yyyy_HH-MM'));
    save_filename = fullfile(selpath,fig_save_filename);
    savefig(gcf, save_filename);
    
end


% Compute Pendulum Response to Disturbance Signal


for i = 1:NFFT
    sample_settling_time(i) = (i - 1)*Sample_Period;
end

Pendulum_Angle_Avg_Peak = 0;
for i = 1:NFFT/2
    if Pendulum_Angle_Avg_Peak < pendulum_angle_norm(i)
        Pendulum_Angle_Avg_Peak = pendulum_angle_norm(i);
        Pendulum_Angle_Avg_Peak_Index = i;
    end
end

Pendulum_Angle_Avg_Peak_Time = sample_settling_time(Pendulum_Angle_Avg_Peak_Index - Input_Offset);

Settling_Time_Tolerance = 0.1;

if data_display == "Disturbance Sensitivity Function"
    Settling_Time_Tolerance = 0.2;
end

if data_display == "Noise Sensitivity Function"
    Settling_Time_Tolerance = 0.2;
end

i = Final_State_Interval_End;
while i > Input_Offset
    if abs(pendulum_angle_norm(i) - pendulum_angle_norm_final_state) > abs(Settling_Time_Tolerance * Pendulum_Angle_Avg_Peak)
        Settling_Time = sample_time(i - Input_Offset);
        Settling_Time_Index = i;
        break;
    end
    i = i - 1;
end

Settling_Time_Value = pendulum_angle_norm(Settling_Time_Index);

disp(sprintf("%s Time to Peak is %0.2f seconds at value of %0.2e Settling Time to Tolerance of %0.1f Percent of %0.2f seconds", 'Pendulum Angle', Pendulum_Angle_Avg_Peak_Time, Pendulum_Angle_Avg_Peak, 100*Settling_Time_Tolerance, Settling_Time));
disp(sprintf("%0.1f\t%0.2f\t%0.1f\t%0.0f\t%0.1f",Pendulum_Angle_Avg_Peak_Time, Pendulum_Angle_Avg_Peak, 100*Settling_Time_Tolerance, Settling_Time));

fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
plot(sample_settling_time - sample_settling_time(Input_Offset), pendulum_angle_norm,'k');
hold on
plot(sample_settling_time(Pendulum_Angle_Avg_Peak_Index) - sample_settling_time(Input_Offset), Pendulum_Angle_Avg_Peak,'*g');
hold on
plot(sample_settling_time(Settling_Time_Index) - sample_settling_time(Input_Offset), Settling_Time_Value,'*r');
xlim([-1 10]);
xlabel('Sample Time (sec)');
ylabel('Pendulum Angle Response');
title_str = sprintf('Pendulum Angle Response');
title(title_str);
legend('Pendulum Response', 'Peak Time', 'Settling Time');

fig_str_step_pendulum = "Pendulum Angle Response to Rotor Step";
fig_save_filename = sprintf('%s_%s.png', fig_str_step_pendulum, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
print(fig_display_step,save_filename,'-dpng')
saveas(gcf, save_filename);
fig_save_filename = sprintf('%s_%s.fig', fig_str_step_pendulum, datestr(now,'_mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);

% Compute Fourier Transforms of stimulus and response

reference_input = rotor_command_avg;
resp = rotor_angle_avg;
NFFT = 32768;
y = fft(resp,NFFT);
yi = fft(reference_input,NFFT);
Sample_Period = (sample_time(NFFT) - sample_time(1))/NFFT;
Fs = (1/Sample_Period);
F = ((0:1/NFFT:1-1/NFFT)*(Fs)).';
Wm = F*2*pi;

frequency_bin_count = 1;

%
% Remove even frequency bins where Fourier Transform of step signal
% displays zero value
%

for m = 1:NFFT
    if mod(m+1,2) == 0
        continue
    end
    Mag(frequency_bin_count) = abs(y(m)/yi(m));
    W(frequency_bin_count) = Wm(m);
    frequency_bin_count = frequency_bin_count + 1;
end

%
% Reduce spectrum frequency bin resolution at high frequency to
% enhance signal to noise at high frequency above W_threshold
%

W_threshold = 2;
[W_value, idx] = min(abs(W - W_threshold));
group_count = [8];
count_threshold = [idx + group_count];

for n = 1:length(group_count)
    i = 1;
    j = 1;
    while i < (length(W))
        if i < count_threshold(n)
            Wavg(j) = W(i);
            Mag_avg(j) = Mag(i);
            i = i + 1;
            j = j + 1;
            continue
        end
        if i >= count_threshold(n)
            Wavg(j) = 0;
            Mag_avg(j) = 0;
            if i + group_count(n) > length(W)
                break
            end
            k = 0;
            while k < group_count(n)
                Wavg(j) = Wavg(j) + W(i + k);
                Mag_avg(j) = Mag_avg(j) + Mag(i + k);
                k = k + 1;
            end
            Wavg(j) = Wavg(j)/group_count(n);
            Mag_avg(j) = Mag_avg(j)/group_count(n);
            j = j + 1;
            i = i + group_count(n);
        end
    end
    
    W = Wavg;
    Mag = Mag_avg;
    
    clear Wavg; clear Mag_avg;
    
end

% Display data set

disp(sprintf(" Number of Measurement Cycles: %i", cycle_count));

if data_display == "Step Response Rotor Angle (degrees)"
    sensitivity_str = "Complimentary Sensitivity Function";
end
if data_display ~= "Step Response Rotor Angle (degrees)"
    sensitivity_str = data_display;
end

% Determine maxium value of Sensitivity Function for frequency less than
% 100 rad/sec


W_Max = 100;
[val, index] = min(abs(W - W_Max));

disp(sprintf("%s Maximum Value = %0.2f dB", sensitivity_str, max(20*log10(Mag(1:index)))));

% Start plot generation

Plot_Length = length(W);
Plot_Start = 1;

figure('units', 'normalized', 'Position',[startx starty endx endy]);
semilogx(W(Plot_Start:Plot_Length),20*log10(Mag(Plot_Start:Plot_Length)),'*')
if data_display == "Noise Sensitivity Function"
    xlim([0.1 1000])
end
if data_display ~= "Noise Sensitivity Function"
    xlim([0.1 100])
end
xlabel('Frequency (rad/sec)');
ylabel('Sensitivity Function (dB)');
yl = ylim;
y_lim_lower = yl(1);
y_lim_upper = yl(2);

while(1)
    h=questdlg('Enter Yes to Accept Plot Limit, No to Enter New Limits (Upper Greater than Lower', '');
    switch h
        case 'Yes'
        case 'No'
            close all;
            prompt = {'Enter Upper Limit (dB))'};
            dlgtitle = 'Input';
            dims = [1 40];
            definput = {' '};
            y_lim_upper_in  = inputdlg(prompt,dlgtitle,dims,definput);
            y_lim_upper = str2num(cell2mat(y_lim_upper_in (1)));
            prompt = {'Enter Lower Limit (dB))'};
            dlgtitle = 'Input';
            dims = [1 40];
            definput = {' '};
            y_lim_lower_in  = inputdlg(prompt,dlgtitle,dims,definput);
            y_lim_lower = str2num(cell2mat(y_lim_lower_in (1)));
    end;
    
    if y_lim_upper > y_lim_lower
        break;
    end
    
    if y_lim_upper <= y_lim_lower
        continue;
    end
    %
    % Select extended frequency range for noise sensitivity function
    %
    fig_sens_fncn = figure('units', 'normalized', 'Position',[startx starty endx endy]);
    semilogx(W(Plot_Start:Plot_Length),20*log10(Mag(Plot_Start:Plot_Length)),'*k')
    if data_display == "Noise Sensitivity Function"
        xlim([0.1 1000])
        xlabel('Frequency (rad/sec)');
        ylabel('Noise Rejection Sensitivity Function (dB)');
    end
    if data_display ~= "Noise Sensitivity Function"
        xlim([0.1 100])
        xlabel('Frequency (rad/sec)');
        ylabel('Sensitivity Function (dB)');
    end
    ylim([y_lim_lower y_lim_upper]);
end

figure('units', 'normalized', 'Position',[startx starty endx endy]);
semilogx(W(Plot_Start:Plot_Length),20*log10(Mag(Plot_Start:Plot_Length)),'*k')
if data_display == "Noise Sensitivity Function"
    xlim([0.1 1000]);
    xlabel('Frequency (rad/sec)');
    ylabel('Noise Rejection Sensitivity Function (dB)');
end
if data_display ~= "Noise Sensitivity Function"
    xlim([0.1 100]);
end
ylim([y_lim_lower y_lim_upper]);
xlabel('Frequency (rad/sec)');
ylabel('Sensitivity Function (dB)');

reference_amplitude = max(reference_input) - min(reference_input);

if data_display == "Step Response Rotor Angle (degrees)"
    
    fig_display_step = figure('units', 'normalized', 'Position',[startx starty endx endy]);
    if rotor_p >= 0
        plot(cycle_sample_time - cycle_sample_time(Input_Offset), reference_input, '--k');
        hold on;
        plot(cycle_sample_time  - cycle_sample_time(Input_Offset), resp, 'k');
    end
    if rotor_p < 0
        plot(cycle_sample_time - cycle_sample_time(Input_Offset), -reference_input, '--k');
        hold on;
        plot(cycle_sample_time  - cycle_sample_time(Input_Offset), -resp, 'k');
    end
    xlabel('Sample Time (sec)');
    ylabel('Rotor Angle (degrees)');
    legend('Rotor Reference Input','Rotor Response');
    title_str = sprintf('Rotor Angle Control System Step Response');
    title(title_str);
    
    fig_str_step = "Rotor_Angle_Step_Response_Degrees";
    fig_save_filename = sprintf('%s_%s.png', fig_str_step, datestr(now,'_mm-dd-yyyy_HH-MM'));
    save_filename = fullfile(selpath,fig_save_filename);
    print(fig_display_step,save_filename,'-dpng')
    saveas(gcf, save_filename);
    fig_save_filename = sprintf('%s_%s.fig', fig_str_step, datestr(now,'_mm-dd-yyyy_HH-MM'));
    save_filename = fullfile(selpath,fig_save_filename);
    savefig(gcf, save_filename);
    
end

fig_sensitivity = figure('units', 'normalized', 'Position',[startx starty endx endy]);
semilogx(W(Plot_Start:Plot_Length),20*log10(Mag(Plot_Start:Plot_Length)),'*k')

if data_display == "Sensitivity Function"
    xlim([ 0.1 100 ]);
    ylim([y_lim_lower y_lim_upper]);
    ylabel('Sensitivity Function (dB)');
    xlabel('Frequency (rad/sec)');
    fig_str = 'Sensitivity_Function_Fig';
    title_str = sprintf('Sensitivity Function');
    title(title_str);
end

if data_display == "Noise Sensitivity Function"
    xlim([ 0.1 1000 ]);
    ylim([y_lim_lower y_lim_upper]);
    ylabel('Noise Rejection Sensitivity Function (dB)');
    xlabel('Frequency (rad/sec)');
    fig_str = 'Noise_Sensitivity_Function_Fig';
    title_str = sprintf('Noise Sensitivity Function');
    title(title_str);
end

if data_display == "Step Response Rotor Angle (degrees)"
    xlim([ 0.1 100 ]);
    ylim([y_lim_lower y_lim_upper]);
    ylabel('Complimentary Sensitivity Function (dB)');
    xlabel('Frequency (rad/sec)');
    fig_str = 'Complimentary_Sensitivity_Function_Fig';
    title_str = sprintf('Complimentary Sensitivity Function');
    title(title_str);
end

if data_display == "Disturbance Sensitivity Function"
    xlim([ 0.1 100 ]);
    ylim([y_lim_lower y_lim_upper]);
    ylabel('Load Disturbance Sensitivity Function (dB)');
    xlabel('Frequency (rad/sec)');
    fig_str = 'Disturbance_Sensitivity_Function_Fig';
    title_str = sprintf('Load Disturbance Sensitivity Function');
    title(title_str);
end

fig_save_filename = sprintf('%s_%s.png', fig_str, datestr(now,'mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
print(save_filename,'-dpng')
saveas(gcf, save_filename);
fig_save_filename = sprintf('%s_%s.fig', fig_str, datestr(now,'mm-dd-yyyy_HH-MM'));
save_filename = fullfile(selpath,fig_save_filename);
savefig(gcf, save_filename);



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