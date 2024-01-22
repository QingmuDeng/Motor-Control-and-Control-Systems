%
%                        STMicroelectronics
%
%  Edukit Rotary Inverted Pendulum Real Time Control System Workbench
%
%                        Matlab Version 2.0
%                         April 4, 2021
%
% ************************* Description *****************************
%
% This system provides real time view and display of data
% from the STEVAL-EDUKIT01 Edukit System
%
%               For Additional Information Please See
% Please see: https://sites.google.com/view/ucla-st-motor-control/home
%
%
%***************************** Usage *********************************
%
%   1) Start the system by selecting Run at the Matlab Editor tab
%   2) The system will prompt to enter a directory into which logging
%      data will be stored
%   3) Select a mode that may include:
%       a) Rotor Step On
%       b) Pendulum Impulse On
%       c) Load Dist On (Load Disturbance Rejection Sensitivity Function)
%       d) Noise Sens On (Noise Sensitivity Function)
%       e) Sensitivity On (Sensitivity Function)
%       f) Sine On (sinusoidal reference tracking signal on)
%       g) Sine Off (sinusoidal reference tracking signal off)
%
%       h) Inc Step (increase the magnitude of parameter change request
%          for change in Pendulum or Rotor gains and change in Torque
%          current values)
%
%       i) Log Data (By selecting this, the user is prompted to enter
%          a number of 40 second measurement sweeps to acquire and store
%          Storage is to the previously selected directory in a file
%          labeled with a filename indicating the selection operation.
%          All measurement variables are included in the *.mat file
%          created.
%          This file may be opened by the system:
%          Edukit_Sensivity_Function_Freq_Response_Computation.m
%          This produces frequency response for the Sensitivity Function
%          data measured in this file.
%
%
%************************* Operation *********************************
%
% This system detects whether it is operating over Windows or Mac OSX
%
% This system operates by opening a serial port interface to the Edukit.
% The serial port is discovered and the Serial Interface Baud Rate is
% set to 23400.
%


clear all;
close all;
fclose all;
echo off;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

global selpath;
if ismac == 1
    fprintf('Select Output Data Directory for Data Log\r\n');
    selpath = uigetdir;
    if isequal(selpath,0)
        disp('User selected Cancel');
    else
        disp(['User selected ', selpath]);
    end
end

if ispc == 1
    fprintf('Select Output Data Directory for Data Log\r\n');
    selpath = uigetdir;
    if isequal(selpath,0)
        disp('User selected Cancel');
    else
        disp(['User selected ', selpath]);
    end
end


%
%
% This system requires Serial Port access to Edukit via Mac OSX
% or Windows platform USB ports.
%
% This system identified the platform type and then detects
% available Serial Ports.  The system must assume that only one
% device that creates a serial port is attached.
%
% A typical Serial port assignment for the USB port on Windows platform is
% Serial_Port = 'COM3';
%
% A typical Serial port assignment for the USB port on MacOSX platforms is
%
% /dev/tty.usbmodem1413
%

%
% Disover and assign serial port. This assumes that platform
% supports only one serial device assignment
%

if ispc == 1
    port_return = seriallist;
    Serial_Port = strip(port_return);
    num_ports = length(Serial_Port);
    if num_ports == 1
        disp(sprintf('Serial Port detected is: %s', string(Serial_Port)));
    end
    if num_ports > 1
        disp(sprintf('Serial Ports detected are:'));
        for i = 1:num_ports
            Port_Identifier(i) = string(Serial_Port(i));
            disp(sprintf('Serial Port Identifier %i is %s', i, Port_Identifier(i)));
        end
    end
end

if ismac == 1
    [status,cmdout] = system('ls /dev/tty.usbmodem*');
    port_return = regexprep(cmdout,'\n+','');
    Serial_Port = strip(port_return);
    num_ports = count(Serial_Port, 'dev');
    Serial_Port_List = split(Serial_Port);
    if num_ports == 1
        disp(sprintf('Serial Port detected is: %s', string(Serial_Port)));
    end
    if num_ports > 1
        disp(sprintf('Serial Ports detected are:'));
        for i = 1:num_ports
            Port_Identifier(i) = string(Serial_Port_List(i));
            disp(sprintf('Serial Port Identifier %i is %s', i, Port_Identifier(i)));
        end
    end
end

%
%   The Real Time Control System Workbench requires assignment of the
%   proper USB Serial Port assigned to the Edukit system by Windows.
%   If multiple devices are connected to the Windows platform, for example
%   this may include keyboard or mice devices, in addition to Edukit, then
%   the proper assignemnt is not known.
%
%   Use Windows Device Manager to determine the proper port.  Then, enter
%   this assignment at the prompt
%

if ispc == 1
if num_ports > 1
        disp(sprintf('\nError: Multiple USB devices have been detected'));
        Serial_Port_List = split(Serial_Port);
        disp(sprintf('Ports detected are:'));
        for i = 1:num_ports
            Port_Identifier(i) = string(Serial_Port_List(i));
            disp(sprintf('Serial Port Identifier %i is %s', i, Port_Identifier(i)));
        end
        
        disp(sprintf('\nReal Time Control System Workbench requires identification of Edukit USB serial port'));
        disp(sprintf('\nIdentify Edukit serial port with Windows Device Manager'));
        Serial_Port_Selection = input('Now, Enter the Edukit Serial Port Identifier: ');
        Serial_Port = Port_Identifier(Serial_Port_Selection);
    end
end

if ismac == 1
    if num_ports > 1
        disp(sprintf('Error: Multiple USB devices have been detected'));
        Serial_Port_List = split(Serial_Port);
        disp(sprintf('Ports detected are:'));
        for i = 1:num_ports
            Port_Identifier(i) = string(Serial_Port_List(i));
            disp(sprintf('Serial Port Identifier %i is %s', i, Port_Identifier(i)));
        end
        
        disp(sprintf('\nReal Time Control System Workbench requires identification of Edukit USB serial port'));
        disp(sprintf('\nIdentify Edukit serial port with Mac OSX Terminal with this command:'));
        disp(sprintf('\n ls /dev/tty.usbmodem*\n\n'));
        Serial_Port_Selection = input('Now, Enter the Edukit Serial Port Identifier: ');
        Serial_Port = Port_Identifier(Serial_Port_Selection);
    end
end

%
% Set Sweep Time for display
%

Time_Limit = 40;

%
% Screen_Update_Rate is the rate in times per second
% that points are added to the animated screen
%
% Screen update rate is set to adapt to computer speed
% Fast machines may set this value at 10. However, slower
% platforms may require a value of 1.
%
% Excessive values of Screen_Update_Rate may cause delays
% and stall of operation
%

Screen_Update_Rate = 2;

%
% Buffer Size determines characteristics of reliable USB serial
% data transport. The value of 256 has met performance requirements
%

Serial_Buffer = 1024;

%
% Number of characters in serial read of one control cycle
%
% The EDUKIT01 firmware provides the data stream. This is
% configured to 9 variables.  This may be changed according
% to changes in firmware in the future and by the user.
%

Read_Length = 9;

Rotor_Angle_Read_Gain = 8.887;
Pendulum_Angle_Read_Gain = 6.667;

%
% Scale factor compensates for gain applied to increase resolution
% in Edukit system for Load Disturbance measurement
%
LOAD_DISTURBANCE_SENSITIVITY_SCALE = 5;

%
% Axis definitions
%

Window_Time = 40;

Rotor_Angle_Range_Start = 10;
Rotor_Angle_Range_Min = 20;
Rotor_Angle_Range_Lower = 60;
Rotor_Angle_Range_Lower_Mid = 70;
Rotor_Angle_Range_Mid = 120;
Rotor_Angle_Range_Upper = 360;

Rotor_Angle_Range = Rotor_Angle_Range_Upper;
Rotor_Angle_Y_Upper_Limit = Rotor_Angle_Range/2;
Rotor_Angle_Y_Lower_Limit = -Rotor_Angle_Y_Upper_Limit;

Rotor_Track_Angle_Range_Start = 15;
Rotor_Track_Angle_Range_Min = 30;
Rotor_Track_Angle_Range_Lower = 60;
Rotor_Track_Angle_Range_Mid = 120;
Rotor_Track_Angle_Range_Upper = 360;
Rotor_Angle_Range_Max = 480;

Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Upper;
Rotor_Track_Angle_Y_Upper_Limit = Rotor_Track_Angle_Range/2;
Rotor_Track_Angle_Y_Lower_Limit = -Rotor_Track_Angle_Y_Upper_Limit;

Rotor_Target_Angle_Y_Upper_Limit = 3000;
Rotor_Target_Angle_Y_Lower_Limit = -3000;
Pendulum_Angle_Y_Upper_Limit = 8;
Pendulum_Angle_Y_Lower_Limit = -8;

%
% Close any open ports
%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%
% Open Nucleo port
%

global nucleo;
nucleo = serial(Serial_Port);
nucleo.Baudrate = 230400;
nucleo.Terminator = 'CR';
set(nucleo,'InputBufferSize', Serial_Buffer);
get(nucleo,{'InputBufferSize','BytesAvailable'});
set(nucleo, 'Timeout', 1);

try
    fopen(nucleo);
catch
    disp 'Serial Port access error - port may not be available';
    disp 'Check USB cable, serial port assignment and close other applications that may access the port';
    mh = msgbox(sprintf('Serial Port not available - Check USB cable and ensure that other applications using the Serial Port are terminated and then restart this system', 'replace'));
    fclose all;
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    
    close all;
    return;
end


global exit_state;
global enable_data_archive;
global enable_data_archive_request;

enable_data_archive = 0;
enable_data_archive_request = 0;

exit_state = 0;

global Pendulum_Impulse_Enable;
Pendulum_Impulse_Enable = 0;

if ispc == 1
    Figure_Font_Size = 10;
end
if ismac == 1
    Figure_Font_Size = 12;
end

fig = figure("menubar", "none", "units", "normalized", "Position", [0 0.05 1 1]);
p1 = subplot(4,1,1);
h1 = animatedline;
grid (p1, 'on');
axis([0,Window_Time,Rotor_Angle_Y_Lower_Limit,Rotor_Angle_Y_Upper_Limit]);
ylabel('Rotor Angle (deg)','Color','k');
set(gca,'FontSize',Figure_Font_Size);
title('STMicroelectronics Edukit Real Time Digital Motor Control Workbench');
axes_p1 = get(gca,'YLabel');

p2 = subplot(4,1,2);
h2 = animatedline;
grid (p2, 'on');
axis([0,Time_Limit,Pendulum_Angle_Y_Lower_Limit,Pendulum_Angle_Y_Upper_Limit]);
ylabel('Pendulum Angle (deg)','Color','k');
set(gca,'FontSize',Figure_Font_Size);

p3 = subplot(4,1,3);
h3 = animatedline;
grid (p3, 'on');
axis([0,Window_Time,Rotor_Track_Angle_Y_Lower_Limit,Rotor_Track_Angle_Y_Upper_Limit]);
ylabel('Rotor Tracking Command (deg)','Color','k');
set(gca,'FontSize',Figure_Font_Size);
axes_p3 = get(gca,'YLabel');

p4 = subplot(4,1,4);
h4 = animatedline;
grid (p4, 'on');
axis([0,Window_Time,Rotor_Target_Angle_Y_Lower_Limit,Rotor_Target_Angle_Y_Upper_Limit]);
xlabel('Sample Time (sec)');
ylabel('Rotor Control (deg/s/s)','Color','k');
set(gca,'FontSize',Figure_Font_Size);

display_cycles = 1;
vert_axis_string = sprintf('Disturbance Sensitivity x %i',LOAD_DISTURBANCE_SENSITIVITY_SCALE);


start_tic = tic;
average_rotor_angle = 0;
average_pendulum_angle = 0;
average_rotor_command = 0;
read_status = 0;

display_max_cycles = 10;
exit_request = 1;

index_count = 1;

display_cycles = 0;

%
% Graphics scaled to full screen and normalized to screen size
%

% Button Settings

offset = 0.1;
offset_spacing = 0.045;
button_height = 0.004;
button_width = 0.044;
button_vert_height = 0.05;
if ismac == 1
    button_font_size = 12;
end
if ispc == 1
    button_font_size = 10;
end

ButtonColorOff = [0.8 0.8 0.8];
ButtonColorOn = 'g';
ButtonColorInc = 'g';
ButtonColorDec = 'w';


c2 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Exit', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOff);
c2.String = {'Exit'};
c2.Callback = @ExitMode_ButtonPushed;
offset = offset + offset_spacing;

c = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Exit', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOff);
c.String = '<html><center>Stop<br /><center>Control</html>';
c.Callback = @TerminateMode_ButtonPushed;
offset = offset + offset_spacing;

c8 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Sine_Drive_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c8.String = '<html><center>Sine<br /><center>On</html>';
c8.Callback = @Sine_Drive_On_ButtonPushed;
offset = offset + offset_spacing;

c9 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Sine_Drive_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOff);
c9.String = '<html><center>Sine<br /><center>Off</html>';
c9.Callback = @Sine_Drive_Off_ButtonPushed;
offset = offset + offset_spacing;

c10 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Step','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c10.String = '<html><center>Inc<br /><center>Step</html>';
c10.Callback = @Inc_Step_ButtonPushed;
offset = offset + offset_spacing;

c11 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Step','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c11.String = '<html><center>Dec<br /><center>Step</html>';
c11.Callback = @Dec_Step_ButtonPushed;
offset = offset + offset_spacing;

c12 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Pend_P','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c12.String = '<html><center>Inc<br /><center>Pend P</html>';
c12.Callback = @Inc_Pend_P_ButtonPushed;
offset = offset + offset_spacing;

c13 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Pend_P','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c13.String = '<html><center>Dec<br /><center>Pend P</html>';
c13.Callback = @Dec_Pend_P_ButtonPushed;
offset = offset + offset_spacing;

c14 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Pend_I','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c14.String = '<html><center>Inc<br /><center>Pend I</html>';
c14.Callback = @Inc_Pend_I_ButtonPushed;
offset = offset + offset_spacing;

c15 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Pend_I','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c15.String = '<html><center>Dec<br /><center>Pend I</html>';
c15.Callback = @Dec_Pend_I_ButtonPushed;
offset = offset + offset_spacing;

c16 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Pend_D','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c16.String = '<html><center>Inc<br /><center>Pend D</html>';
c16.Callback = @Inc_Pend_D_ButtonPushed;
offset = offset + offset_spacing;

c17 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Pend_D','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c17.String = '<html><center>Dec<br /><center>Pend D</html>';
c17.Callback = @Dec_Pend_D_ButtonPushed;
offset = offset + offset_spacing;

c18 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Rotor_P','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c18.String = '<html><center>Inc<br /><center>Rotor P</html>';
c18.Callback = @Inc_Rotor_P_ButtonPushed;
offset = offset + offset_spacing;

c19 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Rotor_P','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c19.String = '<html><center>Dec<br /><center>Rotor P</html>';
c19.Callback = @Dec_Rotor_P_ButtonPushed;
offset = offset + offset_spacing;

c20 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Rotor_I','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c20.String = '<html><center>Inc<br /><center>Rotor I</html>';
c20.Callback = @Inc_Rotor_I_ButtonPushed;
offset = offset + offset_spacing;

c21 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Rotor_I','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c21.String = '<html><center>Dec<br /><center>Rotor I</html>';
c21.Callback = @Dec_Rotor_I_ButtonPushed;
offset = offset + offset_spacing;

c22 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Rotor_D','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c22.String = '<html><center>Inc<br /><center>Rotor D</html>';
c22.Callback = @Inc_Rotor_D_ButtonPushed;
offset = offset + offset_spacing;

c23 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Rotor_D','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c23.String = '<html><center>Dec<br /><center>Rotor D</html>';
c23.Callback = @Dec_Rotor_D_ButtonPushed;
offset = offset + offset_spacing;


offset = 0.92;
offset_spacing = 0.05;
button_height = 0.085;
button_width = 0.07;
button_vert_height = 0.048;
if ismac == 1
    button_font_size = 13;
end
if ispc == 1
    button_font_size = 10;
end


c37 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Torq','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c37.String = '<html><center>Inc<br /><center>Torq</html>';
c37.Callback = @Inc_Torq_ButtonPushed;
button_height = button_height + offset_spacing;

c38 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Torq','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c38.String = '<html><center>Dec<br /><center>Torq</html>';
c38.Callback = @Dec_Torq_ButtonPushed;
button_height = button_height + offset_spacing;

c39 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Max_Speed','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c39.String = '<html><center>Inc<br /><center>Max Spd</html>';
c39.Callback = @Inc_Max_Speed_ButtonPushed;
button_height = button_height + offset_spacing;

c40 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Max_Speed','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c40.String = '<html><center>Dec<br /><center>Max Spd</html>';
c40.Callback = @Dec_Max_Speed_ButtonPushed;
button_height = button_height + offset_spacing;

c41 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Inc_Min_Speed','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorInc);
c41.String = '<html><center>Inc<br /><center>Min Spd</html>';
c41.Callback = @Inc_Min_Speed_ButtonPushed;
button_height = button_height + offset_spacing;

c42 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Dec_Min_Speed','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c42.String = '<html><center>Dec<br /><center>Min Spd</html>';
c42.Callback = @Dec_Min_Speed_ButtonPushed;
button_height = button_height + offset_spacing;

c43 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Sens_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c43.String = '<html><center>Sensitivity<br /><center>On</html>';
c43.Callback = @Sens_On_ButtonPushed;
button_height = button_height + offset_spacing;

c44 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Sens_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c44.String = '<html><center>Sensitivity<br /><center>Off</html>';
c44.Callback = @Sens_Off_ButtonPushed;
button_height = button_height + offset_spacing;

c45 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Noise_Sens_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c45.String = '<html><center>Noise Sens<br /><center>On</html>';
c45.Callback = @Noise_Sens_On_ButtonPushed;
button_height = button_height + offset_spacing;

c46 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Noise_Sens_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c46.String = '<html><center>Noise Sens<br /><center>Off</html>';
c46.Callback = @Noise_Sens_Off_ButtonPushed;
button_height = button_height + offset_spacing;

c45 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Load_Dist_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c45.String = '<html><center>Load Dist<br /><center>On</html>';
c45.Callback = @Load_Dist_On_ButtonPushed;
button_height = button_height + offset_spacing;

c46 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Load_Dist_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c46.String = '<html><center>Load Dist<br /><center>Off</html>';
c46.Callback = @Load_Dist_Off_ButtonPushed;
button_height = button_height + offset_spacing;

c50 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Pendulum_Impulse_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c50.String = '<html><center>Pendulum<br /><center>Impulse On</html>';
c50.Callback = @Pendulum_Impulse_On_ButtonPushed;
button_height = button_height + offset_spacing;

c51 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Pendulum_Impulse_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c51.String = '<html><center>Pendulum<br /><center>Impulse Off</html>';
c51.Callback = @Pendulum_Impulse_Off_ButtonPushed;
button_height = button_height + offset_spacing;

c47 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Rotor_Step_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c47.String = '<html><center>Rotor Step<br /><center>On</html>';
c47.Callback = @Rotor_Step_On_ButtonPushed;
button_height = button_height + offset_spacing;

c48 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Button_Rotor_Step_Off','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorDec);
c48.String = '<html><center>Rotor Steo<br /><center>Off</html>';
c48.Callback = @Rotor_Step_Off_ButtonPushed;
button_height = button_height + offset_spacing;

c49 = uicontrol('Style','pushbutton', 'FontSize', button_font_size,'Tag','Log_Data_Button_On','units', 'normalized', 'Position', [offset button_height button_width button_vert_height], 'BackgroundColor', ButtonColorOn);
c49.String = '<html><center>Log<br /><center>Data</html>';
c49.Callback = @Log_Data_On_ButtonPushed;
button_height = button_height + offset_spacing;


win_x = 0.01;
win_y = 0.12;
win_width = 0.08;
win_height = 0.04;
win_offset = 0.06;
if ismac == 1
    window_fontsize = 11;
end
if ispc == 1
    window_fontsize = 9;
end


t0 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t1 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t2 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t3 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t4 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t5 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t6 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t7 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t8 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t9 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t10 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t11 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t12 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t13 = uicontrol(fig,'Style','text','BackgroundColor', 'k', 'ForegroundColor', 'w','FontWeight', 'bold', 'HorizontalAlignment',...
    'center','FontSize',window_fontsize,'units','normalized','Position', [win_x win_y win_width win_height]);



pend_p = 0;
pend_i = 0;
pend_d = 0;
rotor_p = 0;
rotor_i = 0;
rotor_d = 0;
max_speed = 0;
min_speed = 0;
torq_val = 0;
max_accel = 0;
max_decel = 0;
enable_disturbance_sensitivity_fcn = 0;
enable_noise_sensitivity_fcn = 0;
enable_rotor_step = 0;
enable_pend_impulse = 0;
enable_sensitivity_fnc = 0;
step_size = 0;
%
% display_parameter variable plot is selected by Edukit system according to
% mode selection:
%
% For
% Reference Tracking Mode
%       display_parameter = Rotor Angle
% Disturbance Sensitivity Function Mode
%       display_parameter = Rotor Angle
% Sensitivity Function Mode
%       display_parameter = Rotor Angle - Noise Signal
% Noise Sensitivity Function Mode
%       display_parameter = Rotor Target Angle (input to Rotor
%       actuator)

%
% Calibrate sample period
%
% The EDUKIT01 firmware includes a first variable that includes
% the time period between 100 consecutive samples. This value
% may be averaged and then applied to determine the period associated
% with each cycle.
%

% Configure standard speed sampling

fprintf(nucleo,')');
pause(0.5);
fprintf(nucleo,'y');
pause(0.5);

sample_period_average = 0;
i = 0;
while (i < 100)
    char_count = 0;
    [nucleo_data, char_count] = fscanf(nucleo);
    if char_count == 0
        disp 'Serial Port Read Error, Exiting...';
        fclose all;
        if ~isempty(instrfind)
            fclose(instrfind);
            delete(instrfind);
        end
        close all;
        return;
    end
    read_sequence_start = string(nucleo_data);
    nucleo_data_value = str2num(read_sequence_start);
    data_length = size(nucleo_data_value);
    if data_length(2) ~= Read_Length
        continue;
    end
    % cycle_time includes the offset of 200 (in units of 10 microsec)
    if nucleo_data_value(1) == 2
        sample_period_average = sample_period_average + nucleo_data_value(2) + 200;
        i = i + 1;
    end
end

sample_period_average = sample_period_average/i;

%
% Sample_Period_Scale determines the scaling between reported cycle time
% and sample time.  The EDUKIT01 reported cycle time is an average sver
% 100 cycles.  This is provided in nucleo_data_value(1).
%
% Thus, cycle period is nucleo_data_value(1)/100 in miliseconds
%

Sample_Period_Scale = (1/1000)*(1/100);

Sample_Period = sample_period_average * Sample_Period_Scale;

%
% Enable exit from data logging when sample count exceeds threshold
% of Sample_Count_Threshold
%

control_cycles = 1;
m = 1;
sample_index = 0;
read_status = 0;
start_tic = tic;

sample_sweep_count = 0;
Sweep_Count_Threshold = 3;

Count_Limit = Time_Limit/Sample_Period;
numpoints = 50000;
Sample_Count_Threshold = 3200;
sweep_count_status = 0;
message_status = 0;

while exit_state == 0
    
    sample_index = sample_index + 1;
    attempt_count = 0;
    
    if (sample_sweep_count - 1) >= Sweep_Count_Threshold
        exit_state = 1;
    end
    
    for k = 1:numpoints
        if k == 1
            while true
                char_count = 0;
                [nucleo_data, char_count] = fscanf(nucleo);
                if char_count == 0
                    disp 'Serial Port Read Error, Exiting...';
                    fclose all;
                    if ~isempty(instrfind)
                        fclose(instrfind);
                        delete(instrfind);
                    end
                    close all;
                    return;
                end
                read_sequence_start = string(nucleo_data);
                nucleo_data_value = str2num(read_sequence_start);
                data_length = size(nucleo_data_value);
                if data_length(2) == Read_Length
                    break;
                end
            end
        end
        while true
            if exit_state == 1
                break;
            end
            char_count = 0;
            [nucleo_data, char_count] = fscanf(nucleo);
            if char_count == 0
                disp 'Serial Port Read Error, Exiting...';
                fclose all;
                if ~isempty(instrfind)
                    fclose(instrfind);
                    delete(instrfind);
                end
                close all;
                return;
            end
            
            read_sequence_start = string(nucleo_data);
            nucleo_data_value = str2num(read_sequence_start);
            data_length = size(nucleo_data_value);
            if data_length(2) == Read_Length
                break;
            end
            if data_length(2) ~= Read_Length
                attempt_count = attempt_count + 1;
                if attempt_count > 50
                    disp 'Serial Port Read Error Count, Exiting...';
                    fclose all;
                    if ~isempty(instrfind)
                        fclose(instrfind);
                        delete(instrfind);
                    end
                    close all;
                    return;
                end
            end
        end
        
        % cycle_time includes the offset of 200 (in units of 10 microsec)
        % motor_position_target includes scaling of 100
        
        if (nucleo_data_value(1) == 2)
            cycle_time(k) = nucleo_data_value(2);
            pendulum_angle(k) = nucleo_data_value(4)/Pendulum_Angle_Read_Gain;
            rotor_angle(k) = nucleo_data_value(5)/Rotor_Angle_Read_Gain;
            motor_position_target(k) = nucleo_data_value(8)/Rotor_Angle_Read_Gain;
            if Pendulum_Impulse_Enable == 0
                rotor_command(k) = nucleo_data_value(7)/Rotor_Angle_Read_Gain;
            end
            if Pendulum_Impulse_Enable == 1
                rotor_command(k) = nucleo_data_value(7)/Pendulum_Angle_Read_Gain;
            end
            
        end
        
        if nucleo_data_value(1) == 0
            pend_p = nucleo_data_value(2);
            pend_i = nucleo_data_value(3);
            pend_d = nucleo_data_value(4);
            rotor_p = nucleo_data_value(5);
            rotor_i = nucleo_data_value(6);
            rotor_d = nucleo_data_value(7);
            max_speed = nucleo_data_value(8);
            min_speed = nucleo_data_value(9);
            if k > 1
                sample_time(k) = sample_time(k-1);
                pendulum_angle(k) = pendulum_angle(k-1);
                rotor_angle(k) = rotor_angle(k-1);
                motor_position_target(k) = motor_position_target(k-1);
                rotor_command(k) = rotor_command(k-1);
            end;
            if k == 1
                sample_time(k) = sample_time_p;
                pendulum_angle(k) = pendulum_angle_p;
                rotor_angle(k) = rotor_angle_p;
                motor_position_target(k) = motor_position_target_p;
                rotor_command(k) = rotor_command_p;
            end;
        end
        
        if nucleo_data_value(1) == 1
            torq_val = nucleo_data_value(2);
            max_accel = nucleo_data_value(3);
            max_decel = nucleo_data_value(4);
            enable_disturbance_sensitivity_fcn = nucleo_data_value(5);
            enable_noise_sensitivity_fcn = nucleo_data_value(6);
            enable_rotor_step = nucleo_data_value(7);
            step_size = nucleo_data_value(8)/10;
            enable_sensitivity_fnc = nucleo_data_value(9);
            if k > 1
                sample_time(k) = sample_time(k-1);
                pendulum_angle(k) = pendulum_angle(k-1);
                rotor_angle(k) = rotor_angle(k-1);
                motor_position_target(k) = motor_position_target(k-1);
                rotor_command(k) = rotor_command(k-1);
            end;
            if k == 1
                sample_time(k) = sample_time_p;
                pendulum_angle(k) = pendulum_angle_p;
                rotor_angle(k) = rotor_angle_p;
                motor_position_target(k) = motor_position_target_p;
                rotor_command(k) = rotor_command_p;
            end;
        end
        
        
        sample_time(k) = (k-1)*Sample_Period;
        if k > Count_Limit
            display_cycles = display_cycles + 1;
            if enable_data_archive == 1
                sample_sweep_count = sample_sweep_count + 1;
            end
            sample_time_p = sample_time(k-1);
            pendulum_angle_p = pendulum_angle(k-1);
            rotor_angle_p = rotor_angle(k-1);
            motor_position_target_p = motor_position_target(k-1);
            rotor_command_p = rotor_command(k-1);
            break;
        end
        
        if exit_state == 1
            break;
        end
        
        %
        %   Update data display
        %
        
        if mod((k - 1), 100) == 0 && exit_state == 0
            
            String = sprintf("Step Size\n%.1f step/s", step_size); t0.String = String;
            String = sprintf("Max Speed\n%i step/s", max_speed); t1.String = String;
            String = sprintf("Min Speed\n%i step/s", min_speed); t2.String = String;
            String = sprintf("Max Decel\n%i step/s/s", max_decel); t3.String = String;
            String = sprintf("Max Accel\n%i step/s/s", max_accel); t4.String = String;
            String = sprintf("Torq Current\n%i mA", torq_val); t5.String = String;
            String = sprintf("Rotor PID\nD Gain %.1f", rotor_d); t6.String = String;
            String = sprintf("Rotor PID\nI Gain %.1f", rotor_i); t7.String = String;
            String = sprintf("Rotor PID\nP Gain %.1f", rotor_p); t8.String = String;
            String = sprintf("Pend PID\nP Gain %.1f", pend_p); t11.String = String;
            String = sprintf("Pend PID\nI Gain %.1f", pend_i); t10.String = String;
            String = sprintf("Pend PID\nD Gain %.1f", pend_d); t9.String = String;
            if enable_rotor_step == 1
                String = sprintf("Rotor Step\n   On"); t12.String = String;
            end
            if enable_rotor_step == 0
                String = sprintf("Rotor Step\n  Off"); t12.String = String;
            end
            if enable_sensitivity_fnc == 1
                String = sprintf("Sensitivity\nFunction On"); t13.String = String;
                set(axes_p1, 'String', 'Sensitivity Function');
            end
            if enable_noise_sensitivity_fcn == 1
                String = sprintf("Noise Sens\nFunction On"); t13.String = String;
                set(axes_p1, 'String', 'Noise Sensitivity Function');
            end
            if enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0
                String = sprintf("Comp Sens\nFunction On"); t13.String = String;
                set(axes_p1, 'String', 'Rotor Angle (degrees)');
            end
            if enable_disturbance_sensitivity_fcn == 1
                String = sprintf("Disturbance Sens\nFunction On"); t12.String = String;
                String = sprintf("Disturbance Sens\nFunction On"); t13.String = String;
                set(axes_p1, 'String', vert_axis_string);
            end
            if Pendulum_Impulse_Enable == 1
                String = sprintf("Pendulum\nImpulse On"); t13.String = String;
                set(axes_p3, 'String', 'Pendulum Impulse (deg)');
            end
            if Pendulum_Impulse_Enable == 0
                String = sprintf("Pendulum\nImpulse Off"); t13.String = String;
                set(axes_p3, 'String', 'Rotor Tracking Command (deg)');
            end
        end
        
        %
        % Rotor angle and rotor tracking angle plot autoscale
        %
        update_state = 0;
        if mod((k - 1), 1000) == 0 && k > 1000
            if k > 1000
                Max_Rotor_Angle = max(abs(rotor_angle(1:k)));
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Start;
                Rotor_Angle_Range = Rotor_Angle_Range_Min;
                update_state = 1;
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Min;
                Rotor_Angle_Range = Rotor_Angle_Range_Lower;
                update_state = 1;
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Lower;
                Rotor_Angle_Range = Rotor_Angle_Range_Lower_Mid;
                update_state = 1;
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Lower_Mid;
                Rotor_Angle_Range = Rotor_Angle_Range_Mid;
                update_state = 1;
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Mid;
                Rotor_Angle_Range = Rotor_Angle_Range_Upper;
                update_state = 1;
            end
            if Max_Rotor_Angle > 0.48*Rotor_Angle_Range_Upper;
                Rotor_Angle_Range = Rotor_Angle_Range_Max;
                if enable_noise_sensitivity_fcn == 1
                    Rotor_Angle_Range = Rotor_Angle_Range_Max * 100;
                end
                update_state = 1;
            end
            
            if update_state == 0
                if Max_Rotor_Angle < 0.1*Rotor_Angle_Range_Max;
                    Rotor_Angle_Range = Rotor_Angle_Range_Upper;
                    update_state = 1;
                end
                if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Upper;
                    Rotor_Angle_Range = Rotor_Angle_Range_Upper;
                    update_state = 1;
                end
                if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Lower_Mid;
                    Rotor_Angle_Range = Rotor_Angle_Range_Lower_Mid;
                    update_state = 1;
                end
                if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Lower;
                    Rotor_Angle_Range = Rotor_Angle_Range_Lower;
                    update_state = 1;
                end
                if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Min;
                    Rotor_Angle_Range = Rotor_Angle_Range_Min;
                    update_state = 1;
                end
                if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Start;
                    Rotor_Angle_Range = Rotor_Angle_Range_Start;
                    update_state = 1;
                end
            end
            
            update_state_track = 0;
            if mod((k - 1), 1000) == 0 && k > 1000
                Max_Rotor_Track_Angle = max(abs(rotor_command(1:k)));
                if Max_Rotor_Track_Angle > 0.45*Rotor_Track_Angle_Range_Start;
                    Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Min;
                    update_state_track = 1;
                end
                if Max_Rotor_Track_Angle > 0.45*Rotor_Track_Angle_Range_Min;
                    Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Lower;
                    update_state_track = 1;
                end
                if Max_Rotor_Track_Angle > 0.45*Rotor_Track_Angle_Range_Lower;
                    Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Mid;
                    update_state_track = 1;
                end
                if Max_Rotor_Track_Angle > 0.45*Rotor_Track_Angle_Range_Mid;
                    Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Upper;
                    update_state_track = 1;
                end
                if update_state_track == 0
                    if Max_Rotor_Track_Angle < 0.8*Rotor_Track_Angle_Range_Lower;
                        Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Lower;
                        update_state_track = 1;
                    end
                    if Max_Rotor_Track_Angle < 0.8*Rotor_Track_Angle_Range_Min;
                        Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Min;
                        update_state_track = 1;
                    end
                    if Max_Rotor_Track_Angle < 0.8*Rotor_Track_Angle_Range_Start;
                        Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Start;
                        update_state_track = 1;
                    end
                end
                
            end
            
            if enable_data_archive_request == 1 && enable_data_archive == 0
                enable_data_archive = 1;
                fprintf(nucleo,'>');
            end
            
            if exit_state == 1
                break;
            end
            
            if update_state == 1 && exit_state == 0
                Rotor_Angle_Y_Upper_Limit = Rotor_Angle_Range/2;
                Rotor_Angle_Y_Lower_Limit = -Rotor_Angle_Y_Upper_Limit;
                set(p1, 'Ylim',[Rotor_Angle_Y_Lower_Limit,Rotor_Angle_Y_Upper_Limit]);
                drawnow;
                update_state = 0;
            end
            
            if update_state_track == 1 && exit_state == 0
                Rotor_Track_Angle_Y_Upper_Limit = Rotor_Track_Angle_Range/2;
                Rotor_Track_Angle_Y_Lower_Limit = -Rotor_Track_Angle_Y_Upper_Limit;
                set(p3, 'Ylim',[Rotor_Track_Angle_Y_Lower_Limit,Rotor_Track_Angle_Y_Upper_Limit]);
                drawnow;
                update_state_track = 0;
            end
            
        end
        
        if enable_data_archive == 1 && sweep_count_status == 0
            prompt = {'Enter Sweep Count (4 or greater for Sensivity Functions'};
            dlgtitle = 'Input';
            dims = [1 80];
            definput = {'3'};
            opts.Interpreter = 'tex';
            try
                sweep_entry = inputdlg(prompt,dlgtitle,dims,definput,opts);
                Sweep_Count_Threshold = str2num(cell2mat(sweep_entry(1)))
            catch
                disp(sprintf('No entry of Sweep Count - assigning a value of 3'));
                Sweep_Count_Threshold = 3;
            end
            sweep_count_status = 1;
        end
        
        if enable_data_archive == 1 && message_status == 0
            disp(sprintf("Sampling Started\nSystem Will Exit After %i Sweeps of 40 Seconds Each\n", Sweep_Count_Threshold));
            mh = msgbox(sprintf("     Sampling Started\n   System Will Exit After\n          %i Sweeps\n",  Sweep_Count_Threshold), 'replace');
            th = findall(mh, 'Type', 'Text');
            message_status = 1;
        end
        
        
        if enable_data_archive == 1
            
            rotor_angle_arch(m) = rotor_angle(k);
            motor_position_target_arch(m) = motor_position_target(k);
            rotor_command_arch(m) = rotor_command(k);
            pendulum_angle_arch(m) = pendulum_angle(k);
            sample_time_arch(m) = (m - 1)*Sample_Period;
            m = m + 1;
        end
        
        if k > 1
            addpoints(h1,sample_time(k),rotor_angle(k))
            addpoints(h2,sample_time(k),pendulum_angle(k));
            addpoints(h3,sample_time(k),rotor_command(k));
            addpoints(h4,sample_time(k),motor_position_target(k));
        end
        
        timer_tic = toc(start_tic);     % Set Timer start
        if timer_tic > (1/Screen_Update_Rate)
            drawnow;                    % Update screen according to Screen_Update_Rate
            start_tic = tic;            % Reset timer after updating
        end
    end
    
    if exit_state == 1
        break;
    end
    
    clearpoints(h1);
    clearpoints(h2);
    clearpoints(h3);
    clearpoints(h4);
    drawnow;
    grid on;
    
    
    if enable_data_archive == 1
        disp(sprintf("%i Sampling Sweeps of %i Completed", sample_sweep_count - 1, Sweep_Count_Threshold));
        mh = msgbox(sprintf("%i of %i Sampling Sweeps Completed",  sample_sweep_count - 1, Sweep_Count_Threshold), 'replace');
        th = findall(mh, 'Type', 'Text');
    end
end

close all;

fclose all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

if enable_data_archive == 1
    motor_position_target = motor_position_target_arch;
    rotor_command = rotor_command_arch;
    pendulum_angle = pendulum_angle_arch;
    if enable_sensitivity_fnc == 1
        data_display = 'Sensitivity Function';
        %Edukit system supplies Sensitivty Function in Rotor Angle
        %variable
        rotor_angle = rotor_angle_arch;
        sample_time = sample_time_arch;
    end
    if enable_noise_sensitivity_fcn == 1
        data_display = 'Noise Sensitivity Function';
        %Edukit system supplies Noise Sensitivty Function in Rotor Angle
        %variable
        rotor_angle = (rotor_angle_arch);
        sample_time = sample_time_arch;
    end
    if enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0
        data_display = 'Step Response Rotor Angle (degrees)';
        rotor_angle = rotor_angle_arch;
        sample_time = sample_time_arch;
    end
    if enable_disturbance_sensitivity_fcn == 1
        data_display = 'Disturbance Sensitivity Function';
        %Edukit system supplies Load Disturbance Sensitivty Function in Rotor Angle
        %variable
        rotor_angle = (rotor_angle_arch)/LOAD_DISTURBANCE_SENSITIVITY_SCALE;
        sample_time = sample_time_arch;
    end
    
    if Pendulum_Impulse_Enable == 1 && enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0
        data_display = 'Pendulum Angle Impulse (degrees)';
        rotor_angle = rotor_angle_arch;
        sample_time = sample_time_arch;
    end
end

if enable_data_archive == 1
    save_filename = sprintf('Edukit_%s_%s.mat', data_display, datestr(now,'_mm-dd-yyyy_HH-MM'));
    selpath = [selpath,'/'];
    save_filename = [selpath,save_filename];
    save(save_filename, "data_display", "sample_time","motor_position_target", "rotor_command", "rotor_angle", "pendulum_angle", "Sample_Period", "max_speed", "min_speed", "max_decel", "max_accel", "torq_val", "pend_p", "pend_i", "pend_d", "rotor_p", "rotor_i", "rotor_d", "Pendulum_Impulse_Enable");
end

function TerminateMode_ButtonPushed(src,event)
global nucleo;
global exit_state;
exit_state = 1;
fprintf(nucleo,'q');
fclose all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all;
return;
end

function ExitMode_ButtonPushed(src,event)
global exit_state;
exit_state = 1;
fclose all;
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

close all;
return;
end


function Sine_Drive_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'5');
handles = guihandles(hObject);
return;
end

function Sine_Drive_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'9');
handles = guihandles(hObject);
return;
end

function Inc_Step_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'J');
return;
end

function Dec_Step_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'j');
return;
end

function Inc_Pend_P_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'A');
return;
end

function Dec_Pend_P_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'a');
return;
end

function Inc_Pend_I_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'B');
return;
end

function Dec_Pend_I_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'b');
return;
end

function Inc_Pend_D_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'C');
return;
end

function Dec_Pend_D_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'c');
return;
end


function Inc_Rotor_P_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'D');
return;
end

function Dec_Rotor_P_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'d');
return;
end

function Inc_Rotor_I_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'E');
return;
end

function Dec_Rotor_I_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'e');
return;
end

function Inc_Rotor_D_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'F');
return;
end

function Dec_Rotor_D_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'f');
return;
end

function Mode_M_D_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'u');
return;
end

function Inc_Torq_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'T');
return;
end

function Dec_Torq_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'t');
return;
end

function Inc_Max_Speed_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'S');
return;
end

function Dec_Max_Speed_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'s');
return;
end

function Inc_Min_Speed_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'M');
return;
end

function Dec_Min_Speed_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'m');
return;
end

function Sens_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'V');
return;
end

function Sens_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'v');
return;
end

function Noise_Sens_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'R');
return;
end

function Noise_Sens_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'r');
return;
end

function Load_Dist_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'L');
return;
end

function Load_Dist_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'l');
return;
end

function Log_Data_On_ButtonPushed(hObject,callbackdata,handles)
global enable_data_archive_request;
enable_data_archive_request = 1;
return;
end

function Rotor_Step_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'P');
return;
end

function Rotor_Step_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
fprintf(nucleo,'p');
return;
end

function Pendulum_Impulse_On_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
global Pendulum_Impulse_Enable;
Pendulum_Impulse_Enable = 1;
fprintf(nucleo,'H');
return;
end

function Pendulum_Impulse_Off_ButtonPushed(hObject,callbackdata,handles)
global nucleo;
global Pendulum_Impulse_Enable;
Pendulum_Impulse_Enable = 0;
fprintf(nucleo,'h');
return;
end