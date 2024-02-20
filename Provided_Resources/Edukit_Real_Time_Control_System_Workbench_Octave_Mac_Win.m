%
%                        STMicroelectronics
%
%  Edukit Rotary Inverted Pendulum Real Time Control System Workbench
%
%                     Octave Version 2.0 for MacOSX
%                         October 14, 2021
%
% ************************* Requirement *****************************
%
% This system requires the Octave file, animatedline.m  
% The animatedline.m file must be included in the same directory as this
% file.  
%
% This system also requires usage of Octave 6.1 or greater
%
% Directions for installation of Octave on Windows and MacOSX are 
% included here: https://sites.google.com/view/ucla-st-motor-control/home
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
%***************************** Requirements *************************
%
%    This system requires access to the Edukit serial port created by the USB
%    connection between the computer and the Edukit system.
%
%    For correct operation, only one serial device should be connected to 
%    the computer.s
%
%    If only one device is connected to the computer, this will be 
%    automatically selected and configured.
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
%          Edukit_Sensivity_Function_Freq_Response_Computation_Octave.m 
%          This produces frequency response for the Sensitivity Function
%          data measured in this file.
%          
%
%************************* Operation *********************s************
%
% This system detects whether it is operating over Windows or Mac OSX
%
% This system operates by opening a serial port interface to the Edukit.
% The serial port is discovered and the Serial Interface Baud Rate is 
% set to 23400.
%


clear;
clear -all;
clear -allchild;
close all;
fclose all; 

Serial_Port = '/dev/ttyACM0';

 warning('off', 'all');

pkg load instrument-control

    ver = version;
    if compare_versions('6.1.0', ver, "<=") == 0
      disp(sprintf('This System Requires Octave Version 6.1.0 or greater'));
      mh = msgbox(sprintf('This System Requires Octave Version 6.1.0 or greater', 'replace'));
      return;
    end


%
% Ensure Current Directory includes animateline.m
%

 c = exist('animatedline.m');
 if c ~= 2
   mh = msgbox(sprintf('Error: Octave File animatedline.m not present, ensure file is loaded in same directory', 'replace'));
   disp('Error: Octave File animatedline.m not present, ensure file is loaded in same directory');
    close all;
    fclose all; 
    return;
 end


%
%      
%
% Operation requires that only this system accesses the Serial Port
% Assigned to the Edukit system. Ensure that terminal sessions are
% not operating when executing this system

%
% Octave system display rate is limited due to the combined 
% processing burden of data acquisition and display animation
% This Real Time Viewer operates in two modes:
% 1) Display Mode with sample rate of 25 points per second
% 2) Data Logging rate with sample rate of 500 samples per second
%
% Note that data display is suspended during logging
%
% The Screen_Update_Rate may be adjusted to smaller values to support
% slower machines or increased for faster machines.  The default value is 100 
%

Screen_Update_Rate = 100;

%
% The GNUTERM qt graphics standard serves MacOSX and Windows 10
% 

Select_GNUTERM = 1;
Select_X11 = 0;

if Select_GNUTERM == 1
  setenv('GNUTERM','qt')
  graphics_toolkit('qt');
end

if Select_X11 == 1
  setenv ("GNUTERM", "X11")
end


current_path = pwd;
selpath = uigetdir (current_path, 'Select Output Data Directory for Data Log');
    if strcmp (selpath,"")
        disp('User selected Cancel');
    else
        disp(sprintf('User selected: %s', selpath));
    end



%
% Important Setup Configuration
%

%
% If a single serial port has been assigned by the operating system
% Then this will be discovered and assigned below.
%
% Otherwise, it will be required to set the serial port assignment
% manually to equal that assigned to the IRIP Nucleo.
%


%
% Detect and report available serial port assignments 
%
% Detect whether system OS is Windows or MacOSC 
%


if ispc == 1
   ports = seriallist;
   port_avail = char(ports);
   disp(sprintf('Serial Ports Currently Assigned to Devices: %s\n', port_avail));
   Serial_Port = strtrim(port_avail);
end
 

if isunix == 1
  [~, text] = system ("ls /dev/ttyACM*");
  port_return = regexprep(text,'\n+','');
  disp(sprintf('Serial Ports Currently Assigned to Devices: %s\n', port_return));
  Serial_Port = strtrim(port_return);
end

%
% Typical Macbook or Windows Notebook settings
%

% Screen settings

Plot_Font_Size = 12;

global enable_data_archive;
global Pendulum_Impulse_Enable;
global irip_serial_port;

Pendulum_Impulse_Enable = 0;
enable_data_archive = 0;

function [char_array] = ReadToTermination (srl_handle, term_char)
  
  %
  % Function developed by Steve Hageman
  % https://www.edn.com/read-serial-data-directly-into-octave/
  %
  
  % parameter term_char is optional, if not specified
  % then CR = 'r' = 13dec is the default.
  
  if (nargin == 1)
    term_char = 13;
  end
  
  not_terminated = true;
  i = 1;
  int_array = uint8(1);
  
  while not_terminated
    val = read(srl_handle,1);
    if(val == term_char)
    not_terminated = false;
  end
  % Add char received to array
  int_array(i) = val;
  i = i + 1;
end
% Change int array to a char array and return a string array
char_array = char(int_array);
endfunction

global exit_state;
exit_state = 0;

global button_color_off;
button_color_off = [0.8 0.8 0.8];
global button_color_on;
button_color_on = [0 0.9 0];;

function ExitMode_ButtonPushed(src,event)
global exit_state;
global irip_serial_port;
exit_state = 1;
%fclose(irip_serial_port);
%fclose all;
close all;
return;
end

function TerminateMode_ButtonPushed(src,event)
global irip_serial_port;
global exit_state;
exit_state = 1;
write(irip_serial_port,"q\r");
%fclose(irip_serial_port);
%fclose all;
close all;
return;
end

function Mode_L_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c3; global c4; global c5; 
write(irip_serial_port,"3\r");
return;
end

function Mode_M_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c3; global c4; global c5; 
write(irip_serial_port,"1\r");
return;
end

function Mode_D_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c40;
write(irip_serial_port,"u\r");
return;
end

function Mode_H_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c3; global c4; global c5; 
write(irip_serial_port,"2\r");
return;
end

function Mode_Adaptive_On_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c6; global c7; global c3; global c4; global c5;
global button_color_off; global button_color_on;
set(c6,'backgroundcolor',button_color_on);
set(c7,'backgroundcolor',button_color_off);
set(c3,'backgroundcolor',button_color_off);
set(c4,'backgroundcolor',button_color_off);
set(c5,'backgroundcolor',button_color_off);

write(irip_serial_port,"7\r");
return;
end

function Mode_Adaptive_Off_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c6; global c7; global c3; global c4; global c5;
global button_color_off; global button_color_on;
set(c6,'backgroundcolor',button_color_off);
set(c7,'backgroundcolor',button_color_on);
set(c3,'backgroundcolor',button_color_off);
set(c4,'backgroundcolor',button_color_on);
set(c5,'backgroundcolor',button_color_off);

write(irip_serial_port,"6\r");
return;
end

function Sine_Drive_Off_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c8; global c9; 
global button_color_off; global button_color_on;
set(c8,'backgroundcolor',button_color_off);
set(c9,'backgroundcolor',button_color_on);

write(irip_serial_port,"9\r");
return;
end

function Sine_Drive_On_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c8; global c9;
global button_color_off; global button_color_on;
set(c8,'backgroundcolor',button_color_on);
set(c9,'backgroundcolor',button_color_off);

write(irip_serial_port,"5\r");
return;
end

function Max_Speed_Inc_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c13;
write(irip_serial_port,"S\r");
return;
end

function Max_Speed_Dec_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c14;
write(irip_serial_port,"s\r");
return;
end

function Min_Speed_Inc_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c15;
write(irip_serial_port,"M\r");
return;
end

function Min_Speed_Dec_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c16;
write(irip_serial_port,"m\r");
return;
end

function Max_Accel_Inc_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c17;
write(irip_serial_port,"N\r");
return;
end

function Max_Accel_Dec_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c18;
write(irip_serial_port,"n\r");
return;
end

function Dec_Torq_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c10;
write(irip_serial_port,"t\r");
return;
end

function Inc_Torq_ButtonPushed(hObject,callbackdata,handles)
global irip_serial_port;
global c11;
write(irip_serial_port,"T\r");
return;
end

function Max_Decel_Inc_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c19;
write(irip_serial_port,"O\r");
return;
end

function Max_Decel_Dec_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c20;
write(irip_serial_port,"o\r");
return;
end

function Sens_Fnc_On_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c21;
write(irip_serial_port,"V\r");
return;
end

function Sens_Fnc_Off_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c22;
write(irip_serial_port,"v\r");
return;
end

function Noise_Sens_Fcn_On_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c23;
write(irip_serial_port,"R\r");
return;
end

function Noise_Sens_Fcn_Off_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c24;
write(irip_serial_port,"r\r");
return;
end

function Load_Disturbance_Sens_Fcn_On_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c25;
write(irip_serial_port,"L\r");
return;
end

function Load_Disturbance_Sens_Fcn_Off_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c26;
write(irip_serial_port,"l\r");
return;
end

function Inc_Pend_P_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c27;
write(irip_serial_port,"A\r");
return;
end

function Dec_Pend_P_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c28;
write(irip_serial_port,"a\r");
return;
end

function Inc_Pend_I_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c29;
write(irip_serial_port,"B\r");
return;
end

function Dec_Pend_I_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c30;
write(irip_serial_port,"b\r");
return;
end

function Inc_Step_Size_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c31;
write(irip_serial_port,"J\r");
return;
end

function Dec_Step_Size_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c40;
write(irip_serial_port,"j\r");
return;
end

function Inc_Pend_D_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c32;
write(irip_serial_port,"C\r");
return;
end

function Dec_Pend_D_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c33;
write(irip_serial_port,"c\r");
return;
end

function Inc_Rotor_P_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c34;
write(irip_serial_port,"D\r");
return;
end

function Dec_Rotor_P_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c35;
write(irip_serial_port,"d\r");
return;
end

function Inc_Rotor_I_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c36;
write(irip_serial_port,"E\r");
return;
end

function Dec_Rotor_I_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c37;
write(irip_serial_port,"e\r");
return;
end

function Inc_Rotor_D_ButtonPushed(hObject,callbackdata,handles)           
global irip_serial_port;
global c38;
write(irip_serial_port,"F\r");
return;
end

function Dec_Rotor_D_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c39;
write(irip_serial_port,"f\r");
return;
end

function Log_Data_ButtonPushed(hObject,callbackdata,handles)          
global enable_data_archive;
global c40;
enable_data_archive = 1;
return;
end

function Rotor_Step_On_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c41;
write(irip_serial_port,"P\r");
return;
end

function Rotor_Step_Off_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c42;
write(irip_serial_port,"p\r");
return;
end

function Pendulum_Impulse_On_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c43;
global Pendulum_Impulse_Enable;
Pendulum_Impulse_Enable = 1;
write(irip_serial_port,"H\r");
return;
end

function Pendulum_Impulse_Off_ButtonPushed(hObject,callbackdata,handles)          
global irip_serial_port;
global c44;
global Pendulum_Impulse_Enable;
Pendulum_Impulse_Enable = 0;
write(irip_serial_port,"h\r");
return;
end

try
irip_serial_port = serialport(Serial_Port,230400);
catch
disp(sprintf('Serial Port not available - Ensure that other applications using the Serial Port are terminated '));
mh = msgbox(sprintf('Serial Port not available - Ensure that other applications using the Serial Port are terminated and then restart this system', 'replace'));
close all;
return
end_try_catch

%
% Rotor and Pendulum conversion gain from digital
% sensor value to degrees
%

Rotor_Angle_Read_Gain =     8.889;       
Pendulum_Angle_Read_Gain =  6.667;

%
% Set Sweep Time for display
%

Window_Time = 40;

%
% Plot axis limits including ranges for autoscaling
%

y_max_pend = 3;
y_min_pend = -3;

y_max_rotor = 120;
y_min_rotor = -120;

Rotor_Angle_Range_Start = 20;
Rotor_Angle_Range_Min = 40;
Rotor_Angle_Range_Lower = 50;
Rotor_Angle_Range_Lower_Mid = 70;
Rotor_Angle_Range_Mid = 120;
Rotor_Angle_Range_Upper = 360;
Rotor_Angle_Range_Max = 480;

Rotor_Angle_Range = Rotor_Angle_Range_Upper;
Rotor_Angle_Y_Upper_Limit = Rotor_Angle_Range/2;
Rotor_Angle_Y_Lower_Limit = -Rotor_Angle_Y_Upper_Limit;

Rotor_Track_Angle_Range_Start = 15;
Rotor_Track_Angle_Range_Min = 30;
Rotor_Track_Angle_Range_Lower = 60;
Rotor_Track_Angle_Range_Mid = 120;
Rotor_Track_Angle_Range_Upper = 360;

Rotor_Track_Angle_Range = Rotor_Track_Angle_Range_Upper;
Rotor_Track_Angle_Y_Upper_Limit = Rotor_Track_Angle_Range/2;
Rotor_Track_Angle_Y_Lower_Limit = -Rotor_Track_Angle_Y_Upper_Limit;

Rotor_Target_Angle_Y_Upper_Limit = 3000;
Rotor_Target_Angle_Y_Lower_Limit = -3000;
Pendulum_Angle_Y_Upper_Limit = 4;
Pendulum_Angle_Y_Lower_Limit = -4;

if isunix
  window_xs = 0.0;
  window_ys = 0.0;
  window_width = 0.9;
  window_height = 0.9;
end
if ispc
  window_xs = 0.0;
  window_ys = 0.0;
  window_width = 0.95;
  window_height = 0.95;
end
  

fig = figure(1, "toolbar", "none","menubar","none", "units","normalized","Position", [window_xs window_ys window_width window_height]);
pos = get(gcf,'Position'); 
set(gcf, 'Position', [0.025,0.025, pos(3), pos(4)]); 
set(gcf,'name','STMicroelectronics Edukit Real Time Digital Motor Control Workbench','numbertitle','off')

p1 = subplot(4,1,2);
h1 = animatedline;
grid (p1, 'on');
box on;
axis([0,Window_Time,Pendulum_Angle_Y_Lower_Limit,Pendulum_Angle_Y_Upper_Limit]);
ylabel('Angle (degrees)');
ylabel('Pendulum Angle (degrees)','Color','k');
set(gca,'FontSize',Plot_Font_Size);

report_label_string = 'Rotor Angle (degrees)';

p2 = subplot(4,1,1);
h2 = animatedline;
grid (p2, 'on');
box on;
axis([0,Window_Time,Rotor_Angle_Y_Lower_Limit,Rotor_Angle_Y_Upper_Limit]);
ylabel(report_label_string,'Color','k');
set(gca,'FontSize',Plot_Font_Size);

p3 = subplot(4,1,3);
h3 = animatedline;
grid (p3, 'on');
box on;
axis([0,Window_Time,Rotor_Track_Angle_Y_Lower_Limit,Rotor_Track_Angle_Y_Upper_Limit]);
ylabel('Rotor Command (degrees)','Color','k');
set(gca,'FontSize',Plot_Font_Size);

p4 = subplot(4,1,4);
h4 = animatedline;
grid (p4, 'on');
box on;
axis([0,Window_Time,Rotor_Target_Angle_Y_Lower_Limit,Rotor_Target_Angle_Y_Upper_Limit]);
xlabel('Sample Time (sec)');
ylabel('Rotor Control (deg/s/s)','Color','k');
set(gca,'FontSize',Plot_Font_Size);

% Button Settings 
offset = 0.005;
offset_spacing = 0.0495;
button_height = 0.1;
button_width = 0.045;
button_vert_height = 0.8;
if isunix
button_font_size = 12;
end

if ispc
button_font_size = 10;
end

button_color_control_off = 'w';
button_color_control_on = 'g';
button_color_dec = 'w';
button_color_inc = 'g';

global p;
p = uipanel ("title", "Control System Configuration", "position", [.01 .01 .9 .06]);
global c1;
c1 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_off,"string","Exit",'callback',{@ExitMode_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c2;
c2 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_off,"string","Stop\nControl",'callback',{@TerminateMode_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c8;
c8 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_off,"string","Sine\nOn",'callback',{@Sine_Drive_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c9;
c9 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_off,"string","Sine\nOff",'callback',{@Sine_Drive_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c31;
c31 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nStep",'callback',{@Inc_Step_Size_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c40;
c40 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nStep",'callback',{@Dec_Step_Size_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c27;
c27 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nPend P",'callback',{@Inc_Pend_P_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c28;
c28 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nPend P",'callback',{@Dec_Pend_P_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c29;
c29 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nPend I",'callback',{@Inc_Pend_I_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c30;
c30 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nPend I",'callback',{@Dec_Pend_I_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c32;
c32 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nPend D",'callback',{@Inc_Pend_D_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c33;
c33 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nPend D",'callback',{@Dec_Pend_D_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c34;
c34 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nRotor P",'callback',{@Inc_Rotor_P_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c35;
c35 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nRotor P",'callback',{@Dec_Rotor_P_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c36;
c36 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nRotor I",'callback',{@Inc_Rotor_I_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c37;
c37 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nRotor I",'callback',{@Dec_Rotor_I_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;
global c38;
c38 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nRotor D",'callback',{@Inc_Rotor_D_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset + offset_spacing;
global c39;
c39 = uicontrol('parent',p,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nRotor D",'callback',{@Dec_Rotor_D_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
offset = offset +  offset_spacing;

global pv;
pv = uipanel ("title", "Motor Control", "position", [.92 .01 .07 .95]);

button_height = 0.01;
button_width = 0.9;
button_vert_height = 0.045;
offset_spacing = 0.047;
offset = 0.1;

global c11;
c11 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nTorq",'callback',{@Inc_Torq_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c10;
c10 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nTorq",'callback',{@Dec_Torq_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c13;
c13 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nMax Spd",'callback',{@Max_Speed_Inc_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c14;
c14 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nMax Spd",'callback',{@Max_Speed_Dec_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c15;
c15 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nMin Spd",'callback',{@Min_Speed_Inc_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c16;
c16 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nMin Spd",'callback',{@Min_Speed_Dec_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c17;
c17 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nAccel",'callback',{@Max_Accel_Inc_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c18;
c18 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nAccel",'callback',{@Max_Accel_Dec_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c19;
c19 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_inc,"string","Inc\nDecel",'callback',{@Max_Decel_Inc_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c20;
c20 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_dec,"string","Dec\nDecel",'callback',{@Max_Decel_Dec_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c21;
c21 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Sensitivity\nOn",'callback',{@Sens_Fnc_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c22;
c22 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_off,"string","Sensitivity\nOff",'callback',{@Sens_Fnc_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c23;
c23 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Noise Sens\nOn",'callback',{@Noise_Sens_Fcn_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c24;
c24 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_off,"string","Noise Sens\nOff",'callback',{@Noise_Sens_Fcn_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c25;
c25 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Load Dist\nSens On",'callback',{@Load_Disturbance_Sens_Fcn_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c26;
c26 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_off,"string","Load Dist\nSens Off",'callback',{@Load_Disturbance_Sens_Fcn_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c43
c43 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Pendulum\nImpulse On",'callback',{@Pendulum_Impulse_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c44
c44 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_off,"string","Pendulum\nImpulse Off",'callback',{@Pendulum_Impulse_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c41
c41 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Rotor Step\nOn",'callback',{@Rotor_Step_On_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c42
c42 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_off,"string","Rotor Step\nOff",'callback',{@Rotor_Step_Off_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;
global c40;
c40 = uicontrol('parent',pv,'Style','pushbutton', 'FontSize', button_font_size,'backgroundcolor',button_color_control_on,"string","Log Data",'callback',{@Log_Data_ButtonPushed}, 'Tag','Button_General', 'units', 'normalized', 'Position', [offset button_height button_width button_vert_height]);
button_height = button_height + offset_spacing;


win_x = 0.01;
win_y = 0.1;
win_width = 0.08;
win_height = 0.05;
win_offset = 0.06;
window_foreground = 'y';
window_background = 'k';
if isunix
win_font = 12;
end
if ispc
win_font = 10;
end

t0 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t1 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t2 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t3 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t4 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t5 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t6 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t7 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t8 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t9 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t10 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t11 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t12 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);
win_y = win_y + win_offset;
t13 = uicontrol(fig,'Style','text','BackgroundColor', window_background, 'ForegroundColor', window_foreground,'FontWeight', 'bold', 'HorizontalAlignment',...
'center','FontSize',win_font,'units','normalized','Position', [win_x win_y win_width win_height]);


%
% Character count for each data line transmitted by Edukit
%

Read_Length = 9;

%
% Number of read attempts prior to declaration of error
%

read_attempt_limit = 20;

% Disable speed governor

write(irip_serial_port,")\r");


% Disable high speed sampling

write(irip_serial_port,"y\r");


%
% Flush input and output serial buffers
%

flush(irip_serial_port);


% 
% Calibrate sample period
%

attempt_count = 10;

sample_period_average = 0;
i = 1;
while (i < 100)
ret = ReadToTermination(irip_serial_port);
nucleo_data_value = str2num(ret);
data_length = size(nucleo_data_value);
if data_length(2) ~= Read_Length;
  if data_length(2) == 0
    attempt_count = attempt_count + 1;
    if attempt_count > read_attempt_limit
      fclose(irip_serial_port);
      fclose all;
      close all;
      disp(sprintf(" Serial datastream terminated, existing..."));
      return;
    end
  end
  continue;
end
sample_period_average = sample_period_average + nucleo_data_value(1);
i = i + 1;
end

sample_period_average = sample_period_average/(i - 1);

torq_val = 0;
max_accel = 0;
max_decel = 0;
enable_disturbance_sensitivity_fcn = 0;
enable_noise_sensitivity_fcn = 0;
enable_rotor_step = 0;
step_size = 0;
enable_sensitivity_fnc = 0;
pend_p = 0;
pend_i = 0;
pend_d = 0;
rotor_p = 0;
rotor_i = 0;
rotor_d = 0;
max_speed = 0;
min_speed = 0;

%
% Sample_Period_Scale determines the scaling between reported cycle time 
% and sample time.  The Edukit reported cycle time is average sver 100 cycles.
% Thus, cycle period is nucleo_data_value(1)/100 in milliseconds
% 
%
% Tne Speed Governor reduces data rate by factor of Speed_Scale = 20
%

Speed_Scale = 20;
Sample_Period_Scale = Speed_Scale*(1/1000);
Sample_Period = sample_period_average * Sample_Period_Scale;

% Edukit system supplies Load Disturbance Sensitivty Function in Rotor Angle
% variable.  Load Disturbance is measured with a step stimulus of configurable
% amplitude.  Scaling of Load Disturbance is configured to value of 1
% for Suspended Mode and value of 5 for Inverted Mode.  This will be selected
% in the measurement loop below.

LOAD_DISTURBANCE_SENSITIVITY_SCALE = 1;

j = 1;
k = 1; 
m = 1;

start_tic = tic;
display_cycles = 0;
data_archive_state = 0;
sweep_count_status = 0;

% Enable speed governor for data display mode

write(irip_serial_port,"(\r");

average_cycle_time = 0;

clear nucleo_data_value;
Start_Index = 1;
Start_Index = 1;

%
% Introduce display start delay to permit data 
% acquisition prior to rendering.  This accounts for 
% data acquisition delay appearing in MacOSX Octave.
%

if ispc
Start_Delay = 0;
Close_State = 1;
end

if isunix
Start_Delay = 0;
mh = msgbox(sprintf("Please Stand By During System Start"),'replace');
Close_State = 0;
end

while (exit_state == 0) 

attempt_count = 0;
i = 0;

cycles = 0;

while ( exit_state == 0 )
  
  i = i + 1;
  
    ret = ReadToTermination(irip_serial_port);
    nucleo_data_value = str2num(ret);
    data_length = size(nucleo_data_value);
    if data_length(2) ~= Read_Length;
      if data_length(2) == 0
        attempt_count = attempt_count + 1;
        if attempt_count > read_attempt_limit
          fclose(irip_serial_port);
          fclose all;
          close all;
          disp(sprintf(" Serial datastream terminated, existing..."));
          return;
        end
      end
      continue;
    end
  
  
  %
  % Include scale factor adjustment for reporting of motor_position_target
  %  
  
  if (nucleo_data_value(1) == 2)
    motor_position_target(i) = nucleo_data_value(8)/Rotor_Angle_Read_Gain;
    if Pendulum_Impulse_Enable == 0
      rotor_command(i) = nucleo_data_value(7)/Rotor_Angle_Read_Gain;
    end
    if Pendulum_Impulse_Enable == 1
       rotor_command(i) = nucleo_data_value(7)/Pendulum_Angle_Read_Gain;
    end
    rotor_angle(i) = nucleo_data_value(5)/Rotor_Angle_Read_Gain;
    pendulum_angle(i) = nucleo_data_value(4)/Pendulum_Angle_Read_Gain;
    average_cycle_time = average_cycle_time + nucleo_data_value(2);
  end
  
  if data_archive_state == 0
    window_time(i) = (i-1)*Sample_Period;
    if window_time(i) > Window_Time 
      display_cycles = display_cycles + 1;
      sample_time_p = (i-1)*Sample_Period;
      pendulum_angle_p = pendulum_angle(i-1);
      rotor_angle_p = rotor_angle(i-1);
      motor_position_target_p = motor_position_target(i-1);
      rotor_command_p = rotor_command(i-1);
      break;
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
    if i > 1
      motor_position_target(i) = motor_position_target(i - 1);
      rotor_command(i) = rotor_command(i - 1);
      rotor_angle(i) = rotor_angle(i - 1);
      pendulum_angle(i) = pendulum_angle(i - 1);
      motor_position_target(i) = motor_position_target(i - 1);
    end
    if i == 1
      motor_position_target(i) = motor_position_target_p;
      rotor_command(i) = rotor_command_p;
      rotor_angle(i) = rotor_angle_p;
      pendulum_angle(i) = pendulum_angle_p;
      motor_position_target(i) = motor_position_target_p;
    end
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
    if i > 1
      rotor_command(i) = rotor_command(i - 1);
      rotor_angle(i) = rotor_angle(i - 1);
      pendulum_angle(i) = pendulum_angle(i - 1);
      motor_position_target(i) = motor_position_target(i - 1);
    end
    if i == 1
      motor_position_target(i) = motor_position_target_p;
      rotor_command(i) = rotor_command_p;
      rotor_angle(i) = rotor_angle_p;
      pendulum_angle(i) = pendulum_angle_p;
      motor_position_target(i) = motor_position_target_p;
    end
  end
  
  %Update screen at Screen_Update_Rate
  
  if isunix && cycles >= Start_Delay && Close_State == 0
   close(mh);
   Close_State = 1;
  end
  
  cycles = cycles + 1;
  
  if data_archive_state == 0 
    timer_tic = toc(start_tic);     
    if timer_tic > 1/Screen_Update_Rate  
      if i > Start_Index
        addpoints(h1, window_time(i-1), pendulum_angle(i-1));
        addpoints(h2, window_time(i-1), rotor_angle(i-1));
        addpoints(h3, window_time(i-1), rotor_command(i-1));
        addpoints(h4, window_time(i-1), motor_position_target(i-1));
      end
      drawnow;                    %
      start_tic = tic;            
    end
  end
  
  if exit_state == 1
    break;
  end
  
  % Plot autoscaling system 
  
  if data_archive_state == 0 
    update_state = 0; 
    if mod((i - 1), 100) == 0 && i > 100
      if i > 100
        Max_Rotor_Angle = max(abs(rotor_angle(1:i)));
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
          Rotor_Angle_Range = Rotor_Angle_Range_Max * 10;
        end
        update_state = 1;
      end
      Max_Rotor_Angle = max(abs(rotor_angle(1:i)));
      if update_state == 0
        if Max_Rotor_Angle < 0.1*Rotor_Angle_Range_Max;
          Rotor_Angle_Range = Rotor_Angle_Range_Upper;
          update_state = 1;
        end
        if Max_Rotor_Angle < 0.8*Rotor_Angle_Range_Upper;
          Rotor_Angle_Range = Rotor_Angle_Range_Upper;
          update_state = 1;
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
      end
      
      update_state_track = 0;
      if mod((i - 1), 100) == 0 && i > 100
        Max_Rotor_Track_Angle = max(abs(rotor_command(1:i)));
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
      
      if update_state == 1
        Rotor_Angle_Y_Upper_Limit = Rotor_Angle_Range/2;
        Rotor_Angle_Y_Lower_Limit = -Rotor_Angle_Y_Upper_Limit;
        set(p2, 'Ylim',[Rotor_Angle_Y_Lower_Limit,Rotor_Angle_Y_Upper_Limit]);
        update_state = 0;
      end
      
      if update_state_track == 1
        Rotor_Track_Angle_Y_Upper_Limit = Rotor_Track_Angle_Range/2;
        Rotor_Track_Angle_Y_Lower_Limit = -Rotor_Track_Angle_Y_Upper_Limit;
        set(p3, 'Ylim',[Rotor_Track_Angle_Y_Lower_Limit,Rotor_Track_Angle_Y_Upper_Limit]);
        update_state_track = 0;
      end      
    end
    
    if mod(i,20) == 0 && exit_state == 0 
      
      String = sprintf("Step Size\n%.1f", step_size); set (t0, 'string', String);
      String = sprintf("Max Speed\n%i step/s", max_speed); set (t1, 'string', String);
      String = sprintf("Min Speed\n%i step/s", min_speed); set (t2, 'string', String);
      String = sprintf("Max Decel\n%i step/s/s", max_decel); set (t3, 'string', String);
      String = sprintf("Max Accel\n%i step/s/s", max_accel); set (t4, 'string', String);
      
      
      String = sprintf("Torq Current\n%i mA", torq_val); set (t5, 'string', String);
      String = sprintf("Rotor PID\nD Gain %.1f", rotor_d); set (t6, 'string', String);
      String = sprintf("Rotor PID\nI Gain %.1f", rotor_i); set (t7, 'string', String);
      String = sprintf("Rotor PID\nP Gain %.1f", rotor_p); set (t8, 'string', String);           
      String = sprintf("Pend PID\nP Gain %.1f", pend_p); set (t11, 'string', String);
      String = sprintf("Pend PID\nI Gain %.1f", pend_i); set (t10, 'string', String);
      String = sprintf("Pend PID\nD Gain %.1f", pend_d); set (t9, 'string', String);
      
      if enable_rotor_step == 1 
        String = sprintf("Rotor Step\nOn"); set (t12, 'string', String); 
      end
      if enable_rotor_step == 0 
        String = sprintf("Rotor Step\nOff"); set (t12, 'string', String); 
      end
      
      if enable_sensitivity_fnc == 1
        String = sprintf("Sensitivity\nFunction On"); set (t13, 'string', String);        
        set(p2, 'ylabel', 'Sensitivity Function');
      end
      
      if enable_noise_sensitivity_fcn == 1
        String = sprintf("Noise Sens\nFunction On"); set (t13, 'string', String); 
        set(p2, 'ylabel', 'Noise Sensitivity');
      end
      
      if enable_disturbance_sensitivity_fcn == 1
        String = sprintf("Disturbance Sens\nFunction On"); set (t13, 'string', String); 
        set(p2, 'ylabel', 'Disturbance Sensitivity');
      end
      
     if Pendulum_Impulse_Enable == 1
        String = sprintf("Pendulum\nImpulse On"); set (t13, 'string', String); 
        set(p2, 'ylabel', 'Rotor Angle (degrees)');
        set(p3, 'ylabel', 'Pendulum Impulse (degrees)');
      end
      
      if enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0 && Pendulum_Impulse_Enable == 0
        String = sprintf("Comp Sens\nFunction On"); set (t13, 'string', String); 
        set(p2, 'ylabel', 'Rotor Angle (degrees)');
        set(p3, 'ylabel', 'Rotor Command (degrees)');
      end
    end
  end
  
  if enable_data_archive == 1 && sweep_count_status == 0
    prompt = {'Enter Sweep Count (Minimum of 4 for Sensitivity Functions)'};
    dlgtitle = 'Input';
    dims = [1 40];
    definput = {'4'};
    sweep_entry = inputdlg(prompt,dlgtitle,dims,definput);
    Sweep_Count_Threshold = str2num(cell2mat(sweep_entry(1)));
    Sample_Count_Threshold = round(Sweep_Count_Threshold*40/(Sample_Period/Speed_Scale));
    sweep_count_status = 1;
    close sweep_entry;
    disp(sprintf("Logging of %i Samples Will Start\nData Display Will Be Suspended During Logging", Sample_Count_Threshold));
    mh = msgbox(sprintf("   Logging of %i Samples Will Start\nData Display Suspended During Logging",  Sample_Count_Threshold), 'replace');
  end
end

if exit_state == 0 && data_archive_state == 0
  clearpoints(h1);
  clearpoints(h2);
  clearpoints(h3);
  clearpoints(h4);
  drawnow;
end
if data_archive_state == 1
  sample_sweep_count = sample_sweep_count + 1;
end

% Enable high speed sampling 
if enable_data_archive == 1 && data_archive_state == 0
  data_archive_state = 1;
end
k = k + 1;
if data_archive_state == 1
  break;
end

end

% End of Real Time Display Loop
%
% Start of Data Logging Loop operating at high sample rate

close all;

% Configure high speed sampling and Sample_Period

if data_archive_state == 1
%Disable Speed Governor
write(irip_serial_port,")\r");
%Enable High Speed Sampling
write(irip_serial_port,"Y\r");
Read_Length = 5;
Sample_Period = Sample_Period / Speed_Scale;
end

if data_archive_state == 1  
step_start = 0;

%
% Measure step signal and align start of record for Sensitivity Function 
% Frequency Response computation
%

if enable_rotor_step || enable_disturbance_sensitivity_fcn || enable_noise_sensitivity_fcn || enable_sensitivity_fnc
  m = 1;
  while (1)
    
    ret = ReadToTermination(irip_serial_port);
    nucleo_data_value = str2num(ret);
    data_length = size(nucleo_data_value);
    if data_length(2) ~= Read_Length;
      if data_length(2) == 0
        attempt_count = attempt_count + 1;
        if attempt_count > read_attempt_limit
          fclose(irip_serial_port);
          fclose all;
          close all;
          disp(sprintf(" Serial datastream terminated, existing..."));
          return;
        end
      end
      continue;
    end
    
    
    rotor_command(m) = nucleo_data_value(3)/Rotor_Angle_Read_Gain;
    
    if m > 1
      if rotor_command(m - 1 ) < 0 && rotor_command(m) > 0
        step_start = 1;
      endif
    endif
    m = m + 1;
    if m - step_start > 1000
      break;
    end
  end
end
end

m = 1;
if data_archive_state == 1  
while (1);
  
  ret = ReadToTermination(irip_serial_port);
  nucleo_data_value = str2num(ret);
  data_length = size(nucleo_data_value);
  if data_length(2) ~= Read_Length;
    if data_length(2) == 0
      attempt_count = attempt_count + 1;
      if attempt_count > read_attempt_limit
        fclose(irip_serial_port);
        fclose all;
        close all;
        disp(sprintf(" Serial datastream terminated, existing..."));
        return;
      end
    end
    continue;
  end
  
  motor_position_target(m) = nucleo_data_value(4)/Rotor_Angle_Read_Gain;
  rotor_command(m) = nucleo_data_value(5)/Rotor_Angle_Read_Gain;
  rotor_angle(m) = nucleo_data_value(3)/Rotor_Angle_Read_Gain;
  pendulum_angle(m) = nucleo_data_value(2)/Pendulum_Angle_Read_Gain;
  
  sample_time(m) = (m - 1)*Sample_Period;
  if m > Sample_Count_Threshold
    break;
  end
  
  m = m + 1;
  
  if mod(m,5000) == 0 && m < Sample_Count_Threshold
    disp(sprintf("%i Samples of %i Completed\nData Display Will Be Suspended During Logging", m, Sample_Count_Threshold));
    mh = msgbox(sprintf("      %i Samples of %i Completed\nData Display Will Be Suspended During Logging", m, Sample_Count_Threshold), 'replace');
  end
  
  if m == Sample_Count_Threshold
    close(mh);
    disp(sprintf("Logging of %i Samples Completed", Sample_Count_Threshold ));
    mh = msgbox(sprintf("      Logging of %i Samples Completed", Sample_Count_Threshold));
  end
end

if enable_sensitivity_fnc == 1
  data_display = 'Sensitivity Function';
  %Edukit system supplies Sensitivty Function in Rotor Angle variable
end
if enable_noise_sensitivity_fcn == 1
  data_display = 'Noise Sensitivity Function';
  % Edukit system supplies Noise Sensitivty Function in Rotor Angle variable
end
if enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0
  data_display = 'Step Response Rotor Angle (degrees)';
end
if enable_disturbance_sensitivity_fcn == 1
  data_display = 'Disturbance Sensitivity Function';
  % Edukit system supplies Load Disturbance Sensitivty Function in Rotor Angle
  % variable.  Load Disturbance is measured with a step stimulus of configurable
  % amplitude.  Scaling of Load Disturbance is configured to value of 1
  % for Suspended Mode and value of 5 for Inverted Mode. 
  if rotor_p <= 0 || pend_p <= 0
      LOAD_DISTURBANCE_SENSITIVITY_SCALE = 1;
  end
  rotor_angle = (rotor_angle)/LOAD_DISTURBANCE_SENSITIVITY_SCALE;
end

if Pendulum_Impulse_Enable == 1 && enable_noise_sensitivity_fcn == 0 && enable_sensitivity_fnc == 0 && enable_disturbance_sensitivity_fcn == 0
        data_display = 'Pendulum Angle Impulse (degrees)';
end

if enable_data_archive == 1
  save_filename = sprintf('Edukit_Octave_%s_%s.mat', data_display, datestr(now,'_mm-dd-yyyy_HH-MM'));
  selpath = [selpath,'/'];
  save_filename = [selpath,save_filename];
  save(save_filename, "data_display", "sample_time","motor_position_target", "rotor_command", "rotor_angle", "pendulum_angle", "Sample_Period", "max_speed", "min_speed", "max_decel", "max_accel", "torq_val", "pend_p", "pend_i", "pend_d", "rotor_p", "rotor_i", "rotor_d", "Pendulum_Impulse_Enable");
end

end

% Disable speed governor and ensure standard speed sampling configured

write(irip_serial_port,")\r");

write(irip_serial_port,"y\r");

close all;
fclose all;

