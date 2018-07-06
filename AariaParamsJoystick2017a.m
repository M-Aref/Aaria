%% AariaParams v.1.1 31.5.2018
%1. Run AariaUI2.m to open GUI
%2. Set DH-parameters manually or select one of the presets and simulate
%3. Stop simulation by clicking the stop simulation button.
%4. Simulation data will appear in the workspace after a few seconds
%5. Run SaveF.m to save simulation data to a file with a timestamp name

% % Simulation parameters
AARIAENV.JoystickControl = 1; %Enable joystick control, 0 for random 0scillation
VSS_Joystick = 1; % 1 = control with joystick, 2 = replay previous control
AARIAENV.InputMode = 2;% 1 = random DH table, 2 = custom table
 

AARIAENV.Source_TS = 0.01;%sample time for source blocks
AARIAENV.Sensor_TS = 0.1;%sample time for sensor blocks
AARIAENV.Min_TS = min([AARIAENV.Source_TS, AARIAENV.Sensor_TS]);
AARIAENV.gravity = [0 0 -9.80665];
%--------------------------------------------------------------------------
%% Manual stop
%set_param('AariaJoystickR2017a','SimulationCommand','stop');

%{
%--------------------------------------------------------------------------
model = 'Aaria';
set_param(model,'SimulationMode','accelerator')
set_param(model,'SimMechanicsOpenEditorOnUpdate','off')

load_system(model);
numSims = 4;

simIn(1:numSims) = Simulink.SimulationInput(model);

for idx = 1:numSims
%--------------------------------------------------------------------------
%}
%% INFO

%COLLECTABLE DATA:
%Sensor data:                   SensorData
%DH parameters and link types:  DHparameters
%Joint trajectories:            JointOscillationParameters
%Base trajectories:             BaseOscillationParameters
%Sensor noise seeds             SensorNoiseSeeds

%SensorData table column guide:
%1 time, 
%2-7 joint values[radians or meters]
%8-13 base sensor,  [xyz angular velocity, xyz acceleration]
%14-19 tool sensor, [xyz angular velocity, xyz acceleration]
%20-31 link 1 sensors [xyz angular velocity, xyz acceleration] X 2
%32-43, 44-55, 56-67, 68-79, 80-91
%each link to 91 when using 2 sensor for each link

%DHparameters table column guide:
%[alpha, a, theta, d, linktype]
%   Note: For tool, a is tool length and d is tool radius

%JointOscillationParameters table column guide:
%[amplitude, prismaticAmplitude, bias, freq, phase, torqueAmplitude, torquePrismaticAmplitude, torqueBias, torqueFreq, torquePhase]
%revolute joints use amplitude value, prismatic joints use the prismatic amplitude value

%BaseOscillationParameters table guide:
%Base sine wave parameters columns [translation, rotation]
%columns [x(amplitude, bias, freq, phase),y(ampl...,z..., x(torqueAmplitude, torqueBias, torqueFreq, torquepPhase),y...,z...]

model = 'AariaJoystickR2017a';
simIn(1) = Simulink.SimulationInput(model);
%open('AariaJoystickR2017a.slx')
set_param('AariaJoystickR2017a','SimulationMode','normal')
set_param('AariaJoystickR2017a','SimMechanicsOpenEditorOnUpdate','on')
set_param('AariaJoystickR2017a','StopTime','inf')
 
 %DH TABLE CUSTOM
 %{
 %PUMA 560     %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
 AARIAMAN.Joint1 =   [0          0           0           1.5             1];
 AARIAMAN.Joint2 =   [-pi/2      0           0           0.3             1];
 AARIAMAN.Joint3 =   [0          1           0           -0.3            1];
 AARIAMAN.Joint4 =   [-pi/2      0.1         0           1               1];
 AARIAMAN.Joint5 =   [pi/2       0           0           0               1];
 AARIAMAN.Joint6 =   [-pi/2      0           0           0               1];
%}
 
%{
% % Simulation parameters
AARIAENV.JoystickControl = 1; %Enable joystick control, 0 for random 0scillation
VSS_Joystick = 1; % 1 = control with joystick, 2 = replay previous control
AARIAENV.InputMode = 2;% 1 = random DH table, 2 = custom table
 

AARIAENV.Source_TS = 0.02;%sample time for source blocks
AARIAENV.Sensor_TS = 0.02;%sample time for sensor blocks
AARIAENV.gravity = [0 0 -9.80665];
%}
% % Randomization Parameters
%vec_rand_ab_res inputs: [lower limit, upper limit, resolution];
%Limits included

%DH randomization parameters
AARIARAN.LinkRot =    [-pi/2,pi/2,pi/2];    %manipulator joint angles
AARIARAN.LinkLength = [0,1.0,0.01];      %manipulator link lengths
AARIARAN.LinkOffset = [0,1.0,0.01];      %manipulator offset and prismatic joint travel lengths
AARIARAN.ToolLength = [0,0.4,0.01];      %Tool length
AARIARAN.ToolWidth =  [0.01,0.1,0.01];    %Tool width

%Movement
AARIARAN.BaseOs =    [0.5,1,0.01];      %cartesian base oscillation
AARIARAN.BaseRot =   [0.5,1,0.01];     %base rotation
AARIARAN.JointOs =   [0.5,1,0.01];     %joint oscillation
AARIARAN.TorqueOs =  [1,100,1];       %Torque joint oscillation
AARIARAN.JointBias = [-0.3,0.3,0.01];

%VSS randomization parameters
%Parameters for randomizing the joint type. First link should not be
%allowed to be empty type (0) to avoid 0 DOF manipulators.
%0 = EMPTY, 1 = REVOLUTE, 2 = PRISMATIC, 3 = REVOLUTE TORQUE, 4 = PRISMATIC FORCE
%Base 0 = EMPTY, 1 = Motion control, 2 = Torque/force control

AARIARAN.VSS =       [0,1,2];
AARIARAN.VSSfirst =  [1,2];
AARIARAN.VSSbase =   [0,1];
 
%Joint motion
AARIARAN.amp =       AARIARAN.JointOs; %amplitude of revolute joint oscillation [rad]
AARIARAN.prismamp =  AARIARAN.JointOs; %amplitude of prismatic joint oscillation [m]
AARIARAN.bias =      AARIARAN.JointBias;    %starting point of joint oscillation [rad][m]
AARIARAN.freq =      AARIARAN.JointOs; %oscillation frequency [Hz]
AARIARAN.phase =     AARIARAN.JointOs; %phase shift of oscillations [rad]

%Joint torque
AARIARAN.tamp =      AARIARAN.TorqueOs;  %amplitude of revolute joint oscillation [Nm]
AARIARAN.tprismamp = AARIARAN.TorqueOs;  %amplitude of prismatic joint oscillation [F]
AARIARAN.tbias =     AARIARAN.JointBias;   %starting point of joint oscillation [rad][m]
AARIARAN.tfreq =     AARIARAN.JointOs;%oscillation frequency [Hz]
AARIARAN.tphase =    AARIARAN.JointOs;%phase shift of oscillations [rad]

%Base motion
AARIARAN.transamp  = AARIARAN.BaseOs;   % (m) amplitude of translational oscillations in x y z
AARIARAN.transbias = AARIARAN.BaseOs;   % (m)starting point of translational oscillations in x y z
AARIARAN.transfreq = AARIARAN.BaseOs;   % (rad/s) frequency of translational oscillations in x y z
AARIARAN.transphase= AARIARAN.BaseOs;   % (rad) any phase shift of translational oscillations in x y z

AARIARAN.rotamp  =   AARIARAN.BaseRot;   % amplitude of rotational oscillations about x y z
AARIARAN.rotbias =   AARIARAN.BaseRot;   % starting point of rotational oscillations in x y z
AARIARAN.rotfreq =   AARIARAN.BaseRot;   % (rad/s) frequency of rotational oscillations in x y z
AARIARAN.rotphase=   AARIARAN.BaseRot;   % (rad) any phase shift of rotational oscillations in x y z

%Base torque
AARIARAN.transtamp  = AARIARAN.BaseOs;   % (m) amplitude of translational oscillations in x y z
AARIARAN.transtbias = AARIARAN.BaseOs;   % (m)starting point of translational oscillations in x y z
AARIARAN.transtfreq = AARIARAN.BaseOs;   % (rad/s) frequency of translational oscillations in x y z
AARIARAN.transtphase= AARIARAN.BaseOs;   % (rad) any phase shift of translational oscillations in x y z

AARIARAN.rottamp   = AARIARAN.BaseRot;   % amplitude of rotational oscillations about x y z
AARIARAN.rottbias  = AARIARAN.BaseRot;   % starting point of rotational oscillations in x y z
AARIARAN.rottfreq  = AARIARAN.BaseRot;   % (rad/s) frequency of rotational oscillations in x y z
AARIARAN.rottphase = AARIARAN.BaseRot;   % (rad) any phase shift of rotational oscillations in x y z

% % DH Parameters
%DO NOT CHANGE THIS VALUE
AARIAMAN.linknumber = 8*1.0; %Number of links for parameter assignment loops
%Modular joints 1-6, Tool 7, Base 8
%Change VSS_MODE randomization parameters to remove links
%Adding links is not supported

%Loop for setting DH parameters for links
switch (AARIAENV.InputMode)
    case 1
for nLL=1:AARIAMAN.linknumber-2
    AARIAMAN.DH(nLL).alpha =  vec_rand_ab_res(AARIARAN.LinkRot);
    AARIAMAN.DH(nLL).a =      vec_rand_ab_res(AARIARAN.LinkLength);
    AARIAMAN.DH(nLL).theta =  vec_rand_ab_res(AARIARAN.LinkRot); 
    AARIAMAN.DH(nLL).d =      vec_rand_ab_res(AARIARAN.LinkOffset);
end
    AARIAMAN.DH(7).alpha =  0;
    AARIAMAN.DH(7).a =      vec_rand_ab_res(AARIARAN.ToolLength);
    AARIAMAN.DH(7).theta =  0; 
    AARIAMAN.DH(7).d =      vec_rand_ab_res(AARIARAN.ToolWidth);

    AARIAMAN.DH(8).alpha =  0;
    AARIAMAN.DH(8).a =      0;
    AARIAMAN.DH(8).theta =  0; 
    AARIAMAN.DH(8).d =      0;
    
     case 2
        AARIAMAN.Joint7 =   [0      0.2         0        0.1           1];
        AARIAMAN.Joint8 =   [0      0           0        0             0];
        AARIAMAN.Joints = [AARIAMAN.Joint1;AARIAMAN.Joint2;AARIAMAN.Joint3;AARIAMAN.Joint4;AARIAMAN.Joint5;AARIAMAN.Joint6;AARIAMAN.Joint7;AARIAMAN.Joint8];
        AARIAMAN.Joints = vec2mat(AARIAMAN.Joints,5);
        for nL=1:AARIAMAN.linknumber
            AARIAMAN.DH(nL).alpha =      AARIAMAN.Joints(nL,1);
            AARIAMAN.DH(nL).a =          AARIAMAN.Joints(nL,2);
            AARIAMAN.DH(nL).theta =      AARIAMAN.Joints(nL,3); 
            AARIAMAN.DH(nL).d =          AARIAMAN.Joints(nL,4);
            AARIAMAN.DH(nL).linktype =   AARIAMAN.Joints(nL,5);
        end
end
    
for nLL=1:AARIAMAN.linknumber %changes negative link lengths to positive
    if AARIAMAN.DH(nLL).a < 0
        AARIAMAN.DH(nLL).a = abs(AARIAMAN.DH(nLL).a);
        disp(['Negative DH-parameter "a" value in position ', num2str(nLL), '! Value changed to absolute.'])
    end
end

% % Variant subsystems

%Joint types are randomly selected and assigned here
%0 = EMPTY, 1 = REVOLUTE MOTION, 2 = PRISMATIC MOTION
%3 = REVOLUTE TORQUE, 4 = PRISMATIC FORCE
%Base 0 = EMPTY, 1 = MOTION, 2 = TORQUE/FORCE

%First link is never empty to avoid 0 DOF manipulators
%datasample picks a random number from the given vector "help datasample"
switch (AARIAENV.InputMode)
    case 1
        VSS_MODE1 = datasample(AARIARAN.VSSfirst,1);    % revolute or prismatic
        VSS_MODE2 = datasample(AARIARAN.VSS,1);  % no joint, revolute or prismatic
        VSS_MODE3 = datasample(AARIARAN.VSS,1);  % no joint, revolute or prismatic
        VSS_MODE4 = datasample(AARIARAN.VSS,1);  % no joint, revolute or prismatic
        VSS_MODE5 = datasample(AARIARAN.VSS,1);  % no joint, revolute or prismatic
        VSS_MODE6 = datasample(AARIARAN.VSS,1);  % no joint, revolute or prismatic
        VSS_MODE7 = 1;
        VSS_MODE8 = 0;%datasample(AARIARAN.VSSbase,1);  % stationary, motion, torque base

    case 2
        VSS_MODE1 = AARIAMAN.DH(1).linktype; % set variant subsystems based on input data
        VSS_MODE2 = AARIAMAN.DH(2).linktype;
        VSS_MODE3 = AARIAMAN.DH(3).linktype;
        VSS_MODE4 = AARIAMAN.DH(4).linktype;
        VSS_MODE5 = AARIAMAN.DH(5).linktype;
        VSS_MODE6 = AARIAMAN.DH(6).linktype;   
        VSS_MODE7 = AARIAMAN.DH(6).linktype; 
        VSS_MODE8 = AARIAMAN.DH(8).linktype;  % stationary, motion, torque base
end

idx = 1;     
%workaround for chaning variant types during simulation   by using the override option  
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint1/JointVSS','OverrideUsingVariant',['VSS_MODE1 == ' num2str(VSS_MODE1)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint2/JointVSS','OverrideUsingVariant',['VSS_MODE2 == ' num2str(VSS_MODE2)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint3/JointVSS','OverrideUsingVariant',['VSS_MODE3 == ' num2str(VSS_MODE3)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint4/JointVSS','OverrideUsingVariant',['VSS_MODE4 == ' num2str(VSS_MODE4)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint5/JointVSS','OverrideUsingVariant',['VSS_MODE5 == ' num2str(VSS_MODE5)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Joint6/JointVSS','OverrideUsingVariant',['VSS_MODE6 == ' num2str(VSS_MODE6)]);
simIn(idx) = simIn(idx).setBlockParameter('AariaJoystickR2017a/Base/BaseVSS','OverrideUsingVariant',['VSS_MODE8 == ' num2str(VSS_MODE8)]);
          
%Saving manipulator configuration to DH table
AARIAMAN.DH(1).linktype = VSS_MODE1;
AARIAMAN.DH(2).linktype = VSS_MODE2;
AARIAMAN.DH(3).linktype = VSS_MODE3;
AARIAMAN.DH(4).linktype = VSS_MODE4;
AARIAMAN.DH(5).linktype = VSS_MODE5;
AARIAMAN.DH(6).linktype = VSS_MODE6;
AARIAMAN.DH(7).linktype = VSS_MODE7;
AARIAMAN.DH(8).linktype = VSS_MODE8;


% % Structure and Visualization
AARIAMAN.vis.link_width_x = 0.2;     %link width x [m]
AARIAMAN.vis.link_width_y = 0.2;     %link width y [m]
AARIAMAN.vis.link_width_r = 0.05;    %link radius [m]

AARIAMAN.vis.joint_size_r = AARIAMAN.vis.link_width_x*(0.5)*1.1;%joint width
AARIAMAN.vis.joint_size_l = AARIAMAN.vis.link_width_y*1.1;%joint length
AARIAMAN.vis.joint_mass = 1;%joint mass
AARIAMAN.vis.joint_density = 1000; %kg/m^3
AARIAMAN.vis.link_density = AARIAMAN.vis.joint_density; %kg/m^3
AARIAMAN.vis.rod_density = AARIAMAN.vis.joint_density; %kg/m^3

AARIAMAN.vis.coord_size = 20;%coordinate size in pixels
AARIAMAN.vis.coord_sizeS = 1;%Sensor coordinate size
AARIAMAN.vis.coord_sizeDH = AARIAMAN.vis.coord_size;%DH coordinate size

AARIAMAN.vis.col_j = [0.0 1.0 0.6]; %RGB color of Joints visualization
AARIAMAN.vis.col_l = [0.0 0.0 1.0]; %RGB color of Links
AARIAMAN.vis.col_s = [1.0 0.0 1.0]; %RGB color of sensor frames
AARIAMAN.vis.col_DH = [1.0 0.4 0.4];%RGB color of DH frames

AARIAMAN.vis.opacity_s = 0.0; %sensor frame opacity
AARIAMAN.vis.opacity_DH = 1.0; %DH frame opacity

%Loop for assigning DH parameters to REVOLUTE and PRISMATIC links
for nLL =1:AARIAMAN.linknumber-1
    AARIAMAN.L(nLL).alpha =  eul2rotm([0 0 AARIAMAN.DH(nLL).alpha]); %DH alpha rotation matrix 
    AARIAMAN.L(nLL).a =      [0 0 (AARIAMAN.DH(nLL).a)];             %DH a cartesian translation [m]
    AARIAMAN.L(nLL).theta =  eul2rotm([AARIAMAN.DH(nLL).theta 0 0]); %DH theta rotation matrix
    AARIAMAN.L(nLL).d =      [0 0 (AARIAMAN.DH(nLL).d)];             %DH d cartesian translation [m]
    
    AARIAMAN.L(nLL).cube = [AARIAMAN.vis.link_width_x AARIAMAN.vis.link_width_y abs(AARIAMAN.DH(nLL).a) + 0.001];       %Link dimensions [m]
    AARIAMAN.L(nLL).rod =  [AARIAMAN.vis.link_width_x/2 AARIAMAN.vis.link_width_y/2 abs(AARIAMAN.DH(nLL).d*2) + 0.001]; %Prismatic link dimensions [m]
    AARIAMAN.L(nLL).connector = abs(AARIAMAN.DH(nLL).d)+0.001;                                                %DH d offset
    
    AARIAMAN.L(nLL).massCon = pi*AARIAMAN.vis.link_width_r^2*abs(AARIAMAN.DH(nLL).d)*AARIAMAN.vis.link_density;                                    %Link mass
    AARIAMAN.L(nLL).massLink = AARIAMAN.vis.link_width_x*AARIAMAN.vis.link_width_y*abs(AARIAMAN.DH(nLL).a)*AARIAMAN.vis.link_density;
    AARIAMAN.L(nLL).massRod = AARIAMAN.vis.link_width_x/2*AARIAMAN.vis.link_width_y/2*abs(AARIAMAN.DH(nLL).d)*2*AARIAMAN.vis.rod_density;
    
    %Makes geometry invisible if it too small
    if AARIAMAN.DH(nLL).a < 0.01
        AARIAMAN.L(nLL).linkOpacity = 0.0;
    else
        AARIAMAN.L(nLL).linkOpacity = 1.0;
    end
        
    if AARIAMAN.DH(nLL).d < 0.01
        AARIAMAN.L(nLL).rodOpacity = 0.0;
    else
        AARIAMAN.L(nLL).rodOpacity = 1.0;
    end
    
    AARIAMAN.L(nLL).toDH = eul2rotm([ 0 -pi/2 pi/2],'ZYX'); %Rotation to DH-coordinate system
    AARIAMAN.L(nLL).ang_ini = 0; %Initial joint angle
    AARIAMAN.L(nLL).pos_ini = 0; %Initial prismatic joint position 
end

AARIAMAN.L(7).massLink = pi*AARIAMAN.DH(7).d^2*AARIAMAN.DH(7).a*AARIAMAN.vis.link_density; %Tool mass

% % Sensor Placement and noise
AARIASEN.ErrorGain = 0; % set to 0 to disable sensor error

%Gyroscope
%6 deg / h in-run bias stability
AARIASEN.gBias = 6.25/(3600);%degrees per second
AARIASEN.gInitBias = 0;      %Starting value of the slope
AARIASEN.gMu = 0;            %Normal distribution mean
AARIASEN.gSigma = 0.01;      %0.5; %Normal distribution deviation
AARIASEN.gWalk = 1e-4;       %5.4167e-06;% maximum and (negative)minimum uniform random number per second
%random walk per second 0.3 deg/ sqrt(h) angular random walk
AARIASEN.gNormalGain = 0.7;  %Gain for normally distributed noise

%Accelerometer
AARIASEN.aBias = 32e-6*9.81/(3600);  %g per second
AARIASEN.aInitBias = 0;              %Starting value of the slope
AARIASEN.aMu = 0;                    %Normal distribution mean
AARIASEN.aSigma = 0.01;              %Normal distribution deviation
AARIASEN.aWalk = 1e-4;               %maximum and (negative)minimum uniform random number per second 0.023 m/s/h
AARIASEN.aNormalGain = 0.004;        %Gain for normally distributed noise

AARIASEN.rand = floor(rand(8,12)*10000); %Generates an array of random numbers for sensor noise seeds

%Loop for placing 2 sensors in each modular link
for nLL = 1:6 % to repeat sensor placement for each set of 4 sensors on each link (body)
    nSS = 1; % only for the following settings
    AARIAMAN.L(nLL).S(nSS).pose_p = [AARIAMAN.DH(nLL).a/1.1 AARIAMAN.vis.link_width_x/2 AARIAMAN.vis.link_width_y/2];% origin is in the middle of the link
    AARIAMAN.L(nLL).S(nSS).pose_r = eul2rotm([0 0 0],'ZYX');
    
    nSS = 2; % only for the following settings
    AARIAMAN.L(nLL).S(nSS).pose_p = [AARIAMAN.DH(nLL).a/2 -AARIAMAN.vis.link_width_x/2 -AARIAMAN.vis.link_width_y/2];%origin is in the middle of the link
    AARIAMAN.L(nLL).S(nSS).pose_r = eul2rotm([0 0 0],'ZYX'); 
end

%Tool sensor
AARIAMAN.L(7).S(1).pose_p = [0 0 0];% with respect to its parent body
AARIAMAN.L(7).S(1).pose_r = eul2rotm([0 0 0],'ZYX');

%Base sensor parameters
AARIAMAN.L(8).S(1).pose_p = [0 0 0];% with respect to its parent body
AARIAMAN.L(8).S(1).pose_r = eul2rotm([0 0 0],'ZYX');

% % Base Oscillation
%motion
AARIAMAN.B.trans.ampx  =     vec_rand_ab_res(AARIARAN.transamp);     % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.biasx =     vec_rand_ab_res(AARIARAN.transbias);    % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.freqx =     vec_rand_ab_res(AARIARAN.transfreq);    % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.phasex =    vec_rand_ab_res(AARIARAN.transphase);   % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.ampx  =       vec_rand_ab_res(AARIARAN.rotamp);       % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.biasx =       vec_rand_ab_res(AARIARAN.rotbias);      % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.freqx =       vec_rand_ab_res(AARIARAN.rotfreq);      % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.phasex =      vec_rand_ab_res(AARIARAN.rotphase);     % (rad) any phase shift of rotational oscillations in x y z

AARIAMAN.B.trans.ampy  =     vec_rand_ab_res(AARIARAN.transamp);     % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.biasy =     vec_rand_ab_res(AARIARAN.transbias);    % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.freqy =     vec_rand_ab_res(AARIARAN.transfreq);    % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.phasey =    vec_rand_ab_res(AARIARAN.transphase);   % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.ampy  =       vec_rand_ab_res(AARIARAN.rotamp);       % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.biasy =       vec_rand_ab_res(AARIARAN.rotbias);      % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.freqy =       vec_rand_ab_res(AARIARAN.rotfreq);      % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.phasey =      vec_rand_ab_res(AARIARAN.rotphase);     % (rad) any phase shift of rotational oscillations in x y z

AARIAMAN.B.trans.ampz  =     vec_rand_ab_res(AARIARAN.transamp);     % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.biasz =     vec_rand_ab_res(AARIARAN.transbias);    % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.freqz =     vec_rand_ab_res(AARIARAN.transfreq);    % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.phasez =    vec_rand_ab_res(AARIARAN.transphase);   % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.ampz  =       vec_rand_ab_res(AARIARAN.rotamp);       % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.biasz =       vec_rand_ab_res(AARIARAN.rotbias);      % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.freqz =       vec_rand_ab_res(AARIARAN.rotfreq);      % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.phasez =      vec_rand_ab_res(AARIARAN.rotphase);     % (rad) any phase shift of rotational oscillations in x y z

%torque
AARIAMAN.B.trans.tampx  =     vec_rand_ab_res(AARIARAN.transtamp);   % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.tbiasx =     vec_rand_ab_res(AARIARAN.transtbias);  % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.tfreqx =     vec_rand_ab_res(AARIARAN.transtfreq);  % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.tphasex =    vec_rand_ab_res(AARIARAN.transtphase); % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.tampx  =       vec_rand_ab_res(AARIARAN.rottamp);     % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.tbiasx =       vec_rand_ab_res(AARIARAN.rottbias);    % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.tfreqx =       vec_rand_ab_res(AARIARAN.rottfreq);    % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.tphasex =      vec_rand_ab_res(AARIARAN.rottphase);   % (rad) any phase shift of rotational oscillations in x y z

AARIAMAN.B.trans.tampy  =     vec_rand_ab_res(AARIARAN.transtamp);   % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.tbiasy =     vec_rand_ab_res(AARIARAN.transtbias);  % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.tfreqy =     vec_rand_ab_res(AARIARAN.transtfreq);  % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.tphasey =    vec_rand_ab_res(AARIARAN.transtphase); % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.tampy  =       vec_rand_ab_res(AARIARAN.rottamp);     % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.tbiasy =       vec_rand_ab_res(AARIARAN.rottbias);    % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.tfreqy =       vec_rand_ab_res(AARIARAN.rottfreq);    % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.tphasey =      vec_rand_ab_res(AARIARAN.rottphase);   % (rad) any phase shift of rotational oscillations in x y z

AARIAMAN.B.trans.tampz  =     vec_rand_ab_res(AARIARAN.transtamp);   % (m) amplitude of translational oscillations in x y z
AARIAMAN.B.trans.tbiasz =     vec_rand_ab_res(AARIARAN.transtbias);  % (m)starting point of translational oscillations in x y z
AARIAMAN.B.trans.tfreqz =     vec_rand_ab_res(AARIARAN.transtfreq);  % (rad/s) frequency of translational oscillations in x y z
AARIAMAN.B.trans.tphasez =    vec_rand_ab_res(AARIARAN.transtphase); % (rad) any phase shift of translational oscillations in x y z

AARIAMAN.B.rot.tampz  =       vec_rand_ab_res(AARIARAN.rottamp);     % amplitude of rotational oscillations about x y z
AARIAMAN.B.rot.tbiasz =       vec_rand_ab_res(AARIARAN.rottbias);    % starting point of rotational oscillations in x y z
AARIAMAN.B.rot.tfreqz =       vec_rand_ab_res(AARIARAN.rottfreq);    % (rad/s) frequency of rotational oscillations in x y z
AARIAMAN.B.rot.tphasez =      vec_rand_ab_res(AARIARAN.rottphase);   % (rad) any phase shift of rotational oscillations in x y z


% % Joint Oscillation
%Loop for randomizing joint movements

for nJ=1:AARIAMAN.linknumber-2
    %motion
    AARIAMAN.J(nJ).amp =         vec_rand_ab_res(AARIARAN.amp); %amplitude of revolute joint oscillation [rad]
    AARIAMAN.J(nJ).prismamp =    vec_rand_ab_res(AARIARAN.prismamp); %amplitude of prismatic joint oscillation [m]
    AARIAMAN.J(nJ).bias =        vec_rand_ab_res(AARIARAN.bias); %starting point of joint oscillation [rad][m]
    AARIAMAN.J(nJ).freq =        vec_rand_ab_res(AARIARAN.freq); %oscillation frequency [Hz]
    AARIAMAN.J(nJ).phase =       vec_rand_ab_res(AARIARAN.phase); %phase shift of oscillations [rad]

    %torque
    AARIAMAN.J(nJ).tamp =        vec_rand_ab_res(AARIARAN.tamp); %amplitude of revolute joint oscillation [rad]
    AARIAMAN.J(nJ).tprismamp =   vec_rand_ab_res(AARIARAN.tprismamp); %amplitude of prismatic joint oscillation [m]
    AARIAMAN.J(nJ).tbias =       vec_rand_ab_res(AARIARAN.tbias); %starting point of joint oscillation [rad][m]
    AARIAMAN.J(nJ).tfreq =       vec_rand_ab_res(AARIARAN.tfreq); %oscillation frequency [Hz]
    AARIAMAN.J(nJ).tphase =      vec_rand_ab_res(AARIARAN.tphase); %phase shift of oscillations [rad]
end
    
     
% % Data sets
%Joint sine wave trajectory parameters 
%rows [amplitude, prismatic amplitude, bias, frequency, phase, torqueAmplitude, forcePrismaticAmplitude, torqueBias, torqueFrequency, torquePhase]
AARIAMAN.Jconfig = [AARIAMAN.J(1:6).amp;AARIAMAN.J(1:6).prismamp;AARIAMAN.J(1:6).bias;AARIAMAN.J(1:6).freq;AARIAMAN.J(1:6).phase;AARIAMAN.J(1:6).tamp;AARIAMAN.J(1:6).tprismamp;AARIAMAN.J(1:6).tbias;AARIAMAN.J(1:6).tfreq;AARIAMAN.J(1:6).tphase]';

%Base sine wave parameters columns [translation; rotation]
%columns [x(amplitude, bias, freq, phase),y(ampl...,z... x(torqueAmplitude, torqueBias, torqueFreq, torquepPhase),y...,z...]
AARIAMAN.Bconfig = [cell2mat(struct2cell(AARIAMAN.B.trans)),cell2mat(struct2cell(AARIAMAN.B.rot))];

%DH table [alpha, a, theta, d, linktype]   
AARIAMAN.DHconfig = zeros(5,length(AARIAMAN.DH));
for nLL=1:AARIAMAN.linknumber
   vec1 = [AARIAMAN.DH(nLL).alpha;AARIAMAN.DH(nLL).a;AARIAMAN.DH(nLL).theta;AARIAMAN.DH(nLL).d;AARIAMAN.DH(nLL).linktype];
   AARIAMAN.DHconfig(1:5,nLL)= vec1;
end
AARIAMAN.DHconfig = AARIAMAN.DHconfig';

% % Dynamic model
AARIAMAN.D.endspring = 1e6; %Prismatic joint end collision
AARIAMAN.D.endmass = 100; %Mass for damper
AARIAMAN.D.revolutedamping = 1; %Internal joint damping coefficient N*m/(deg/s)
AARIAMAN.D.prismaticdamping = 100;%Internal joint damping coefficient N/(m/s)

for nLL =1:AARIAMAN.linknumber
    AARIAMAN.L(nLL).xmax = AARIAMAN.DH(nLL).d; %Prismatic joint maximum extension
    AARIAMAN.L(nLL).xmin = -AARIAMAN.DH(nLL).d;%Prismatic joint minimum extension
end

% % Camera
%horizontal_fov = 2*atan(11/9);%~100deg
%vertical_fov = 2*atan(8.2/9);   %~84 deg
Camera.horizontal_fov = 1.18;%~100deg
Camera.vertical_fov = 1;   %~84 deg
Camera.focus = 2; % Camera focus distance [m]
Camera.range = 6; % Max visibility distance
Camera.NoiseGain = 0;
Camera.Offset = [0 0.5 -0.5]; % Cartesian from tooltip [m] [left, up, front]
Camera.Rotation = eul2rotm([0 pi/4 0],'ZYX'); % [Rad] rotate camera view
Camera.PickAngle = pi/8; %maximum angle difference for pickup [rad]
Camera.PickDistance = 0.2; %maximum distance from object for pickup [m]
Camera.PickRotation = 0.95; %minimum rotation matrix row maximum value for pickup

Camera.AzmTol = 0.1; %Difference between spherical coordinates from camera to tooltip and camera to object
Camera.DstTol = 0.1;
Camera.IncTol = 0.2;

Camera.ObjectTranslation = [vec_rand_ab_res([-2,2,0.1]) vec_rand_ab_res([-2,2,0.1]) vec_rand_ab_res([0.1,2,0.1])]; % Cartesian object translation
Camera.ObjectEul = [vec_rand_ab_res([-pi,pi,pi/4]) vec_rand_ab_res([-pi,pi,pi/4]) vec_rand_ab_res([-pi,pi,pi/4])]; %Object rotation angles

Camera.ObjectRotation = eul2rotm(Camera.ObjectEul,'ZYX'); %Object rotation matrix

%Camera.ObjectTranslation = [0 1 1]; % Cartesian object translation
%Camera.ObjectRotation = eul2rotm([0 0 0],'ZYX'); %Object rotation matrix
Camera.ObjectRadius = 0.05; %Tooltip compensation for object center point



% % Parsim variable assignment
simIn(idx) = simIn(idx).setVariable('AARIAMAN',AARIAMAN);
simIn(idx) = simIn(idx).setVariable('AARIASEN',AARIASEN);
simIn(idx) = simIn(idx).setVariable('AARIARAN',AARIARAN);
simIn(idx) = simIn(idx).setVariable('AARIAENV',AARIAENV);
simIn(idx) = simIn(idx).setVariable('Camera',Camera);
simIn(idx) = simIn(idx).setVariable('VSS_Joystick',VSS_Joystick);


clearvars vec1 nLL nSS nJ

simOut = sim(simIn);
assignin('base','simOut', simOut) 

%{
%--------------------------------------------------------------------------
end

%% Parsim parameters
simOut = parsim(simIn ...
    ,'ShowSimulationManager','on'...
    ,'TransferBaseWorkspaceVariables', false...
    ,'UseFastRestart', false ...
    ...,'TimeOut',10 ...
    ...,'IgnoredZcDiagnostic','none'...
    ,'ShowProgress', false...
    );
filename = strcat(num2str(now),'.mat');
save(filename)

%--------------------------------------------------------------------------
%}