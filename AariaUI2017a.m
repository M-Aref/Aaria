classdef AariaUI2017a < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        EditField             matlab.ui.control.NumericEditField
        EditField_2           matlab.ui.control.NumericEditField
        EditField_3           matlab.ui.control.NumericEditField
        EditField_4           matlab.ui.control.NumericEditField
        EditField_5           matlab.ui.control.NumericEditField
        EditField_6           matlab.ui.control.NumericEditField
        EditField_7           matlab.ui.control.NumericEditField
        EditField_8           matlab.ui.control.NumericEditField
        EditField_9           matlab.ui.control.NumericEditField
        EditField_10          matlab.ui.control.NumericEditField
        EditField_11          matlab.ui.control.NumericEditField
        EditField_12          matlab.ui.control.NumericEditField
        EditField_13          matlab.ui.control.NumericEditField
        EditField_14          matlab.ui.control.NumericEditField
        EditField_15          matlab.ui.control.NumericEditField
        EditField_16          matlab.ui.control.NumericEditField
        EditField_17          matlab.ui.control.NumericEditField
        EditField_18          matlab.ui.control.NumericEditField
        EditField_19          matlab.ui.control.NumericEditField
        EditField_20          matlab.ui.control.NumericEditField
        EditField_21          matlab.ui.control.NumericEditField
        EditField_22          matlab.ui.control.NumericEditField
        EditField_23          matlab.ui.control.NumericEditField
        EditField_24          matlab.ui.control.NumericEditField
        EditField_25          matlab.ui.control.NumericEditField
        EditField_26          matlab.ui.control.NumericEditField
        EditField_27          matlab.ui.control.NumericEditField
        EditField_28          matlab.ui.control.NumericEditField
        EditField_29          matlab.ui.control.NumericEditField
        EditField_30          matlab.ui.control.NumericEditField
        AlphaLabel            matlab.ui.control.Label
        aLabel                matlab.ui.control.Label
        ThetaLabel            matlab.ui.control.Label
        dLabel                matlab.ui.control.Label
        LinktypeLabel         matlab.ui.control.Label
        SimulateButton        matlab.ui.control.Button
        StopsimulationButton  matlab.ui.control.Button
        Joint1Label           matlab.ui.control.Label
        Joint2Label           matlab.ui.control.Label
        Joint3Label           matlab.ui.control.Label
        Joint4Label           matlab.ui.control.Label
        Joint5Label           matlab.ui.control.Label
        Joint6Label           matlab.ui.control.Label
        DHParametersLabel     matlab.ui.control.Label
        SCARAButton           matlab.ui.control.Button
        PUMA560Button         matlab.ui.control.Button
        CARTESIANButton       matlab.ui.control.Button
        HIABButton            matlab.ui.control.Button
        TWORButton            matlab.ui.control.Button
        LoadandsimulatepresetmanipulatorconfigurationsLabel  matlab.ui.control.Label
    end

    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(~)
           %load_system('AariaJoystickR2017a.slx')
        end
        
        % Button pushed function: SimulateButton
        function SimulateButtonPushed(app, ~)
      
            try
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                % {
                AARIAMAN.Joint1 =   [app.EditField.Value app.EditField_2.Value app.EditField_3.Value app.EditField_4.Value app.EditField_5.Value];
                AARIAMAN.Joint2 =   [app.EditField_6.Value app.EditField_7.Value app.EditField_8.Value app.EditField_9.Value app.EditField_10.Value];
                AARIAMAN.Joint3 =   [app.EditField_11.Value app.EditField_12.Value app.EditField_13.Value app.EditField_14.Value app.EditField_15.Value];
                AARIAMAN.Joint4 =   [app.EditField_16.Value app.EditField_17.Value app.EditField_18.Value app.EditField_19.Value app.EditField_20.Value];
                AARIAMAN.Joint5 =   [app.EditField_21.Value app.EditField_22.Value app.EditField_23.Value app.EditField_24.Value app.EditField_25.Value];
                AARIAMAN.Joint6 =   [app.EditField_26.Value app.EditField_27.Value app.EditField_28.Value app.EditField_29.Value app.EditField_30.Value];
                %}
                %{
                %       CUSTOM        %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [pi/2       -0.12          0         1.35             1];
                 AARIAMAN.Joint2 =   [0          1.9142         0          0               1];
                 AARIAMAN.Joint3 =   [pi/2         0           pi/2       0.106            1];
                 AARIAMAN.Joint4 =   [-pi/2        0            0         1.75             2];
                 AARIAMAN.Joint5 =   [-pi/2       0.13        -pi/2        0               1];
                 AARIAMAN.Joint6 =   [0           0.35          0          0               1];
                %}
                assignin('base','AARIAMAN', AARIAMAN)    
                run AariaParamsJoystick2017a
            end
            catch
            end
        end
        
        % Button pushed function: StopsimulationButton
        function StopsimulationButtonPushed(~, ~)
            set_param('AariaJoystickR2017a','SimulationCommand','stop');
            %filename = strcat(num2str(now),'.mat');
            %save(filename)

        end
        
        % Button pushed function: SCARAButton
        function SCARAButtonPushed(~, ~)
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                %SCARA        %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [0          0           0           2               1];
                 AARIAMAN.Joint2 =   [0          1           0           0               1];
                 AARIAMAN.Joint3 =   [pi         1           0           2               2];
                 AARIAMAN.Joint4 =   [0          0           0           0               1];
                 AARIAMAN.Joint5 =   [pi/2       1           pi/2        0.5             0];
                 AARIAMAN.Joint6 =   [pi/2       1           pi/2        0.5             0];
                 assignin('base','AARIAMAN', AARIAMAN)    
                 run AariaParamsJoystick2017a
            end
        end

        % Button pushed function: PUMA560Button
        function PUMA560ButtonPushed(~, ~)
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                %PUMA 560     %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [0          0           0           1.5             1];
                 AARIAMAN.Joint2 =   [-pi/2      0           0           0.3             1];
                 AARIAMAN.Joint3 =   [0          1           0           -0.3            1];
                 AARIAMAN.Joint4 =   [-pi/2      0.1         0           1               1];
                 AARIAMAN.Joint5 =   [pi/2       0           0           0               1];
                 AARIAMAN.Joint6 =   [-pi/2      0           0           0               1];
                assignin('base','AARIAMAN', AARIAMAN)    
                run AariaParamsJoystick2017a
            end
        end

        % Button pushed function: CARTESIANButton
        function CARTESIANButtonPushed(~, ~)
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                %Cartesian    %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [0          0           0           3               2];
                 AARIAMAN.Joint2 =   [pi/2       0           pi/2        2.0             2];
                 AARIAMAN.Joint3 =   [pi/2       0           pi/2        2.0             2];
                 AARIAMAN.Joint4 =   [-pi/2      0           0           0.5             1];
                 AARIAMAN.Joint5 =   [pi/2       0           0           0               1];
                 AARIAMAN.Joint6 =   [-pi/2      0           0           0               1];
                assignin('base','AARIAMAN', AARIAMAN)    
                run AariaParamsJoystick2017a
            end
        end
        
        % Button pushed function: HIABButton
        function HIABButtonPushed(~, ~)
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                %       CUSTOM        %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [0       -0.12          0         1.35             1];
                 AARIAMAN.Joint2 =   [-pi/2          0         0          0               1];
                 AARIAMAN.Joint3 =   [0         1.9142           pi/2       0.106            1];
                 AARIAMAN.Joint4 =   [-pi/2        0            0         1.75+3             2];
                 AARIAMAN.Joint5 =   [0       0.13        -pi/2        0               0];
                 AARIAMAN.Joint6 =   [0           0.35          0          0               0];
                 %{
                 AARIAMAN.Joint1 =   [pi/2       -0.12          0         1.35             1];
                 AARIAMAN.Joint2 =   [0          1.9142         0          0               1];
                 AARIAMAN.Joint3 =   [pi/2         0           pi/2       0.106            1];
                 AARIAMAN.Joint4 =   [-pi/2        0            0         1.75+3             2];
                 AARIAMAN.Joint5 =   [-pi/2       0.13        -pi/2        0               1];
                 AARIAMAN.Joint6 =   [0           0.35          0          0               1];
                 %}
                assignin('base','AARIAMAN', AARIAMAN)    
                run AariaParamsJoystick2017a
            end
        end
        
        % Button pushed function: TWORButton
        function TWORButtonPushed(~, ~)
            if get_param('AariaJoystickR2017a','SimulationStatus') == 'stopped'
                %Cartesian    %ALPHA     A(legth)    THETA      D(offset)     LINKTYPE
                 AARIAMAN.Joint1 =   [pi/2       0           0           0               1];
                 AARIAMAN.Joint2 =   [0          1           0           0               1];
                 AARIAMAN.Joint3 =   [0          1           0           0               1];
                 AARIAMAN.Joint4 =   [-pi/2      0           0           0.5             0];
                 AARIAMAN.Joint5 =   [pi/2       0           0           0               0];
                 AARIAMAN.Joint6 =   [-pi/2      0           0           0               0];
                assignin('base','AARIAMAN', AARIAMAN)    
                run AariaParamsJoystick2017a
            end
        end
    end

    % App initialization and construction
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure;
            app.UIFigure.Position = [100 100 634 480];
            app.UIFigure.Name = 'UI Figure';

            % Create EditField
            app.EditField = uieditfield(app.UIFigure, 'numeric');
            app.EditField.Position = [116 384 100 22];

            % Create EditField_2
            app.EditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_2.Position = [215 384 100 22];

            % Create EditField_3
            app.EditField_3 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_3.Position = [314 384 100 22];

            % Create EditField_4
            app.EditField_4 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_4.Position = [413 384 100 22];

            % Create EditField_5
            app.EditField_5 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_5.Position = [512 384 100 22];

            % Create EditField_6
            app.EditField_6 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_6.Position = [116 363 100 22];

            % Create EditField_7
            app.EditField_7 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_7.Position = [215 363 100 22];

            % Create EditField_8
            app.EditField_8 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_8.Position = [314 363 100 22];

            % Create EditField_9
            app.EditField_9 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_9.Position = [413 363 100 22];

            % Create EditField_10
            app.EditField_10 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_10.Position = [512 363 100 22];

            % Create EditField_11
            app.EditField_11 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_11.Position = [116 342 100 22];

            % Create EditField_12
            app.EditField_12 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_12.Position = [215 342 100 22];

            % Create EditField_13
            app.EditField_13 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_13.Position = [314 342 100 22];

            % Create EditField_14
            app.EditField_14 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_14.Position = [413 342 100 22];

            % Create EditField_15
            app.EditField_15 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_15.Position = [512 342 100 22];

            % Create EditField_16
            app.EditField_16 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_16.Position = [116 321 100 22];

            % Create EditField_17
            app.EditField_17 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_17.Position = [215 321 100 22];

            % Create EditField_18
            app.EditField_18 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_18.Position = [314 321 100 22];

            % Create EditField_19
            app.EditField_19 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_19.Position = [413 321 100 22];

            % Create EditField_20
            app.EditField_20 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_20.Position = [512 321 100 22];

            % Create EditField_21
            app.EditField_21 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_21.Position = [116 300 100 22];

            % Create EditField_22
            app.EditField_22 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_22.Position = [215 300 100 22];

            % Create EditField_23
            app.EditField_23 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_23.Position = [314 300 100 22];

            % Create EditField_24
            app.EditField_24 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_24.Position = [413 300 100 22];

            % Create EditField_25
            app.EditField_25 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_25.Position = [512 300 100 22];

            % Create EditField_26
            app.EditField_26 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_26.Position = [116 279 100 22];

            % Create EditField_27
            app.EditField_27 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_27.Position = [215 279 100 22];

            % Create EditField_28
            app.EditField_28 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_28.Position = [314 279 100 22];

            % Create EditField_29
            app.EditField_29 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_29.Position = [413 279 100 22];

            % Create EditField_30
            app.EditField_30 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_30.Position = [512 279 100 22];

            % Create AlphaLabel
            app.AlphaLabel = uilabel(app.UIFigure);
            app.AlphaLabel.Position = [150 405 37 15];
            app.AlphaLabel.Text = 'Alpha';

            % Create aLabel
            app.aLabel = uilabel(app.UIFigure);
            app.aLabel.Position = [252 405 25 15];
            app.aLabel.Text = 'a';

            % Create ThetaLabel
            app.ThetaLabel = uilabel(app.UIFigure);
            app.ThetaLabel.Position = [340 405 36 15];
            app.ThetaLabel.Text = 'Theta';

            % Create dLabel
            app.dLabel = uilabel(app.UIFigure);
            app.dLabel.Position = [450 405 25 15];
            app.dLabel.Text = 'd';

            % Create LinktypeLabel
            app.LinktypeLabel = uilabel(app.UIFigure);
            app.LinktypeLabel.Position = [544 405 54 15];
            app.LinktypeLabel.Text = 'Link type';

            % Create SimulateButton
            app.SimulateButton = uibutton(app.UIFigure, 'push');
            app.SimulateButton.ButtonPushedFcn = createCallbackFcn(app, @SimulateButtonPushed, true);
            app.SimulateButton.Position = [314 239 100 22];
            app.SimulateButton.Text = 'Simulate';

            % Create StopsimulationButton
            app.StopsimulationButton = uibutton(app.UIFigure, 'push');
            app.StopsimulationButton.ButtonPushedFcn = createCallbackFcn(app, @StopsimulationButtonPushed, true);
            app.StopsimulationButton.Position = [314 208 100 22];
            app.StopsimulationButton.Text = 'Stop simulation';

            % Create Joint1Label
            app.Joint1Label = uilabel(app.UIFigure);
            app.Joint1Label.Position = [58 388 38 15];
            app.Joint1Label.Text = 'Joint1';

            % Create Joint2Label
            app.Joint2Label = uilabel(app.UIFigure);
            app.Joint2Label.Position = [58 367 38 15];
            app.Joint2Label.Text = 'Joint2';

            % Create Joint3Label
            app.Joint3Label = uilabel(app.UIFigure);
            app.Joint3Label.Position = [58 346 38 15];
            app.Joint3Label.Text = 'Joint3';

            % Create Joint4Label
            app.Joint4Label = uilabel(app.UIFigure);
            app.Joint4Label.Position = [58 325 38 15];
            app.Joint4Label.Text = 'Joint4';

            % Create Joint5Label
            app.Joint5Label = uilabel(app.UIFigure);
            app.Joint5Label.Position = [58 304 38 15];
            app.Joint5Label.Text = 'Joint5';

            % Create Joint6Label
            app.Joint6Label = uilabel(app.UIFigure);
            app.Joint6Label.Position = [58 283 38 15];
            app.Joint6Label.Text = 'Joint6';

            % Create DHParametersLabel
            app.DHParametersLabel = uilabel(app.UIFigure);
            app.DHParametersLabel.FontSize = 16;
            app.DHParametersLabel.Position = [116 449 117 20];
            app.DHParametersLabel.Text = 'DH-Parameters';
            
            % Create TWORButton
            app.TWORButton = uibutton(app.UIFigure, 'push');
            app.TWORButton.ButtonPushedFcn = createCallbackFcn(app, @TWORButtonPushed, true);
            app.TWORButton.Position = [116 150 100 22];
            app.TWORButton.Text = '2R';
            
            % Create HIABButton
            app.HIABButton = uibutton(app.UIFigure, 'push');
            app.HIABButton.ButtonPushedFcn = createCallbackFcn(app, @HIABButtonPushed, true);
            app.HIABButton.Position = [116 120 100 22];
            app.HIABButton.Text = 'HIAB';
            
            % Create SCARAButton
            app.SCARAButton = uibutton(app.UIFigure, 'push');
            app.SCARAButton.ButtonPushedFcn = createCallbackFcn(app, @SCARAButtonPushed, true);
            app.SCARAButton.Position = [116 90 100 22];
            app.SCARAButton.Text = 'SCARA';

            % Create PUMA560Button
            app.PUMA560Button = uibutton(app.UIFigure, 'push');
            app.PUMA560Button.ButtonPushedFcn = createCallbackFcn(app, @PUMA560ButtonPushed, true);
            app.PUMA560Button.Position = [116 60 100 22];
            app.PUMA560Button.Text = 'PUMA 560';

            % Create CARTESIANButton
            app.CARTESIANButton = uibutton(app.UIFigure, 'push');
            app.CARTESIANButton.ButtonPushedFcn = createCallbackFcn(app, @CARTESIANButtonPushed, true);
            app.CARTESIANButton.Position = [116 30 100 22];
            app.CARTESIANButton.Text = 'CARTESIAN';
            
            % Create LoadandsimulatepresetmanipulatorconfigurationsLabel
            app.LoadandsimulatepresetmanipulatorconfigurationsLabel = uilabel(app.UIFigure);
            app.LoadandsimulatepresetmanipulatorconfigurationsLabel.Position = [260 90 338 28];
            app.LoadandsimulatepresetmanipulatorconfigurationsLabel.Text = {'Load and simulate preset manipulator configurations.'; 'Pushing the button during simulation will stop the simulation.'};
        
        end
    end

    methods (Access = public)

        % Construct app
        function app = AariaUI2017a

            % Create and configure components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            set_param('AariaJoystickR2017a','SimulationCommand','stop');
            

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end