classdef final_project < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        ConnectRoboDKButton             matlab.ui.control.Button
        ExampleConfigurationPanel       matlab.ui.container.Panel
        Tet1EditField                   matlab.ui.control.NumericEditField
        Tet1EditFieldLabel              matlab.ui.control.Label
        Tet2EditField                   matlab.ui.control.NumericEditField
        Tet2EditFieldLabel              matlab.ui.control.Label
        Tet3EditField                   matlab.ui.control.NumericEditField
        Tet3EditFieldLabel              matlab.ui.control.Label
        Tet4EditField                   matlab.ui.control.NumericEditField
        Tet4EditFieldLabel              matlab.ui.control.Label
        Tet5EditField                   matlab.ui.control.NumericEditField
        Tet5EditFieldLabel              matlab.ui.control.Label
        Tet6EditFieldLabel              matlab.ui.control.Label
        Tet6EditField                   matlab.ui.control.NumericEditField
        SolveButton                     matlab.ui.control.Button
        ShowonRoboDKButton              matlab.ui.control.Button
        SelectSolution18EditFieldLabel  matlab.ui.control.Label
        SelectSolution18EditField       matlab.ui.control.NumericEditField
        DesiredPosePanel                matlab.ui.container.Panel
        T11EditField                    matlab.ui.control.NumericEditField
        T11EditFieldLabel               matlab.ui.control.Label
        T12EditField                    matlab.ui.control.NumericEditField
        T12EditFieldLabel               matlab.ui.control.Label
        T13EditField                    matlab.ui.control.NumericEditField
        T13EditFieldLabel               matlab.ui.control.Label
        T14EditField                    matlab.ui.control.NumericEditField
        T14EditFieldLabel               matlab.ui.control.Label
        T21EditField                    matlab.ui.control.NumericEditField
        T21EditFieldLabel               matlab.ui.control.Label
        T22EditField                    matlab.ui.control.NumericEditField
        T22EditFieldLabel               matlab.ui.control.Label
        T23EditField                    matlab.ui.control.NumericEditField
        T23EditFieldLabel               matlab.ui.control.Label
        T24EditField                    matlab.ui.control.NumericEditField
        T24EditFieldLabel               matlab.ui.control.Label
        T31EditField                    matlab.ui.control.NumericEditField
        T31EditFieldLabel               matlab.ui.control.Label
        T32EditField                    matlab.ui.control.NumericEditField
        T32EditFieldLabel               matlab.ui.control.Label
        T33EditField                    matlab.ui.control.NumericEditField
        T33EditFieldLabel               matlab.ui.control.Label
        T34EditField                    matlab.ui.control.NumericEditField
        T34EditFieldLabel               matlab.ui.control.Label
        T41EditField                    matlab.ui.control.NumericEditField
        T41EditFieldLabel               matlab.ui.control.Label
        T42EditField                    matlab.ui.control.NumericEditField
        T42EditFieldLabel               matlab.ui.control.Label
        T43EditField                    matlab.ui.control.NumericEditField
        T43EditFieldLabel               matlab.ui.control.Label
        T44EditField                    matlab.ui.control.NumericEditField
        T44EditFieldLabel               matlab.ui.control.Label
        InputTypeButtonGroup            matlab.ui.container.ButtonGroup
        ExampleConfigurationButton      matlab.ui.control.RadioButton
        DesiredPoseButton               matlab.ui.control.RadioButton
        UITable                         matlab.ui.control.Table
        ModellingandControlofRobotsFinalProjectLabel  matlab.ui.control.Label
        KerimMoral518191043Label        matlab.ui.control.Label
    end

    
    properties (Access = private)
        selected_conf
        tetc_deg_all
        robot
        robodk_connected
        solved
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.selected_conf = 1;
            app.robodk_connected = 0;
            app.solved = 0;
        end

        % Button pushed function: ConnectRoboDKButton
        function ConnectRoboDKButtonPushed(app, event)
            %Generate a Robolink object RDK. This object interfaces with RoboDK.
            RDK = Robolink;
            app.robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT);
            
            if app.robot.Valid() == 0
                app.ConnectRoboDKButton.BackgroundColor = [1.00 0.00 0.00];
            else
                app.ConnectRoboDKButton.BackgroundColor = [0.00 1.00 0.00];
                app.robodk_connected = 1;
            end
            
        end

        % Button pushed function: SolveButton
        function SolveButtonPushed(app, event)
            %Robot parameters
            a1 = 585; a2 = 650; a3 = 192;
            d1 = 190; d2 = 730; d3 = 132;
            
            %Angle variables
            syms tet1 tet2 tet3 tet4 tet5 tet6
            
            %Screw axes
            w1 = [0 0 1]';
            w2 = [0 1 0]';
            w3 = [0 1 0]';
            w4 = [1 0 0]';
            w5 = [0 1 0]';
            w6 = [1 0 0]';
            
            
            %Screw points
            q1 = [0        0        0]';
            q2 = [d1       0        a1]';
            q3 = [d1       0        a1+a2]';
            q4 = [d1+d2    0        a1+a2+a3]';
            q5 = q4;
            q6 = q4;
            
            %Point velocities
            v1 = -cross(w1,q1);
            v2 = -cross(w2,q2);
            v3 = -cross(w3,q3);
            v4 = -cross(w4,q4);
            v5 = -cross(w5,q5);
            v6 = -cross(w6,q6);
            
            %Skews
            w1_skew = skew(w1);
            S1_skew = [w1_skew v1; zeros(1,4)];
            
            w2_skew = skew(w2);
            S2_skew = [w2_skew v2; zeros(1,4)];
            
            w3_skew = skew(w3);
            S3_skew = [w3_skew v3; zeros(1,4)];
            
            w4_skew = skew(w4);
            S4_skew = [w4_skew v4; zeros(1,4)];
            
            w5_skew = skew(w5);
            S5_skew = [w5_skew v5; zeros(1,4)];
            
            w6_skew = skew(w6);
            S6_skew = [w6_skew v6; zeros(1,4)];
            
            %Home position
            R0 = [1 0 0; 0 1 0; 0 0 1];
            q0 = [d1+d2+d3 0        a1+a2+a3]';
            M = [R0 q0; 0 0 0 1];
            
            
            if(app.selected_conf == 1)
                tetd1 = deg2rad(app.Tet1EditField.Value);  tetd2 = deg2rad(app.Tet2EditField.Value);
                tetd3 = deg2rad(app.Tet3EditField.Value);  tetd4 = deg2rad(app.Tet4EditField.Value);
                tetd5 = deg2rad(app.Tet5EditField.Value);  tetd6 = deg2rad(app.Tet6EditField.Value);
                
                tetd = [tetd1 tetd2 tetd3 tetd4 tetd5 tetd6];
                
                %Compute forward kinematics
                T = expm(S1_skew * tet1);
                T = T * expm(S2_skew * tet2);
                T = T * expm(S3_skew * tet3);
                T = T * expm(S4_skew * tet4) ;
                T = T * expm(S5_skew * tet5) ;
                T = T * expm(S6_skew * tet6) ;
                
                T = T * M;
                
                %Desired Position
                Td = real(double(subs(T, [tet1 tet2 tet3 tet4 tet5 tet6], tetd)));
                
            else
                Td11 = app.T11EditField.Value;   Td12 = app.T12EditField.Value;
                Td13 = app.T13EditField.Value;   Td14 = app.T14EditField.Value;
                
                Td21 = app.T21EditField.Value;   Td22 = app.T22EditField.Value;
                Td23 = app.T23EditField.Value;   Td24 = app.T24EditField.Value;
                
                Td31 = app.T31EditField.Value;   Td32 = app.T32EditField.Value;
                Td33 = app.T33EditField.Value;   Td34 = app.T34EditField.Value;
                
                Td41 = app.T41EditField.Value;   Td42 = app.T42EditField.Value;
                Td43 = app.T43EditField.Value;   Td44 = app.T44EditField.Value;
                
                Td = [Td11 Td12 Td13 Td14;
                    Td21 Td22 Td23 Td24;
                    Td31 Td32 Td33 Td34;
                    Td41 Td42 Td43 Td44;];
                
            end
            %% Begin I.K Calculations
            %Calculate Teta1
            p1 = Td*inv(M)*hp(q4);
            p1x = p1(1); p1y = p1(2);
            
            tetc1_1 = atan2( p1y, p1x);
            tetc1_2 = atan2(-p1y,-p1x);
            
            %             tetc1 = tetc1_1; %Use first or second teta1 value
            
            tetc1_all = [tetc1_1 tetc1_2];
            
            %Translated points for non-singularity
            q1p = [0 0 0]';      q2p = [0 0 a1]';
            q3p = [0 0 a1+a2]';  q4p = [d2 0 a1+a2+a3]';
            q5p = q4p;           q6p = q4p;
            
            count = 1;
            app.tetc_deg_all = zeros(8,6);
            for i=1:2 %teta1
                tetc1 = tetc1_all(i);
                
                Tt = [1 0 0 -d1*cos(tetc1);
                    0 1 0 -d1*sin(tetc1);
                    0 0 1 0;
                    0 0 0 1];
                
                Mp = [1 0 0 d2+d3;
                    0 1 0 0;
                    0 0 1 a1+a2+a3;
                    0 0 0 1];
                
                
                %Calculate Teta3
                T1 = Tt * Td* inv(Mp);
                % T1 = Td* inv(M);
                p2 = T1 * hp(q4p);
                
                %Verify results
                sigma_2 = sqrt(p2(1)^2+p2(2)^2+(p2(3)-a1)^2);
                tet0_2 = atan2(a2*d2, -a2*a3);
                theta3c_0_2 = acos((d2^2+a3^2+a2^2-p2(1)^2 ...
                    -p2(2)^2-(p2(3)-a1)^2) / (2*a2*sqrt(d2^2+a3^2)));
                thetac3_1_2 = tet0_2 + theta3c_0_2;
                thetac3_2_2 = tet0_2 - theta3c_0_2;
                
                %                 tetc3 = thetac3_2_2; %Teta3 value that is being used.
                
                tetc3_all = [thetac3_1_2 thetac3_2_2];
                
                %Solving teta2
                v1 = -cross(w1,q1p);
                S1_skew = [w1_skew v1; zeros(1,4)];
                
                v2 = -cross(w2,q2p);
                S2_skew = [w2_skew v2; zeros(1,4)];
                
                v3 = -cross(w3,q3p);
                S3_skew = [w3_skew v3; zeros(1,4)];
                
                v4 = -cross(w4,q4p);
                S4_skew = [w4_skew v4; zeros(1,4)];
                
                v5 = -cross(w5,q5p);
                S5_skew = [w5_skew v5; zeros(1,4)];
                
                v6 = -cross(w6,q6p);
                S6_skew = [w6_skew v6; zeros(1,4)];
                
                for j=1:2 %teta3
                    tetc3 = tetc3_all(j);
                    T2 = expm(-S1_skew*tetc1)*  Tt * Td* inv(Mp);
                    
                    q7_hp = expm(S3_skew*tetc3)*hp(q4p);
                    p3 = T2*hp(q4p); %p3 hatalı olmalı
                    p3x = p3(1); p3y = p3(2); p3z = p3(3);
                    q7x = q7_hp(1); q7y = q7_hp(2); q7z = q7_hp(3);
                    
                    at1 = p3x*q7z - p3x*a1 - q7x*p3z + q7x*a1;
                    at2 = q7x*p3x + q7z*p3z - q7z*a1 - p3z*a1 + a1^2;
                    tetc2 = atan2(at1,at2);
                    
                    %Solving teta4 and teta5
                    T3 = expm(-S3_skew*tetc3)*expm(-S2_skew*tetc2)*T2;
                    q6pp = [0 0 a1+a2+a3]';
                    p4 = T3*hp(q6pp);
                    
                    p4x = p4(1); p4y=p4(2); p4z=p4(3);
                    
                    %Hesaplar manual olarak teyit edildi.
                    tetc4_1 = atan2( p4y, (a1+a2+a3-p4z));
                    tetc4_2 = atan2(-p4y,-(a1+a2+a3-p4z));
                    
                    tetc5_2 = atan2( sqrt(2*p4x*d2-p4x^2),d2-p4x);
                    tetc5_1 = atan2(-sqrt(2*p4x*d2-p4x^2),d2-p4x);
                    
%                     tetc4 = tetc4_2;
%                     tetc5 = tetc5_2;
                    
                    tetc4_all = [tetc4_1 tetc4_2];
                    tetc5_all = [tetc5_1 tetc5_2];
                    
                    for k=1:2
                        tetc4 = tetc4_all(k);
                        tetc5 = tetc5_all(k);
                        %Calculate teta6
                        T4 = expm(-S5_skew*tetc5)*expm(-S4_skew*tetc4)*T3;
                        q8 = [0 0 a1+a2]';
                        p5 = T4 * hp(q8);
                        p5x = p5(1); p5y = p5(2); p5z = p5(3);
                        tetc6 = atan2(p5y, p5z-a1-a2-a3);
                        tetc6 = pi - tetc6;
                        
                        tetc         = [tetc1 tetc2 tetc3 tetc4 tetc5 tetc6];
                        tetc_deg     = CheckTet(rad2deg(tetc));
                        app.tetc_deg_all(count,:) = tetc_deg;
                        count = count+1;

                    end
                end
            end
            app.solved = 1;
            
            if(app.solved && app.robodk_connected)
                app.ShowonRoboDKButton.Enable = 'on';
            end
            
            Tet1 = app.tetc_deg_all(:,1); Tet4 = app.tetc_deg_all(:,4);
            Tet2 = app.tetc_deg_all(:,2); Tet5 = app.tetc_deg_all(:,5);
            Tet3 = app.tetc_deg_all(:,3); Tet6 = app.tetc_deg_all(:,6);
            
            t = table(Tet1, Tet2, Tet3, Tet4, Tet5, Tet6);
            app.UITable.Data = t;
            
        end

        % Selection changed function: InputTypeButtonGroup
        function InputTypeButtonGroupSelectionChanged(app, event)
            if(app.ExampleConfigurationButton.Value)
                app.selected_conf = 1; %Example config
                app.ExampleConfigurationPanel.Enable = 'on';
                app.DesiredPosePanel.Enable          = 'off';
            else
                app.selected_conf = 2;  %Desired pose
                app.ExampleConfigurationPanel.Enable = 'off';
                app.DesiredPosePanel.Enable          = 'on';
            end
        end

        % Button pushed function: ShowonRoboDKButton
        function ShowonRoboDKButtonPushed(app, event)
            sol_num = app.SelectSolution18EditField.Value;
            app.robot.setJoints(app.tetc_deg_all(sol_num,:));
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 745 601];
            app.UIFigure.Name = 'MATLAB App';

            % Create ConnectRoboDKButton
            app.ConnectRoboDKButton = uibutton(app.UIFigure, 'push');
            app.ConnectRoboDKButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectRoboDKButtonPushed, true);
            app.ConnectRoboDKButton.BackgroundColor = [1 1 1];
            app.ConnectRoboDKButton.Position = [335 530 109 22];
            app.ConnectRoboDKButton.Text = 'Connect RoboDK';

            % Create ExampleConfigurationPanel
            app.ExampleConfigurationPanel = uipanel(app.UIFigure);
            app.ExampleConfigurationPanel.Title = 'Example Configuration';
            app.ExampleConfigurationPanel.Position = [43 326 236 168];

            % Create Tet1EditField
            app.Tet1EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet1EditField.Position = [59 112 48 22];
            app.Tet1EditField.Value = -4.57;

            % Create Tet1EditFieldLabel
            app.Tet1EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet1EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet1EditFieldLabel.Position = [16 112 28 22];
            app.Tet1EditFieldLabel.Text = 'Tet1';

            % Create Tet2EditField
            app.Tet2EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet2EditField.Position = [59 71 48 22];
            app.Tet2EditField.Value = 8.88;

            % Create Tet2EditFieldLabel
            app.Tet2EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet2EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet2EditFieldLabel.Position = [16 71 28 22];
            app.Tet2EditFieldLabel.Text = 'Tet2';

            % Create Tet3EditField
            app.Tet3EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet3EditField.Position = [59 30 48 22];
            app.Tet3EditField.Value = 17.94;

            % Create Tet3EditFieldLabel
            app.Tet3EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet3EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet3EditFieldLabel.Position = [16 30 28 22];
            app.Tet3EditFieldLabel.Text = 'Tet3';

            % Create Tet4EditField
            app.Tet4EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet4EditField.Position = [174 112 44 22];
            app.Tet4EditField.Value = 180;

            % Create Tet4EditFieldLabel
            app.Tet4EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet4EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet4EditFieldLabel.Position = [131 112 28 22];
            app.Tet4EditFieldLabel.Text = 'Tet4';

            % Create Tet5EditField
            app.Tet5EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet5EditField.Position = [174 71 44 22];
            app.Tet5EditField.Value = 61.88;

            % Create Tet5EditFieldLabel
            app.Tet5EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet5EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet5EditFieldLabel.Position = [131 71 28 22];
            app.Tet5EditFieldLabel.Text = 'Tet5';

            % Create Tet6EditFieldLabel
            app.Tet6EditFieldLabel = uilabel(app.ExampleConfigurationPanel);
            app.Tet6EditFieldLabel.HorizontalAlignment = 'right';
            app.Tet6EditFieldLabel.Position = [132 30 28 22];
            app.Tet6EditFieldLabel.Text = 'Tet6';

            % Create Tet6EditField
            app.Tet6EditField = uieditfield(app.ExampleConfigurationPanel, 'numeric');
            app.Tet6EditField.Position = [175 30 43 22];
            app.Tet6EditField.Value = -142.61;

            % Create SolveButton
            app.SolveButton = uibutton(app.UIFigure, 'push');
            app.SolveButton.ButtonPushedFcn = createCallbackFcn(app, @SolveButtonPushed, true);
            app.SolveButton.Position = [317 281 100 22];
            app.SolveButton.Text = 'Solve';

            % Create ShowonRoboDKButton
            app.ShowonRoboDKButton = uibutton(app.UIFigure, 'push');
            app.ShowonRoboDKButton.ButtonPushedFcn = createCallbackFcn(app, @ShowonRoboDKButtonPushed, true);
            app.ShowonRoboDKButton.Enable = 'off';
            app.ShowonRoboDKButton.Position = [437 19 111 22];
            app.ShowonRoboDKButton.Text = 'Show on RoboDK';

            % Create SelectSolution18EditFieldLabel
            app.SelectSolution18EditFieldLabel = uilabel(app.UIFigure);
            app.SelectSolution18EditFieldLabel.HorizontalAlignment = 'right';
            app.SelectSolution18EditFieldLabel.Position = [179 19 114 22];
            app.SelectSolution18EditFieldLabel.Text = 'Select Solution (1:8)';

            % Create SelectSolution18EditField
            app.SelectSolution18EditField = uieditfield(app.UIFigure, 'numeric');
            app.SelectSolution18EditField.Position = [308 19 100 22];
            app.SelectSolution18EditField.Value = 1;

            % Create DesiredPosePanel
            app.DesiredPosePanel = uipanel(app.UIFigure);
            app.DesiredPosePanel.Enable = 'off';
            app.DesiredPosePanel.Title = 'Desired Pose';
            app.DesiredPosePanel.Position = [317 326 405 168];

            % Create T11EditField
            app.T11EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T11EditField.Position = [59 112 46 22];
            app.T11EditField.Value = 0.8;

            % Create T11EditFieldLabel
            app.T11EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T11EditFieldLabel.HorizontalAlignment = 'right';
            app.T11EditFieldLabel.Position = [19 112 25 22];
            app.T11EditFieldLabel.Text = 'T11';

            % Create T12EditField
            app.T12EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T12EditField.Position = [161 112 41 22];
            app.T12EditField.Value = -0.3;

            % Create T12EditFieldLabel
            app.T12EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T12EditFieldLabel.HorizontalAlignment = 'right';
            app.T12EditFieldLabel.Position = [120 112 26 22];
            app.T12EditFieldLabel.Text = 'T12';

            % Create T13EditField
            app.T13EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T13EditField.Position = [252 112 40 22];
            app.T13EditField.Value = -0.5;

            % Create T13EditFieldLabel
            app.T13EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T13EditFieldLabel.HorizontalAlignment = 'right';
            app.T13EditFieldLabel.Position = [211 112 26 22];
            app.T13EditFieldLabel.Text = 'T13';

            % Create T14EditField
            app.T14EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T14EditField.Position = [348 112 40 22];
            app.T14EditField.Value = 1132;

            % Create T14EditFieldLabel
            app.T14EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T14EditFieldLabel.HorizontalAlignment = 'right';
            app.T14EditFieldLabel.Position = [307 112 26 22];
            app.T14EditFieldLabel.Text = 'T14';

            % Create T21EditField
            app.T21EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T21EditField.Position = [59 78 46 22];
            app.T21EditField.Value = -0.1;

            % Create T21EditFieldLabel
            app.T21EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T21EditFieldLabel.HorizontalAlignment = 'right';
            app.T21EditFieldLabel.Position = [18 78 26 22];
            app.T21EditFieldLabel.Text = 'T21';

            % Create T22EditField
            app.T22EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T22EditField.Position = [161 78 41 22];
            app.T22EditField.Value = 0.8;

            % Create T22EditFieldLabel
            app.T22EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T22EditFieldLabel.HorizontalAlignment = 'right';
            app.T22EditFieldLabel.Position = [120 78 26 22];
            app.T22EditFieldLabel.Text = 'T22';

            % Create T23EditField
            app.T23EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T23EditField.Position = [252 78 40 22];
            app.T23EditField.Value = -0.6;

            % Create T23EditFieldLabel
            app.T23EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T23EditFieldLabel.HorizontalAlignment = 'right';
            app.T23EditFieldLabel.Position = [211 78 26 22];
            app.T23EditFieldLabel.Text = 'T23';

            % Create T24EditField
            app.T24EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T24EditField.Position = [348 78 40 22];
            app.T24EditField.Value = -90;

            % Create T24EditFieldLabel
            app.T24EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T24EditFieldLabel.HorizontalAlignment = 'right';
            app.T24EditFieldLabel.Position = [307 78 26 22];
            app.T24EditFieldLabel.Text = 'T24';

            % Create T31EditField
            app.T31EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T31EditField.Position = [59 46 46 22];
            app.T31EditField.Value = 0.6;

            % Create T31EditFieldLabel
            app.T31EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T31EditFieldLabel.HorizontalAlignment = 'right';
            app.T31EditFieldLabel.Position = [18 46 26 22];
            app.T31EditFieldLabel.Text = 'T31';

            % Create T32EditField
            app.T32EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T32EditField.Position = [161 46 41 22];
            app.T32EditField.Value = 0.5;

            % Create T32EditFieldLabel
            app.T32EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T32EditFieldLabel.HorizontalAlignment = 'right';
            app.T32EditFieldLabel.Position = [120 46 26 22];
            app.T32EditFieldLabel.Text = 'T32';

            % Create T33EditField
            app.T33EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T33EditField.Position = [252 46 40 22];
            app.T33EditField.Value = 0.7;

            % Create T33EditFieldLabel
            app.T33EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T33EditFieldLabel.HorizontalAlignment = 'right';
            app.T33EditFieldLabel.Position = [211 46 26 22];
            app.T33EditFieldLabel.Text = 'T33';

            % Create T34EditField
            app.T34EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T34EditField.Position = [348 46 40 22];
            app.T34EditField.Value = 1145;

            % Create T34EditFieldLabel
            app.T34EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T34EditFieldLabel.HorizontalAlignment = 'right';
            app.T34EditFieldLabel.Position = [307 46 26 22];
            app.T34EditFieldLabel.Text = 'T34';

            % Create T41EditField
            app.T41EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T41EditField.Position = [59 9 46 22];

            % Create T41EditFieldLabel
            app.T41EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T41EditFieldLabel.HorizontalAlignment = 'right';
            app.T41EditFieldLabel.Position = [18 9 26 22];
            app.T41EditFieldLabel.Text = 'T41';

            % Create T42EditField
            app.T42EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T42EditField.Position = [161 9 41 22];

            % Create T42EditFieldLabel
            app.T42EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T42EditFieldLabel.HorizontalAlignment = 'right';
            app.T42EditFieldLabel.Position = [120 9 26 22];
            app.T42EditFieldLabel.Text = 'T42';

            % Create T43EditField
            app.T43EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T43EditField.Position = [252 9 40 22];

            % Create T43EditFieldLabel
            app.T43EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T43EditFieldLabel.HorizontalAlignment = 'right';
            app.T43EditFieldLabel.Position = [211 9 26 22];
            app.T43EditFieldLabel.Text = 'T43';

            % Create T44EditField
            app.T44EditField = uieditfield(app.DesiredPosePanel, 'numeric');
            app.T44EditField.Position = [348 9 40 22];
            app.T44EditField.Value = 1;

            % Create T44EditFieldLabel
            app.T44EditFieldLabel = uilabel(app.DesiredPosePanel);
            app.T44EditFieldLabel.HorizontalAlignment = 'right';
            app.T44EditFieldLabel.Position = [307 9 26 22];
            app.T44EditFieldLabel.Text = 'T44';

            % Create InputTypeButtonGroup
            app.InputTypeButtonGroup = uibuttongroup(app.UIFigure);
            app.InputTypeButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @InputTypeButtonGroupSelectionChanged, true);
            app.InputTypeButtonGroup.Title = 'Input Type';
            app.InputTypeButtonGroup.Position = [475 504 235 75];

            % Create ExampleConfigurationButton
            app.ExampleConfigurationButton = uiradiobutton(app.InputTypeButtonGroup);
            app.ExampleConfigurationButton.Text = 'Example Configuration';
            app.ExampleConfigurationButton.Position = [11 29 143 22];
            app.ExampleConfigurationButton.Value = true;

            % Create DesiredPoseButton
            app.DesiredPoseButton = uiradiobutton(app.InputTypeButtonGroup);
            app.DesiredPoseButton.Text = 'Desired Pose';
            app.DesiredPoseButton.Position = [11 7 94 22];

            % Create UITable
            app.UITable = uitable(app.UIFigure);
            app.UITable.ColumnName = {'Tet1'; 'Tet2'; 'Tet3'; 'Tet4'; 'Tet5'; 'Tet6'};
            app.UITable.RowName = {'1,'; '2,'; '3,'; '4,'; '5,'; '6,'; '7,'; '8,'};
            app.UITable.Position = [94 54 572 212];

            % Create ModellingandControlofRobotsFinalProjectLabel
            app.ModellingandControlofRobotsFinalProjectLabel = uilabel(app.UIFigure);
            app.ModellingandControlofRobotsFinalProjectLabel.FontWeight = 'bold';
            app.ModellingandControlofRobotsFinalProjectLabel.Position = [43 533 270 22];
            app.ModellingandControlofRobotsFinalProjectLabel.Text = 'Modelling and Control of Robots Final Project';

            % Create KerimMoral518191043Label
            app.KerimMoral518191043Label = uilabel(app.UIFigure);
            app.KerimMoral518191043Label.Position = [44 511 134 22];
            app.KerimMoral518191043Label.Text = 'Kerim Moral 518191043';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = final_project

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end