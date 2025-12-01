classdef APPV2 < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        UIAxes                     matlab.ui.control.UIAxes
        StartButton                matlab.ui.control.Button
        StopButton                 matlab.ui.control.Button

        % PID
        pField                     matlab.ui.control.NumericEditField
        iField                     matlab.ui.control.NumericEditField
        dField                     matlab.ui.control.NumericEditField
        fField                     matlab.ui.control.NumericEditField

        % Target
        TargetPosField             matlab.ui.control.NumericEditField

        % System Constants
        massField                  matlab.ui.control.NumericEditField
        spoolRadiusField           matlab.ui.control.NumericEditField
        reductionRatioField        matlab.ui.control.NumericEditField
        motorCountField            matlab.ui.control.NumericEditField
        kineticFrictionField       matlab.ui.control.NumericEditField
        staticFrictionField        matlab.ui.control.NumericEditField
        controlPeriodField         matlab.ui.control.NumericEditField
    end

    properties (Access = private)
        running logical = false
        plotHandle matlab.graphics.chart.primitive.Line
        angularPosition double = 0
        angularVelocity double = 0
        angularAcceleration double = 0
        previousError double = 0
        integralSum double = 0
        elapsedTime double = 0
    end

    properties (Constant)
        maxVoltage = 12
        freeSpeed = (5900 * 2 * pi)/60
        stallCurrent = 11
        backEMFconstant = 12 / ((5900 * 2 * pi)/60)
        armatureResistance = 12 / 11
        gravityConstant = -9.81
    end

    methods (Access = private)

        function simulateStep(app)
            % Read system constants from numeric fields
            mass = app.massField.Value;
            spoolRadius = app.spoolRadiusField.Value;
            reductionRatio = app.reductionRatioField.Value;
            motorCount = app.motorCountField.Value;
            kineticFrictionForce = app.kineticFrictionField.Value;
            staticFrictionForce = app.staticFrictionField.Value;
            controlPeriod = app.controlPeriodField.Value;

            % Read PID and target
            p = app.pField.Value;
            i = app.iField.Value;
            d = app.dField.Value;
            f = app.fField.Value;
            target = app.TargetPosField.Value;

            dt = controlPeriod;

            % Update elapsed time manually
            app.elapsedTime = app.elapsedTime + dt;
            t = app.elapsedTime;

            % Update kinematics
            app.angularPosition = app.angularPosition + dt * app.angularVelocity + 0.5 * app.angularAcceleration * dt^2;
            app.angularVelocity = app.angularVelocity + app.angularAcceleration * dt;

            position = ((app.angularPosition / (2*pi)) * reductionRatio) * 2*pi*spoolRadius*100;

            % PID
            err = target - position;
            derr = (err - app.previousError) / dt;
            app.previousError = err;

            app.integralSum = app.integralSum + err * dt;
            app.integralSum = max(min(app.integralSum,1),-1);

            control = p*err + i*app.integralSum + d*derr + f;
            appliedPower = max(min(control,1),-1);
            appliedVoltage = appliedPower * app.maxVoltage;

            % Motor physics
            backEMF = app.backEMFconstant * app.angularVelocity;
            netV = appliedVoltage - backEMF;
            current = netV / app.armatureResistance;
            motorTorque = app.backEMFconstant * current;

            motorTorqueAtShaft = (motorTorque * motorCount) / reductionRatio;
            gravityTorqueAtShaft = app.gravityConstant * mass * spoolRadius;

            if abs(app.angularVelocity) < 0.1
                friction = sign(motorTorqueAtShaft + gravityTorqueAtShaft) * staticFrictionForce * spoolRadius;
                if abs(friction) >= abs(motorTorqueAtShaft + gravityTorqueAtShaft)
                    netT = 0;
                    app.angularVelocity = 0;
                else
                    friction = sign(app.angularVelocity) * kineticFrictionForce * spoolRadius;
                    netT = motorTorqueAtShaft + gravityTorqueAtShaft + friction;
                end
            else
                friction = sign(app.angularVelocity) * kineticFrictionForce * spoolRadius;
                netT = motorTorqueAtShaft + gravityTorqueAtShaft + friction;
            end

            app.angularAcceleration = netT / (mass * spoolRadius^2);

            % Update plot
            x = get(app.plotHandle,'XData');
            y = get(app.plotHandle,'YData');
            x(end+1) = t;
            y(end+1) = position;
            set(app.plotHandle,'XData',x,'YData',y);

            % Dynamic axes
            windowTime = 5; % show last 5 seconds
            if t < windowTime
                app.UIAxes.XLim = [0 windowTime];
            else
                app.UIAxes.XLim = [t-windowTime t];
            end
            app.UIAxes.YLim = [min(y)-5 max(y)+5];

            drawnow limitrate
        end

        function runLoop(app)
            while app.running
                simulateStep(app);
                pause(app.controlPeriodField.Value);
            end
        end

        function StartButtonPushed(app, ~)
            app.running = true;
            app.angularPosition = 0;
            app.angularVelocity = 0;
            app.angularAcceleration = 0;
            app.integralSum = 0;
            app.previousError = 0;
            app.elapsedTime = 0;
            set(app.plotHandle,'XData',0,'YData',0);
            runLoop(app);
        end

        function StopButtonPushed(app, ~)
            app.running = false;
        end
    end

    methods (Access = private)

        function createComponents(app)
            % Create figure
            app.UIFigure = uifigure('Name','PID Visualizer','Position',[100 100 900 600],'Color',[0.2 0.2 0.2]);

            % Axes
            app.UIAxes = uiaxes(app.UIFigure,'Position',[50 250 500 300]);
            app.UIAxes.Color = [0.1 0.1 0.1];
            app.UIAxes.XColor = [1 1 1]; app.UIAxes.YColor = [1 1 1];
            app.UIAxes.GridColor = [0.5 0.5 0.5];
            title(app.UIAxes,'Position vs Time','Color',[1 1 1]); xlabel(app.UIAxes,'Time (s)','Color',[1 1 1]); ylabel(app.UIAxes,'Position (cm)','Color',[1 1 1]);
            app.plotHandle = plot(app.UIAxes,0,0,'Color',[0 1 0],'LineWidth',2);

            % Start/Stop buttons
            app.StartButton = uibutton(app.UIFigure,'push','Position',[600 500 100 40],...
                'Text','Start','ButtonPushedFcn',@(src,event) StartButtonPushed(app),...
                'BackgroundColor',[0.6 0.6 0.6],'FontWeight','bold');
            app.StopButton = uibutton(app.UIFigure,'push','Position',[720 500 100 40],...
                'Text','Stop','ButtonPushedFcn',@(src,event) StopButtonPushed(app),...
                'BackgroundColor',[0.6 0.6 0.6],'FontWeight','bold');

            % Grouping positions
            baseX = 600; baseY = 450; spacingY = 40;

            % PID Labels and fields
            uilabel(app.UIFigure,'Position',[baseX-40 baseY 30 25],'Text','P','FontColor',[1 1 1]);
            app.pField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY 100 25],'Value',0.014,'Limits',[0 Inf],'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-40 baseY-spacingY 30 25],'Text','I','FontColor',[1 1 1]);
            app.iField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-spacingY 100 25],'Value',0.03,'Limits',[0 Inf],'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-40 baseY-2*spacingY 30 25],'Text','D','FontColor',[1 1 1]);
            app.dField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-2*spacingY 100 25],'Value',0.0006,'Limits',[0 Inf],'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-40 baseY-3*spacingY 30 25],'Text','F','FontColor',[1 1 1]);
            app.fField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-3*spacingY 100 25],'Value',0.07,'Limits',[0 Inf],'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            % Target
            uilabel(app.UIFigure,'Position',[baseX-80 baseY-4*spacingY 80 25],'Text','Target (cm)','FontColor',[1 1 1]);
            app.TargetPosField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-4*spacingY 100 25],'Value',100,'Limits',[0 200],'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            % System constants
            uilabel(app.UIFigure,'Position',[baseX-120 baseY-5*spacingY 120 25],'Text','Mass (kg)','FontColor',[1 1 1]);
            app.massField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-5*spacingY 100 25],'Value',1.2,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-6*spacingY 120 25],'Text','Spool Radius (m)','FontColor',[1 1 1]);
            app.spoolRadiusField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-6*spacingY 100 25],'Value',0.02,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-7*spacingY 120 25],'Text','Reduction Ratio','FontColor',[1 1 1]);
            app.reductionRatioField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-7*spacingY 100 25],'Value',12/60,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-8*spacingY 120 25],'Text','Motor Count','FontColor',[1 1 1]);
            app.motorCountField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-8*spacingY 100 25],'Value',2,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-9*spacingY 120 25],'Text','Kinetic Friction (N)','FontColor',[1 1 1]);
            app.kineticFrictionField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-9*spacingY 100 25],'Value',0.5*9.81*1.2,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-10*spacingY 120 25],'Text','Static Friction (N)','FontColor',[1 1 1]);
            app.staticFrictionField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-10*spacingY 100 25],'Value',1.5*9.81*1.2,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

            uilabel(app.UIFigure,'Position',[baseX-120 baseY-11*spacingY 120 25],'Text','Control Period (s)','FontColor',[1 1 1]);
            app.controlPeriodField = uieditfield(app.UIFigure,'numeric','Position',[baseX baseY-11*spacingY 100 25],'Value',0.026,'BackgroundColor',[0.3 0.3 0.3],'FontColor',[1 1 1]);

        end
    end

    methods (Access = public)
        function app = APPV2
            createComponents(app);
        end
    end
end