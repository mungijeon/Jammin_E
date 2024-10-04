function radar_gui
    global range_estimated;
    global coor_value;

    
    plot_flag = 0;
    % Create figure, Need to setting
    hFig = figure('Name', 'Radar Simulation', 'NumberTitle', 'off', ...
                  'Color', 'k', 'MenuBar', 'none', 'ToolBar', 'none', ...
                  'Position', [0, 0, 800, 600]);
    x = hFig.Position;
    % ROS Setup
    motor_left = rospublisher('/motor_left', 'std_msgs/Int32');
    motor_left_msg = rosmessage(motor_left);
    motor_left_msg.Data = 1;
    motor_right = rospublisher('/motor_right', 'std_msgs/Int32');
    motor_right_msg = rosmessage(motor_right);
    motor_right_msg.Data = 1;
    motor_center = rospublisher('/motor_center', 'std_msgs/Int32');
    motor_center_msg = rosmessage(motor_center);
    motor_center_msg.Data = 1;
    
    % coor_value 0 1 2
    matlab_left = rossubscriber('/topic_detect', 'std_msgs/Int32', @Callback_left); 
    matlab_right = rossubscriber('/topic_theta', 'std_msgs/Int32', @Callback_right); 
    matlab_center = rossubscriber('/motor_center','std_msgs/Int32', @Callback_center);


    % Create axes
    hAxes = axes('Parent', hFig, 'Color', 'k', 'XColor', 'g', 'YColor', 'g', ...
                 'XLim', [-200, 200], 'YLim', [-200, 200], 'NextPlot', 'add');
    axis(hAxes, 'equal');
    axis(hAxes, 'off');
    
    %for plot
    sig_dB = load('sig_dB.txt');

    % Draw radar circles & angles
    theta = linspace(0, 2*pi, 100);
    for r = [50, 100, 150, 200]
        plot(hAxes, r*cos(theta), r*sin(theta), 'g');
    end

    % Draw radar lines with angles
    for angle = 0:10:350
        plot(hAxes, [0 r*cosd(angle)], [0 r*sind(angle)], 'g');
        text(cosd(angle)*1.05*r, sind(angle)*1.05*r, sprintf('%d°', angle), ...
            'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'center');
    end

    % Radar sweep line
    sweepLine = plot(hAxes, [0 1], [0 0], 'g', 'LineWidth', 2);
    previousLines = [];

     % Initialize variables for red dot
    redDot = [];
    blinkState = false; % Toggle between on/off for blinking


    % Distance & Angle Text
    distanceText = uicontrol('Style', 'text', 'String', 'Distance: ', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [x(3)-50, 120, 140, 20], 'HorizontalAlignment', 'left');
    angleText = uicontrol('Style', 'text', 'String', 'Angle: ', ...
                          'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                          'Position', [x(3)-50, 100, 140, 20], 'HorizontalAlignment', 'left');

    % On/Off Button
    toggleButton_l = uicontrol('Style', 'togglebutton', 'String', 'On', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [x(3)-50, x(4)-50, 80, 30], 'Callback', @motor_l);
    toggleButton_r = uicontrol('Style', 'togglebutton', 'String', '시작', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [x(3)-50, x(4)-100, 80, 30], 'Callback', @motor_r);
    toggleButton_c = uicontrol('Style', 'togglebutton', 'String', '开始', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [x(3)-50, x(4)-150, 80, 30], 'Callback', @motor_c);



    % Initialize
    radarOn = true;
    % detectedPosition = []; % Store detected ROS position
    angle_value = 0;  % Default angle value if not received from ROS
    %run('signal_processing.m');
    redDot = plot(hAxes, -75, 65 ,'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    set(redDot, 'Visible', 'off');
    redDot_l = plot(hAxes, -75, 65 ,'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    redDot_r = plot(hAxes, 75, 65 ,'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    redDot_c = plot(hAxes, 10, 110 ,'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
    % Animation loop
    while ishandle(hFig)
        if radarOn
            prev_range_estimated = NaN;
            
            

            for angle = 0:360
                % Add sweep line
                set(sweepLine, 'XData', [0 r*cosd(angle)]);
                set(sweepLine, 'YData', [0 r*sind(angle)]);

                % angle_value = angle;
                % set(angleText, 'String', sprintf('Angle: %.2f°', angle_value));
                
                if ~isempty(previousLines)
                    for k = 1:length(previousLines)
                        set(previousLines(k), 'Color', [0 1 0]*0.8, 'LineWidth', 2); % Fade effect
                    end
                end
                newLine = plot(hAxes, [0 r*cosd(angle)], [0 r*sind(angle)], 'g', 'LineWidth', 3);
                previousLines = [previousLines, newLine];

                % Remove old lines to avoid excessive plotting
                if length(previousLines) > 30  % Limit the number of previous lines
                    delete(previousLines(1));
                    previousLines(1) = [];
                end
                
                %{
                if range_estimated ~= prev_range_estimated
                    run('signal_processing.m');  % Run the script to update distance
                end
                
 
                
                % Update Distance
                if exist('range_estimated','var')
                    set(distanceText, 'String', sprintf('Distance: %.2f', range_estimated));
                    prev_range_estimated = range_estimated;
                else
                    set(distanceText, 'String', 'Distance: N/A');
                end

                % Check if detectedPosition is updated
                if ~isempty(detectedPosition)
                    blinkState = ~blinkState; % Toggle blink state
                    if blinkState
                        if isempty(redDot) || ~isvalid(redDot)
                            redDot = plot(hAxes, detectedPosition(1), detectedPosition(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
                        else
                            set(redDot, 'Visible', 'on');
                        end
                    else
                        set(redDot, 'Visible', 'off');
                    end
                end
                %}
                
                if coor_value == 0
                    set(redDot_l, 'Visible', 'on');
                    set(distanceText, 'String', sprintf('Distance: %.2f', 7.5));
                    set(angleText, 'String', sprintf('Angle: %.2f°', 139));
                    set(redDot_r, 'Visible', 'off');
                    set(redDot_c, 'Visible', 'off');
                    if blinkState
                        set(redDot_l, 'Visible', 'off');
                        pause(0.01);
                    end
                    blinkState = ~blinkState;
                elseif coor_value == 1
                    set(redDot_r, 'Visible', 'on');
                    set(distanceText, 'String', sprintf('Distance: %.2f', 7.5));
                    set(angleText, 'String', sprintf('Angle: %.2f°', 42));
                    set(redDot_l, 'Visible', 'off');
                    set(redDot_c, 'Visible', 'off');
                    if blinkState
                        set(redDot_r, 'Visible', 'off');
                        pause(0.01);
                    end
                    blinkState = ~blinkState;
                else
                    if plot_flag == 1
                        x = linspace(1, 2048 , 2048 );
                        plot(x, sig_dB, 'r')
                        xlim([-50 250]);  
                        ylim([6 20]);  
                        
                        xlabel('Range Bin');
                        ylabel('Magnitude (dB)');
                        title('FMCW Radar Signal Spectrum');
                        plot_flag = 0;
                    end
                    set(redDot_c, 'Visible', 'on');
                    set(distanceText, 'String', sprintf('Distance: %.2f', 8));
                    set(angleText, 'String', sprintf('Angle: %.2f°', 85));
                    set(redDot_l, 'Visible', 'off');
                    set(redDot_r, 'Visible', 'off');
                    if blinkState
                        set(redDot_c, 'Visible', 'off');
                        pause(0.01);
                    end
                    blinkState = ~blinkState;

                    
                end
                
                drawnow;
                pause(0.001); % Radar sweep speed
                
                % Check On/Off
                if ~radarOn
                    break;
                end
            end
        end
        pause(0.05);  % Infinite loop avoidance
    end

    % On/Off Toggle
    %{
    function toggleRadar(~, ~)
        radarOn = get(toggleButton, 'Value');
        if radarOn
            set(toggleButton, 'String', 'On');
        else
            set(toggleButton, 'String', 'Off');
        end
    end
    %}
    function motor_l(~, ~)
        radarOn = get(toggleButton_l, 'Value');
        send(motor_left,motor_left_msg);
        plot_flag = 1;
        if radarOn
            set(toggleButton_l, 'String', 'On');
            run('signal_processing.m');
        else
            set(toggleButton_l, 'String', 'Off');
        end
    end
    function motor_r(~, ~)
        radarOn = get(toggleButton_r, 'Value');
        send(motor_right,motor_right_msg);
        plot_flag = 1;
        if radarOn
            set(toggleButton_r, 'String', '시작');
            
        else
            set(toggleButton_r, 'String', '중지');
        end
    end
    function motor_c(~, ~)
        radarOn = get(toggleButton_c, 'Value');
        send(motor_center,motor_center_msg);
        plot_flag = 1;
        if radarOn
            set(toggleButton_c, 'String', '开始');
        else
            set(toggleButton_c, 'String', '停止');
        end
    end

    function Callback_left(~, msg)
        coor_value = msg.Data;
    end
    function Callback_right(~, msg)
        coor_value = msg.Data;
    end
    function Callback_center(~, msg)
        coor_value = msg.Data;
    end
end
