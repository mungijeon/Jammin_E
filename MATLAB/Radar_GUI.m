function radar_gui
    global range_estimated;

    % Create figure
    hFig = figure('Name', 'Radar Simulation', 'NumberTitle', 'off', ...
                  'Color', 'k', 'MenuBar', 'none', 'ToolBar', 'none', ...
                  'Position', [100, 100, 600, 600]);

    % Create axes
    hAxes = axes('Parent', hFig, 'Color', 'k', 'XColor', 'g', 'YColor', 'g', ...
                 'XLim', [-1 1], 'YLim', [-1 1], 'NextPlot', 'add');
    axis(hAxes, 'equal');
    axis(hAxes, 'off');

    % Draw radar circles & angles
    theta = linspace(0, 2*pi, 100);
    for r = [0.25, 0.5, 0.75, 1]
        plot(hAxes, r*cos(theta), r*sin(theta), 'g');
    end

    % Draw radar lines with angles
    for angle = 0:10:350
        plot(hAxes, [0 cosd(angle)], [0 sind(angle)], 'g');
        text(cosd(angle)*1.05, sind(angle)*1.05, sprintf('%d°', angle), ...
            'Color', 'g', 'FontSize', 8, 'HorizontalAlignment', 'center');
    end

    % Radar sweep line
    sweepLine = plot(hAxes, [0 1], [0 0], 'g', 'LineWidth', 2);
    previousLines = [];

    % Distance & Angle Text
    distanceText = uicontrol('Style', 'text', 'String', 'Distance: ', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [450, 50, 140, 20], 'HorizontalAlignment', 'left');
    angleText = uicontrol('Style', 'text', 'String', 'Angle: ', ...
                          'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                          'Position', [450, 30, 140, 20], 'HorizontalAlignment', 'left');

    % On/Off Button
    toggleButton = uicontrol('Style', 'togglebutton', 'String', 'On', ...
                             'ForegroundColor', 'g', 'BackgroundColor', 'k', ...
                             'Position', [500, 550, 80, 30], 'Callback', @toggleRadar);

    % Initialize
    radarOn = true;
    %run('signal_processing.m');

    % Animation loop
    while ishandle(hFig)
        if radarOn
            prev_range_estimated = NaN;
            for angle = 0:360
                % Add sweep line
                set(sweepLine, 'XData', [0 cosd(angle)]);
                set(sweepLine, 'YData', [0 sind(angle)]);

                angle_value = angle;
                set(angleText, 'String', sprintf('Angle: %.2f°', angle_value));
                
                if ~isempty(previousLines)
                    for k = 1:length(previousLines)
                        set(previousLines(k), 'Color', [0 1 0]*0.8, 'LineWidth', 2); % Fade effect
                    end
                end
                newLine = plot(hAxes, [0 cosd(angle)], [0 sind(angle)], 'g', 'LineWidth', 3);
                previousLines = [previousLines, newLine];

                % Remove old lines to avoid excessive plotting
                if length(previousLines) > 30  % Limit the number of previous lines
                    delete(previousLines(1));
                    previousLines(1) = [];
                end


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
    function toggleRadar(~, ~)
        radarOn = get(toggleButton, 'Value');
        if radarOn
            set(toggleButton, 'String', 'On');
        else
            set(toggleButton, 'String', 'Off');
        end
    end
end
