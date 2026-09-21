function IK_visualizer()
% IK_VISUALIZER  3-DOF leg inverse-kinematics visualization with UI controls.
% Run by typing: IK_visualizer

    % Parameters
    L2 = 8;
    L3 = 12;
    TARGET_X = 5;
    TARGET_Y = 8;
    TARGET_Z = -12;
    KNEE_SIGN = 1;

    % Figure and axes
    f = figure('Name', '3-DOF Leg Inverse Kinematics Visualization', 'Position', [100 100 950 600]);
    ax = axes('Parent', f, 'Units', 'pixels', 'Position', [50 50 500 500]);
    R_max = L2 + L3 + 5;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X Axis');
    ylabel(ax, 'Y Axis (Hip Rotation)');
    zlabel(ax, 'Z Axis (Vertical)');
    xlim(ax, [-R_max, R_max]);
    ylim(ax, [-R_max, R_max]);
    zlim(ax, [-R_max, R_max]);
    view(ax, 30, 30);

    % UI text
    uicontrol('Style', 'text', 'String', 'IK Results:', ...
              'Position', [570 480 300 20], 'FontWeight', 'bold', 'FontSize', 12, 'HorizontalAlignment', 'left');
    h_result_text = uicontrol('Style', 'text', 'String', 'Angles will appear here...', ...
                              'Position', [570 350 300 150], 'HorizontalAlignment', 'left', 'FontName', 'FixedWidth');

    % Sliders and edits
    control_start_y = 390;
    control_height = 40;
    slider_data = cell(1,3);
    for i = 1:3
        switch i
            case 1, label = 'Target X'; initial_val = TARGET_X;
            case 2, label = 'Target Y'; initial_val = TARGET_Y;
            case 3, label = 'Target Z'; initial_val = TARGET_Z;
        end
        y_pos = control_start_y - i * control_height * 1.5;
        slider_struct = struct();
        slider_struct.label = label;
        slider_struct.min_val = -R_max;
        slider_struct.max_val = R_max;
        slider_struct.initial_val = initial_val;
        uicontrol('Style', 'text', 'String', slider_struct.label, ...
                  'Position', [570 y_pos 80 control_height], 'HorizontalAlignment', 'left');
        slider_struct.handle = uicontrol('Style', 'slider', ...
                                         'Min', slider_struct.min_val, 'Max', slider_struct.max_val, ...
                                         'Value', slider_struct.initial_val, ...
                                         'Position', [650 y_pos 150 control_height], ...
                                         'Callback', @update_plot_callback, ...
                                         'Tag', num2str(i));
        slider_struct.edit_handle = uicontrol('Style', 'edit', ...
                                              'String', num2str(slider_struct.initial_val), ...
                                              'Position', [810 y_pos 50 control_height], ...
                                              'Callback', @update_plot_callback, ...
                                              'Tag', num2str(i));
        slider_data{i} = slider_struct;
    end

    % Knee toggle
    knee_btn = uicontrol('Style', 'togglebutton', 'String', 'Knee Up', ...
                         'Position', [570 control_start_y - 7*control_height 120 30], ...
                         'BackgroundColor', [0.8 0.8 0.8], ...
                         'Value', KNEE_SIGN < 0, ...
                         'Callback', @update_plot_callback);

    % Store data
    ud = struct('L2', L2, 'L3', L3, 'KNEE_SIGN', KNEE_SIGN, ...
                'slider_data', {slider_data}, 'ax', ax, ...
                'h_result_text', h_result_text, 'h_plot', [], ...
                'h_arcs', {cell(1,5)}, 'h_labels', {cell(1,6)}, ...
                'h_target_marker', [], 'h_knee_btn', knee_btn);
    set(f, 'UserData', ud);

    % Initial draw
    update_plot(f);

    % Nested callbacks and functions
    function update_plot_callback(hObj, ~)
        fig = ancestor(hObj, 'figure');
        data = get(fig, 'UserData');
        style = get(hObj, 'Style');
        if strcmp(style, 'slider')
            tag = str2double(get(hObj, 'Tag'));
            if ~isnan(tag) && tag >= 1 && tag <= numel(data.slider_data)
                val = get(hObj, 'Value');
                set(data.slider_data{tag}.edit_handle, 'String', num2str(val, 2));
            end
        elseif strcmp(style, 'edit')
            tag = str2double(get(hObj, 'Tag'));
            if ~isnan(tag) && tag >= 1 && tag <= numel(data.slider_data)
                val = str2double(get(hObj, 'String'));
                if isnan(val) || isinf(val)
                    current_slider_val = get(data.slider_data{tag}.handle, 'Value');
                    set(hObj, 'String', num2str(current_slider_val, 2));
                    return;
                end
                min_val = get(data.slider_data{tag}.handle, 'Min');
                max_val = get(data.slider_data{tag}.handle, 'Max');
                val = max(min_val, min(max_val, val));
                set(data.slider_data{tag}.handle, 'Value', val);
                set(hObj, 'String', num2str(val, 2));
            end
        elseif strcmp(style, 'togglebutton')
            if get(hObj, 'Value')
                data.KNEE_SIGN = -1;
                set(hObj, 'String', 'Knee Down');
            else
                data.KNEE_SIGN = 1;
                set(hObj, 'String', 'Knee Up');
            end
            set(fig, 'UserData', data);
        end
        update_plot(fig);
    end

    function update_plot(fig)
        data = get(fig, 'UserData');
        ax = data.ax;
        L2 = data.L2;
        L3 = data.L3;
        KNEE_SIGN = data.KNEE_SIGN;
        TARGET_X = get(data.slider_data{1}.handle, 'Value');
        TARGET_Y = get(data.slider_data{2}.handle, 'Value');
        TARGET_Z = get(data.slider_data{3}.handle, 'Value');

        [thetah, thetat, thetas] = ik_leg(TARGET_X, TARGET_Y, TARGET_Z, L2, L3, KNEE_SIGN);

        % Joint positions
        P0 = [0;0;0];
        X1p = L2 * cos(thetat);
        Z1p = L2 * sin(thetat);
        P1 = [X1p * cos(thetah); X1p * sin(thetah); Z1p];

        X2p = L2 * cos(thetat) + L3 * cos(thetat + thetas);
        Z2p = L2 * sin(thetat) + L3 * sin(thetat + thetas);
        P2 = [X2p * cos(thetah); X2p * sin(thetah); Z2p];

        X_coords = [P0(1), P1(1), P2(1)];
        Y_coords = [P0(2), P1(2), P2(2)];
        Z_coords = [P0(3), P1(3), P2(3)];

        % Clear previous graphics
        if ~isempty(data.h_plot) && all(ishandle(data.h_plot))
            delete(data.h_plot);
        end
        if ~isempty(data.h_target_marker) && ishandle(data.h_target_marker)
            delete(data.h_target_marker);
        end
        for j = 1:numel(data.h_arcs)
            h = data.h_arcs{j};
            if ~isempty(h) && ishandle(h), delete(h); end
            data.h_arcs{j} = [];
        end
        for k = 1:numel(data.h_labels)
            h = data.h_labels{k};
            if ~isempty(h) && ishandle(h), delete(h); end
            data.h_labels{k} = [];
        end

        % Draw links and target
        data.h_plot = plot3(ax, X_coords, Y_coords, Z_coords, 'b-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        data.h_target_marker = plot3(ax, TARGET_X, TARGET_Y, TARGET_Z, 'r*', 'MarkerSize', 12, 'LineWidth', 2);

        % Arcs for angles
        arc_radius_H = L2/4;
        arc_radius_T = L2/3.5;
        arc_radius_S = L3/4;

        angles_h = linspace(0, thetah, 50);
        X_arc_h = P0(1) + arc_radius_H * cos(angles_h);
        Y_arc_h = P0(2) + arc_radius_H * sin(angles_h);
        Z_arc_h = (P0(3) + 0.1) * ones(1, numel(angles_h));
        data.h_arcs{1} = plot3(ax, X_arc_h, Y_arc_h, Z_arc_h, '-', 'Color', 'g', 'LineWidth', 2);

        [data.h_arcs{2}, xT, yT, zT] = draw_arc_3d_local(ax, P0, arc_radius_T, 0, thetat, thetah, [0.85 0.32 0.09]);
        [data.h_arcs{3}, xS, yS, zS] = draw_arc_3d_local(ax, P1, arc_radius_S, thetat, thetat + thetas, thetah, 'm');

        % Labels
        mid_angle_h = thetah/2;
        X_text_h = (arc_radius_H + 1.5) * cos(mid_angle_h);
        Y_text_h = (arc_radius_H + 1.5) * sin(mid_angle_h);
        data.h_labels{1} = text(ax, X_text_h, Y_text_h, 1.5, ...
                                 sprintf('θ_H: %.1f°', rad2deg(thetah)), ...
                                 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        data.h_labels{2} = text(ax, xT, yT, zT, ...
                                 sprintf('θ_T: %.1f°', rad2deg(thetat)), ...
                                 'Color', [0.85 0.32 0.09], 'FontSize', 12, 'FontWeight', 'bold');
        data.h_labels{3} = text(ax, xS, yS, zS, ...
                                 sprintf('θ_S: %.1f°', rad2deg(thetas)), ...
                                 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
        data.h_labels{4} = text(ax, TARGET_X, TARGET_Y, TARGET_Z, ' Target', 'Color', 'r', 'FontSize', 12);
        data.h_labels{5} = text(ax, P0(1), P0(2), P0(3), ' Hip (Base)', 'FontSize', 12, 'FontWeight', 'bold');

        title(ax, sprintf('3-DOF Leg IK (Knee Sign: %d)', KNEE_SIGN));
        try
            subtitle(ax, sprintf('Target: (%.1f, %.1f, %.1f) | L2=%.1f, L3=%.1f', TARGET_X, TARGET_Y, TARGET_Z, L2, L3));
        catch
            % Older MATLAB versions may not support subtitle
        end

        if KNEE_SIGN > 0
            knee_text = 'Knee Up (+1)';
        else
            knee_text = 'Knee Down (-1)';
        end

        output_text = sprintf(['Target (X, Y, Z): (%.2f, %.2f, %.2f)\n', ...
                              'Link Lengths (L2, L3): (%.1f, %.1f)\n', ...
                              'Knee Configuration: %s\n', ...
                              '----------------------------\n', ...
                              'Theta Hip (thetah): %.2f degrees\n', ...
                              'Theta Thigh (thetat): %.2f degrees\n', ...
                              'Theta Shin (thetas): %.2f degrees\n'], ...
                              TARGET_X, TARGET_Y, TARGET_Z, L2, L3, ...
                              knee_text, rad2deg(thetah), rad2deg(thetat), rad2deg(thetas));
        set(data.h_result_text, 'String', output_text);
        set(fig, 'UserData', data);
        drawnow;
    end

    function [thetah, thetat, thetas] = ik_leg(x,y,z,l2,l3,kneeSign)
        % Simple planar IK after projecting to hip rotation
        r = sqrt(x*x + y*y);
        thetah = atan2(y, x);
        xp = r;
        zp = z;
        d2 = xp*xp + zp*zp;
        d = sqrt(d2);
        % Check reachability
        if d > (l2 + l3) || d < abs(l2 - l3)
            thetah = 0; thetat = 0; thetas = 0;
            return;
        end
        D = (d2 - l2*l2 - l3*l3) / (2*l2*l3);
        D = max(-1, min(1, D));
        s = kneeSign * sqrt(max(0, 1 - D*D));
        thetas = atan2(s, D);
        phi = atan2(zp, xp);
        psi = atan2(l3*sin(thetas), l2 + l3*cos(thetas));
        thetat = phi - psi;
    end
end

% Local helper function file-level (separate from main function)
function [h_plot, X_text, Y_text, Z_text] = draw_arc_3d_local(ax, CenterP, radius, start_angle, end_angle, thetaH_yaw, color)
    % DRAW_ARC_3D_LOCAL  Draw an arc in 3D rotated about Z by thetaH_yaw.
    num_points = 50;
    angles = linspace(start_angle, end_angle, num_points);
    X_arc_2D = radius * cos(angles);
    Z_arc_2D = radius * sin(angles);
    Y_arc_2D = zeros(1, num_points);
    Rz = [cos(thetaH_yaw), -sin(thetaH_yaw), 0;
          sin(thetaH_yaw),  cos(thetaH_yaw), 0;
          0,                0,               1];
    P_arc_3D = Rz * [X_arc_2D; Y_arc_2D; Z_arc_2D];
    X_arc = P_arc_3D(1, :) + CenterP(1);
    Y_arc = P_arc_3D(2, :) + CenterP(2);
    Z_arc = P_arc_3D(3, :) + CenterP(3);
    h_plot = plot3(ax, X_arc, Y_arc, Z_arc, '-', 'Color', color, 'LineWidth', 2);
    mid_angle = (start_angle + end_angle) / 2;
    X_text = CenterP(1) + (radius + 1) * cos(mid_angle) * cos(thetaH_yaw);
    Y_text = CenterP(2) + (radius + 1) * cos(mid_angle) * sin(thetaH_yaw);
    Z_text = CenterP(3) + (radius + 1) * sin(mid_angle);
end
