function image_processing_app
    fig = uifigure('Name', 'Image Processing App', 'Position', [100, 100, 700, 500]);
    global currentImage;
    currentImage = [];

    % UI Elements
    uilabel(fig, 'Text', 'Choose Processing Type:', 'Position', [20, 450, 200, 20]);
    uidropdown(fig, 'Items', {'Pixel Manipulation', 'Geometric Transformation'},...
        'Position', [200, 450, 200, 22], 'ValueChangedFcn', @selectionChanged);
    imgAxes = uiaxes(fig, 'Position', [400, 100, 250, 250]);
    uibutton(fig, 'Text', 'Upload Image', 'Position', [20, 400, 100, 30], 'ButtonPushedFcn', @uploadImage);

    function uploadImage(~, ~)
        [file, path] = uigetfile({'*.jpg;*.png;*.bmp', 'Image Files'});
        if isequal(file, 0), return; end
        currentImage = imread(fullfile(path, file));
        imshow(currentImage, 'Parent', imgAxes);
    end

    function selectionChanged(~, event)
        delete(findall(fig, 'Type', 'uipanel'));
        if strcmp(event.Value, 'Pixel Manipulation')
            showPixelManipulationUI();
        else
            showGeometricTransformationUI();
        end
    end

    %% Pixel Manipulation UI
    function showPixelManipulationUI()
        delete(findall(fig, 'Type', 'uipanel', 'Title', 'Pixel Parameters'));
        pnl = uipanel(fig, 'Title', 'Pixel Parameters', 'Position', [20, 100, 350, 280]);

        uilabel(pnl, 'Text', 'Brightness:', 'Position', [10, 220, 80, 22]);
        uislider(pnl, 'Position', [100, 230, 200, 3], 'Limits', [-100, 100], 'Value', 0, 'Tag', 'brightness');

        uilabel(pnl, 'Text', 'Contrast:', 'Position', [10, 170, 80, 22]);
        uislider(pnl, 'Position', [100, 180, 200, 3], 'Limits', [0.1, 3], 'Value', 1, 'Tag', 'contrast');

        uicheckbox(pnl, 'Text', 'Invert Colors', 'Position', [10, 130], 'Tag', 'invert');
        uicheckbox(pnl, 'Text', 'Convert to Grayscale', 'Position', [150, 130], 'Tag', 'grayscale');

        uibutton(pnl, 'Text', 'Apply', 'Position', [230, 20, 70, 30], ...
            'ButtonPushedFcn', @(~, ~)applyPixelManipulation());
    end

    function applyPixelManipulation()
        if isempty(currentImage)
            uialert(fig, 'Upload an image first!', 'Error');
            return;
        end

        img = double(currentImage);

        brightness = findobj(fig, 'Tag', 'brightness').Value;
        contrast = findobj(fig, 'Tag', 'contrast').Value;
        invert = findobj(fig, 'Tag', 'invert').Value;
        grayscale = findobj(fig, 'Tag', 'grayscale').Value;

        img = contrast * img + brightness;

        if invert
            img = 255 - img;
        end

        if grayscale && size(img,3) == 3
            img = mean(img, 3);
        end

        img = uint8(max(min(img, 255), 0));
        imshow(img, 'Parent', imgAxes);
    end

    %% Geometric Transformation UI
    function showGeometricTransformationUI()
        pnl = uipanel(fig, 'Title', 'Geometric Transformation', 'Position', [20, 100, 350, 280]);
        uilabel(pnl, 'Text', 'Choose Type:', 'Position', [10, 230, 100, 20]);
        uidropdown(pnl, 'Items', {'Polar', 'Twirl', 'Ripple'},...
            'Position', [120, 230, 200, 22], 'ValueChangedFcn', @geoTransformSelection);
    end

    function geoTransformSelection(src, event)
        delete(findall(fig, 'Type', 'uipanel', 'Title', 'Parameters'));
        pnlParams = uipanel(fig, 'Title', 'Parameters', 'Position', [20, 50, 350, 120]);

        type = event.Value;

        switch type
            case 'Polar'
                createInputField(pnlParams, 'r', [10, 50]);
                createInputField(pnlParams, 'theta', [120, 50]);

            case 'Twirl'
                createInputField(pnlParams, 'theta', [10, 50]);
                createInputField(pnlParams, 'dely', [120, 50]);
                createInputField(pnlParams, 'delx', [10, 20]);
                createInputField(pnlParams, 'phi', [120, 20]);
                createInputField(pnlParams, 'rmax', [230, 50]);
                createInputField(pnlParams, 'r', [230, 20]);
                createInputField(pnlParams, 'x', [10, -10]);
                createInputField(pnlParams, 'y', [120, -10]);
                createInputField(pnlParams, 'xc', [230, -10]);
                createInputField(pnlParams, 'yc', [10, -40]);

            case 'Ripple'
                createInputField(pnlParams, 'x', [10, 50]);
                createInputField(pnlParams, 'y', [120, 50]);
                createInputField(pnlParams, 'xp', [10, 20]);     % x’
                createInputField(pnlParams, 'yp', [120, 20]);    % y’
                createInputField(pnlParams, 'ax', [230, 50]);
                createInputField(pnlParams, 'ay', [230, 20]);
                createInputField(pnlParams, 'wx', [10, -10]);
                createInputField(pnlParams, 'wy', [120, -10]);
        end

        uibutton(pnlParams, 'Text', 'Apply', 'Position', [250, -40, 70, 30], ...
            'ButtonPushedFcn', @(~, ~)applyGeoTransformation(type));
    end

    function createInputField(panel, label, position)
        uilabel(panel, 'Text', [label ':'], 'Position', [position(1), position(2), 50, 20]);
        uieditfield(panel, 'numeric', 'Position', [position(1) + 50, position(2), 50, 22], 'Tag', label);
    end

    function applyGeoTransformation(type)
        if isempty(currentImage)
            uialert(fig, 'Upload an image first!', 'Error');
            return;
        end

        img = double(currentImage);
        [h, w, c] = size(img);

        fields = findall(fig, 'Type', 'uieditfield');
        values = struct();
        for i = 1:numel(fields)
            values.(fields(i).Tag) = str2double(fields(i).Value);
        end

        switch type
            case 'Polar'
                r_val = values.r;
                theta_val = values.theta;
                [X, Y] = meshgrid(1:w, 1:h);
                Xc = w / 2; Yc = h / 2;
                r = sqrt((X - Xc).^2 + (Y - Yc).^2) / r_val;
                theta = atan2(Y - Yc, X - Xc) + theta_val;
                newX = r .* cos(theta) * r_val + Xc;
                newY = r .* sin(theta) * r_val + Yc;

            case 'Twirl'
                Xc = values.xc; Yc = values.yc;
                rmax = values.rmax;
                phi = values.phi;
                [X, Y] = meshgrid(1:w, 1:h);
                dx = X - Xc;
                dy = Y - Yc;
                r = sqrt(dx.^2 + dy.^2);
                angle = atan2(dy, dx) + phi * (rmax - r) ./ rmax;
                newX = r .* cos(angle) + Xc;
                newY = r .* sin(angle) + Yc;

            case 'Ripple'
                ax = values.ax; ay = values.ay;
                wx = values.wx; wy = values.wy;
                [X, Y] = meshgrid(1:w, 1:h);
                newX = X + ax * sin(wx * Y);
                newY = Y + ay * cos(wy * X);
        end

        result = zeros(size(img));
        for i = 1:c
            result(:,:,i) = interp2(double(img(:,:,i)), newX, newY, 'linear', 0);
        end
        result = uint8(result);
        imshow(result, 'Parent', imgAxes);
    end
end
