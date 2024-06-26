
SERIAL_PORT = 'COM13';
MEASUREMENTS_PER_PLOT = 480;
PLOT_MAX_RANGE = 4.0;
PLOT_AUTO_RANGE = false;
PLOT_CONFIDENCE = true;
PLOT_CONFIDENCE_COLOUR_MAP = 'jet'; 
PRINT_DEBUG = false;


PACKET_LENGTH = 47;
MEASUREMENT_LENGTH = 12;


State = {'SYNC0', 'SYNC1', 'SYNC2', 'LOCKED', 'UPDATE_PLOT'};

function [angle, distance, confidence] = parse_lidar_data(data)
    data = typecast(data, 'uint8');
    length = data(1);
    speed = data(2);
    start_angle = double(data(3) + bitshift(data(4), 8)) / 100.0;
    stop_angle = double(data(end-5) + bitshift(data(end-4), 8)) / 100.0;
    
    pos_data = data(5:end-6);
    timestamp = data(end-3) + bitshift(data(end-2), 8);
    crc = data(end);
    
    if stop_angle < start_angle
        stop_angle = stop_angle + 360.0;
    end
    step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1);
    angle = start_angle + (0:(MEASUREMENT_LENGTH-1)) * step_size;
    distance = pos_data(1:2:end);
    confidence = pos_data(2:2:end);
    
    if PRINT_DEBUG
        disp([length, speed, start_angle, pos_data, stop_angle, timestamp, crc]);
    end
end


function [x, y, confidence] = get_xyc_data(measurements)
    angle = [measurements.angle];
    distance = [measurements.distance];
    confidence = [measurements.confidence];
    x = sind(angle) .* (distance / 1000.0);
    y = cosd(angle) .* (distance / 1000.0);
end


running = true;
lidar_serial = serialport(SERIAL_PORT, 230400, 'Timeout', 0.5);
data = uint8([]);
state = State{1};
measurements = struct('angle', {}, 'distance', {}, 'confidence', {});

% Pr?paration de l'intrigue
figure;
hold on;
axis equal;
if PLOT_CONFIDENCE
    scatter_handle = scatter([], [], 10, [], 'filled');
else
    plot_handle = plot([], [], '.');
end
xlim([-PLOT_MAX_RANGE, PLOT_MAX_RANGE]);
ylim([-PLOT_MAX_RANGE, PLOT_MAX_RANGE]);


function on_plot_close(~, ~)
    global running;
    running = false;
end

addlistener(gcf, 'ObjectBeingDestroyed', @on_plot_close);


while running
    switch state
        case 'SYNC0'
            data = uint8([]);
            measurements = struct('angle', {}, 'distance', {}, 'confidence', {});
            if read(lidar_serial, 1, 'uint8') == 84 
                data = [data; 84];
                state = 'SYNC1';
            end
        case 'SYNC1'
            if read(lidar_serial, 1, 'uint8') == 44 
                state = 'SYNC2';
                data = [data; 44];
            else
                state = 'SYNC0';
            end
        case 'SYNC2'
            data = [data; read(lidar_serial, PACKET_LENGTH - 2, 'uint8')];
            if length(data) ~= PACKET_LENGTH
                state = 'SYNC0';
                continue;
            end
            [angle, distance, confidence] = parse_lidar_data(data);
            measurements(end+1).angle = angle;
            measurements(end).distance = distance;
            measurements(end).confidence = confidence;
            state = 'LOCKED';
        case 'LOCKED'
            data = read(lidar_serial, PACKET_LENGTH, 'uint8');
            if data(1) ~= 84 || length(data) ~= PACKET_LENGTH
                disp('WARNING: Serial sync lost');
                state = 'SYNC0';
                continue;
            end
            [angle, distance, confidence] = parse_lidar_data(data);
            measurements(end+1).angle = angle;
            measurements(end).distance = distance;
            measurements(end).confidence = confidence;
            if length(measurements) > MEASUREMENTS_PER_PLOT
                state = 'UPDATE_PLOT';
            end
        case 'UPDATE_PLOT'
            [x, y, c] = get_xyc_data(measurements);
            if PLOT_AUTO_RANGE
                max_val = max([max(abs(x)), max(abs(y))]) * 1.2;
                xlim([-max_val, max_val]);
                ylim([-max_val, max_val]);
            end
            if PLOT_CONFIDENCE
                scatter_handle.XData = x;
                scatter_handle.YData = y;
                scatter_handle.CData = c;
                colormap(PLOT_CONFIDENCE_COLOUR_MAP);
            else
                plot_handle.XData = x;
                plot_handle.YData = y;
            end
            drawnow;
            state = 'LOCKED';
            measurements = struct('angle', {}, 'distance', {}, 'confidence', {});
    end
end

clear lidar_serial;
