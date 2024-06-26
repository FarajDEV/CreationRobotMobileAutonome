
port = 'COM13';
baudrate = 230400;
s = serial(port, 'BaudRate', baudrate);
fopen(s);


x_data = [];
y_data = [];
way_points_x = [];
way_points_y = [];

x_robot = 0;
y_robot = 0;


figure;
subplot(2, 1, 1);
hold on;
title('Trajectoire de notre robot mobile autonome');
xlabel('Position x du robot');
ylabel('Position y du robot');

subplot(2, 1, 2);
hold on;
title('Calcul des waypoints');

total_steps = 100;
total_time = 10;


for step = 1:total_steps
    subplot(2, 1, 1);
    scatter(x_robot, y_robot, 'r', 'filled');
    pause(total_time / total_steps);

   
    line = fgets(s);
    if ischar(line)
        try
           
            data = sscanf(line, '%f,%f,%f,%f');
            if length(data) == 4
                x_robot = data(1);
                y_robot = data(2);
                waypoint_x = data(3);
                waypoint_y = data(4);
                
              
                x_data = [x_data, x_robot];
                y_data = [y_data, y_robot];
                way_points_x = [way_points_x, waypoint_x];
                way_points_y = [way_points_y, waypoint_y];
                
                subplot(2, 1, 1);
                scatter(x_robot, y_robot, 'r', 'filled');
                
                subplot(2, 1, 2);
                scatter(x_data, y_data, 'b', 'filled');
                scatter(waypoint_x, waypoint_y, 'b', 'filled');
                pause(0.05);
            else
                disp(['Ligne de donn?es incorrecte: ', line]);
            end
        catch ME
            disp(['Erreur lors de la conversion des donn?es: ', ME.message]);
        end
    end
end


fclose(s);
delete(s);


subplot(2, 1, 1);
hold off;
subplot(2, 1, 2);
hold off;
