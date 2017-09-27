%% startup arduino
a = arduino('com4', 'Mega2560', 'Libraries', 'Servo');
 
%% connect to port
s = servo(a, 'D5');

%% write value to servo morter
Base_angle = 90;
Min_angle = -40;
Max_angle = 40;

for angle = Min_angle:10:Max_angle
    Wangle = (angle+Base_angle)/180;
    writePosition(s, Wangle);
    current_pos = readPosition(s);
    current_pos = current_pos*180;
    fprintf('Current motor position is %d degrees\n', current_pos);
    pause(1);
end
Wangle = (Base_angle)/180;
writePosition(s, Wangle);

%% 
% close session
delete(a)
