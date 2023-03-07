clear all; clc;
dir = '/home/mxcheng/Documents/needle_steering/ros/bags/';
name = 'ls30mm_tx3mm_ls30mm_00';
%name = 'linear_insertion_60mm_00';

path = sprintf('%s%s/%s_0.db3', dir, name, name);

bag = ros2bagreader(path);
%baginfo = ros2("bag","info",file)

demo_select = select(bag,"Topic","/demo_point");
points = readMessages(demo_select);

N = length(points);
STEP = 0.1; % mm

inputs = points{1}.inputs';
outputs = points{1}.outputs';
prev_idx = 1;

for i = 2 : length(points)
    cur_dis = norm(points{i}.inputs - points{prev_idx}.inputs);
    if cur_dis >= STEP
        disp(points{i}.inputs)
        inputs = [inputs; points{i}.inputs'];
        outputs = [outputs; points{i}.outputs'];
        prev_idx = i;
    end
end
inputs = inputs';
outputs = outputs';

outputs = outputs * 1000.0;

N = size(outputs, 2);

j_est = [[-1, 0, 0];[0, 1, 0];[0, 0, -1]]';
errors = zeros(N - 1, 1);
disp(j_est)

dets = zeros(N, 1);
dets(1) = det(j_est);
for i = 1 : N - 1
    dx = inputs(:, i + 1) - inputs(:, i);
    dy = outputs(:, i + 1) - outputs(:, i);
    
    y_est = j_est * dx + outputs(:, i);
    errors(i) = norm(y_est - outputs(:, i + 1));
    j_est = j_est + (dy - j_est * dx) * dx' / (dx' * dx);
    dets(i + 1) = det(j_est);
end

disp(j_est)
disp(dets)

steps = linspace(1, N - 1, N - 1);
plot(steps, errors)
saveas(gcf, "errors.png")
%figure
%plot(steps, dets(2:N), "*");
%grid on;
