function [BVr, BVf, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta)
NE = length(x);
global vehicle_geometrics_
xr = x + vehicle_geometrics_.r2x .* cos(theta);
yr = y + vehicle_geometrics_.r2x .* sin(theta);
xf = x + vehicle_geometrics_.f2x .* cos(theta);
yf = y + vehicle_geometrics_.f2x .* sin(theta);
BVr = zeros(NE,4); BVf = zeros(NE,4); % xmin, xmax, ymin, ymax

delete('CC');
fid = fopen('CC', 'w');
for ii = 1 : NE
    x = xr(ii); y = yr(ii);
    lb = GetBoxVertexes(x, y, ii);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.1;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.1;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.1;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.1;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge, ii);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVr(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xr(ii) = x; yr(ii) = y;
    
    x = xf(ii); y = yf(ii);
    lb = GetBoxVertexes(x, y, ii);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.1;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.1;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.1;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.1;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge, ii);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVf(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xf(ii) = x; yf(ii) = y;
    fprintf(fid, '%g 1 %f \r\n', ii, BVr(ii,1));
    fprintf(fid, '%g 2 %f \r\n', ii, BVr(ii,2));
    fprintf(fid, '%g 3 %f \r\n', ii, BVr(ii,3));
    fprintf(fid, '%g 4 %f \r\n', ii, BVr(ii,4));
    fprintf(fid, '%g 5 %f \r\n', ii, BVf(ii,1));
    fprintf(fid, '%g 6 %f \r\n', ii, BVf(ii,2));
    fprintf(fid, '%g 7 %f \r\n', ii, BVf(ii,3));
    fprintf(fid, '%g 8 %f \r\n', ii, BVf(ii,4));
end
fclose(fid);
end

function lb = GetBoxVertexes(x,y,time_index)
global vehicle_geometrics_
% up left down right
basic_step = 0.1;
max_step = 4.0;
lb = ones(1,4) .* vehicle_geometrics_.radius;
if (~IsBoxValid(x,y,time_index,lb))
    lb = zeros(1,4);
    return;
end
is_completed = zeros(1,4);
while (sum(is_completed) < 4)
    for ind = 1 : 4
        if (is_completed(ind))
            continue;
        end
        test = lb;
        if (test(ind) + basic_step > max_step)
            is_completed(ind) = 1;
            continue;
        end
        test(ind) = test(ind) + basic_step;
        if (IsCurrentEnlargementValid(x, y, test, lb, ind, time_index))
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
lb = lb - vehicle_geometrics_.radius;
end

function is_valid = IsCurrentEnlargementValid(x, y, test, lb, ind, time_index)
switch ind
    case 1
        A = [x - lb(2), y + lb(1)];
        B = [x + lb(4), y + lb(1)];
        EA = [x - test(2), y + test(1)];
        EB = [x + test(4), y + test(1)];
        V_check = [A; B; EB; EA];
    case 2
        A = [x - lb(2), y + lb(1)];
        D = [x - lb(2), y - lb(3)];
        EA = [x - test(2), y + test(1)];
        ED = [x - test(2), y - test(3)];
        V_check = [A; D; ED; EA];
    case 3
        C = [x + lb(4), y - lb(3)];
        D = [x - lb(2), y - lb(3)];
        EC = [x + test(4), y - test(3)];
        ED = [x - test(2), y - test(3)];
        V_check = [C; D; ED; EC];
    case 4
        B = [x + lb(4), y + lb(1)];
        C = [x + lb(4), y - lb(3)];
        EB = [x + test(4), y + test(1)];
        EC = [x + test(4), y - test(3)];
        V_check = [C; B; EB; EC];
    otherwise
        is_valid = 0;
        return;
end
global obstacles_ road_barriers_
obs_x = road_barriers_.x; obs_y = road_barriers_.y;
err_distance = abs(obs_x - x) + abs(obs_y - y);
ind = find(err_distance <= 15);
obs_x = obs_x(ind); obs_y = obs_y(ind);
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 20)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehiclePolygonGrids(cur_obs_x, cur_obs_y, cur_obs_theta);
    obs_x = [obs_x, V.x]; obs_y = [obs_y, V.y];
end
if (any(inpolygon(obs_x, obs_y, V_check(:,1)', V_check(:,2)')))
    is_valid = 0;
else
    is_valid = 1;
end
end

function V = CreateVehiclePolygonGrids(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V.x = [linspace(AX, BX, 4), linspace(BX, CX, 4), linspace(CX, DX, 4), linspace(DX, AX, 4)];
V.y = [linspace(AY, BY, 4), linspace(BY, CY, 4), linspace(CY, DY, 4), linspace(DY, AY, 4)];
end

function is_valid = IsBoxValid(x, y, time_index, lb)
C = [x + lb(4), y - lb(3)];
D = [x - lb(2), y - lb(3)];
A = [x - lb(2), y + lb(1)];
B = [x + lb(4), y + lb(1)];
Vx = [A(1), B(1), C(1), D(1), A(1)];
Vy = [A(2), B(2), C(2), D(2), A(2)];
global obstacles_ road_barriers_
obs_x = road_barriers_.x;
obs_y = road_barriers_.y;
err_distance = abs(obs_x - x) + abs(obs_y - y);
ind = find(err_distance <= 10);
obs_x = obs_x(ind);
obs_y = obs_y(ind);
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 20)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehiclePolygonGrids(cur_obs_x, cur_obs_y, cur_obs_theta);
    obs_x = [obs_x, V.x]; obs_y = [obs_y, V.y];
end
if (any(inpolygon(obs_x, obs_y, Vx, Vy)))
    is_valid = 0;
else
    is_valid = 1;
end
end