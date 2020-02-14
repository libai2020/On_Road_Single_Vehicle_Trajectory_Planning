function road_barriers_ = GenerateRoadBarrierGrids()
global BV_
llx = []; lly = []; uux = []; uuy = [];
ds = 1.0; d = BV_.s0 - 10;
while (1)
    [xr, yr, lb, rb, theta] = ProvideReferenceLineInfo(d);
    d = d + ds;
    llx = [llx, xr - lb * cos(pi/2 + theta)];
    lly = [lly, yr - lb * sin(pi/2 + theta)];
    uux = [uux, xr - rb * cos(pi/2 + theta)];
    uuy = [uuy, yr - rb * sin(pi/2 + theta)];
    if (d >= 200)
        break;
    end
end
road_barriers_.x = [llx, uux];
road_barriers_.y = [lly, uuy];
end