function WriteParameters(tf, Nfe)
global BV_
delete('BV');
fid = fopen('BV', 'w');
fprintf(fid, '1  %f\r\n', BV_.x0);
fprintf(fid, '2  %f\r\n', BV_.y0);
fprintf(fid, '3  %f\r\n', BV_.theta0);
fprintf(fid, '4  %f\r\n', BV_.v0);
fprintf(fid, '5  %f\r\n', BV_.phy0);
fprintf(fid, '6  %f\r\n', tf);
fprintf(fid, '7  %f\r\n', Nfe);
fclose(fid);