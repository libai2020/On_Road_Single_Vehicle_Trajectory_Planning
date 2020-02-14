function WriteGuidingLineFile(x, y, theta)
delete('GL');
fid = fopen('GL', 'w');
Nfe = length(x);
for ii = 1 : Nfe
    fprintf(fid, '%g 1  %f\r\n', ii, x(ii));
    fprintf(fid, '%g 2  %f\r\n', ii, y(ii));
    fprintf(fid, '%g 3  %f\r\n', ii, theta(ii));
end
fclose(fid);