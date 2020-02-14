param BV{i in {1..7}};
param Nfe = BV[7];
param tf = BV[6];
param hi = tf / Nfe;
set I := {1..Nfe};
param CC{i in I, j in {1..8}};

param amax == 5.0;
param vmax == 25.0;
param wmax == 1.0;
param phymax == 0.7;

param Lfc == 2.5877;
param Lrc == 0.2432;
param L_wheelbase == 2.8;

var x{i in I};
var y{i in I};
var theta{i in I};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};
var xf{i in I};
var yf{i in I};
var xr{i in I};
var yr{i in I};

minimize obj:
sum{i in {2..Nfe}}((x[i] - x[i-1] - hi * v[i] * cos(theta[i]))^2 + (y[i] - y[i-1] - hi * v[i] * sin(theta[i]))^2 + (v[i] - v[i-1] - hi * a[i])^2 + (theta[i] - theta[i-1] - hi * tan(phy[i]) * v[i] / L_wheelbase)^2 + (phy[i] - phy[i-1] - hi * w[i])^2 + (xf[i] - x[i] - Lfc * cos(theta[i]))^2 + (yf[i] - y[i] - Lfc * sin(theta[i]))^2 + (xr[i] - x[i] - Lrc * cos(theta[i]))^2 + (yr[i] - y[i] - Lrc * sin(theta[i]))^2);

s.t. Rear_disc_x {i in I}:
CC[i,1] <= xr[i] <= CC[i,2];
s.t. Rear_disc_y {i in I}:
CC[i,3] <= yr[i] <= CC[i,4];
s.t. Front_disc_x {i in I}:
CC[i,5] <= xf[i] <= CC[i,6];
s.t. Front_disc_y {i in I}:
CC[i,7] <= yf[i] <= CC[i,8];

s.t. EQ_init_x :
x[1] = BV[1];
s.t. EQ_init_y :
y[1] = BV[2];
s.t. EQ_init_theta :
theta[1] = BV[3];
s.t. EQ_init_phy :
phy[1] = BV[5];
s.t. EQ_init_w :
w[1] = 0;
s.t. EQ_init_v :
v[1] = BV[4];
s.t. EQ_init_a :
a[1] = 0;

s.t. Bonds_phy {i in I}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_a {i in I}:
-amax <= a[i] <= amax;
s.t. Bonds_v {i in I}:
0 <= v[i] <= vmax;
s.t. Bonds_w {i in I}:
-wmax <= w[i] <= wmax;

data;
param: BV := include BV;
param: CC := include CC;