% System parameters
%--------------------------------------------------------------------------
%                       Micky Paramneters
%
% Piezoactuator parameters
a1=0.001;             % time constant of the actuator' dynamics (s)
dp=1.0773;            % piezoelectric constant (µm/V)
p=1.3;                % compliance (inverse of stiffness of the actuator) (µm/mN)

alfabw=0.40648;       % parameter of hysteresis 
betabw=0.00833;       % parameter of hysteresis
gammabw=0.00833;      % parameter of hysteresis


% Deformable object parameters 
kob=1/3.738;          % stiffness of the object, a polystirene (mN/µm)
cob=1;                % damping parameter of the object (mN.s/µm)

% Force model parameters
e2=a1+p*cob;
e1=1+p*kob;

f2=dp*cob;
f1=dp*kob;

g2=cob;
g1=kob;

% a = e1/e2;    b = f1/e2;      c = f2/e2;      d = g2/e2;      e = g1/e2;
% a1 =  e1/e2;  a3 = f1/e2;     a2 = f2/e2;     a4 = g2/e2;     a5 = g1/e2;
%--------------------------------------------------------------------------

a1 = e1/e2;
a2 = f2/e2;
a3 = f1/e2; % 0.28 el problema en el control lo da cuando a3 es diferente de cero
a4 = g2/e2;
a5 = g1/e2;

Gamma = [ -a1, -a5;
           0,   0];

% Se tiene que cumplir que B >|G|
A = 0.40648; %alfabw=0.40648; %entre mas grandes estos parametros, mas picos causan
B = 0.40648; %betabw=0.00833; %----------------- ESTE LO CAMBIE, es necesario por la condicion demostrada de B>|G|
G = 0.00833; %gammabw=0.00833;

% Observer parameters
% see file "s para dimension 2.tex" in G:\Mi unidad\CIO\papers\2023\micky-force\simulaciones\calculosMatrizS



ax = 10.2;
theta = 8.8; %1.9/5; % 1.9;    3.2
lobs =  200; %1422.5/65; %la que quiero que salga 1422.5
kobs =  lobs/2 - (theta^3)/(2*ax) ; %0.124; %(lobs/2 - (theta^3/(2*a5))); %0.5*(lobs - theta^3);        % Distirbance observer gain where 450 is the proposed value kobs = - 0.65*1094.2 - (theta^3/(2*a5));

Gamma_bar = [    0,    ax,   0 ;
                 0,    0,    1 ;
              kobs,    0,    0 ];

Sinv = [    3*theta                            ,    3*((theta^2)/(ax))                     ,   (1/(ax))*(theta^3 + 2*kobs*ax) ;
            3*((theta^2)/(ax))                 ,   (1/(ax^2))*(5*theta^3 - 2*kobs*ax)      ,   (1/(ax^2))*(2*theta^4 + kobs*ax*theta); 
            (1/(ax))*(theta^3 + 2*kobs*ax)     ,   (1/(ax^2))*(2*theta^4 + kobs*ax*theta)  ,   (1/(ax^2))*(2*theta^4 + kobs*ax*theta)*((theta^4 + 2*kobs*ax*theta)/(2*theta^3 + kobs*ax))];


%Sinv = [    3*theta                            ,    3*((theta^2)/(a5))                     ,   (1/(a5))*(theta^3 + 2*kobs*a5) ;
%            3*((theta^2)/(a5))                 ,   (1/(a5^2))*(5*theta^3 - 2*kobs*a5)      ,   (1/(a5^2))*(2*theta^4 + kobs*a5*theta); 
%            (1/(a5))*(theta^3 + 2*kobs*a5)     ,   (1/(a5^2))*(2*theta^4 + kobs*a5*theta)  ,   (1/(a5^2))*(2*theta^4 + kobs*a5*theta)*((theta^4 + 2*kobs*a5*theta)/(2*theta^3 + kobs*a5))];

%Sinv = [    3*theta              ,   3*theta^2               ,   theta^3 + 2*kobs ;
%            3*theta^2            ,   5*theta^3 - 2*kobs      ,   2*theta^4 + kobs*theta; 
%            theta^3 + 2*kobs     ,   2*theta^4 + kobs*theta  ,   ((1)/(2*theta^3 + kobs)) * (theta^4 + 2*kobs*theta) * (2*theta^4 + kobs*theta)];

C = [1 , 0];
Cbar = [1, 0, 0];

% Desired trajectory parameters
omega = 1;
kappa = 0.09; %0.09

%% Barrier control parameters
l = 0.65;   %0.5  1.1
k1 = 120;  %120 proportional
k2 = 100;  %100 barrier
% proportional term sat functions
L_p = 7; %12
M_p = 7.1; %13
% barrier term sat functions
L_b = 15; % 15
M_b = 16; % 16
