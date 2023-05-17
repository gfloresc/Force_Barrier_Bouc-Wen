% %% Piezoactuator parameters[ ORIGINAL SET ]
alfa1 = 1;              % parameter of actuator'dynamic (s)
alfa2 = 0.001;          % time constant of the actuator' dynamics (s)

dp = 1.0773;            % piezoelectric constant (µm/V)
p = 1.3;                % compliance (inverse of stiffness of the actuator) (µm/mN)

% Deformable object parameters 
kob = 1/3.738;          % stiffness of the object, a polystirene (mN/µm)
cob = 1;                % damping parameter of the object (mN.s/µm)

% Hysteresis parameters ( Se tiene que cumplir que B >|G| )
A = 0.40648; %alfabw=0.40648; %entre mas grandes estos parametros, mas picos causan
B = 0.40648; %betabw=0.00833; %----------------- ESTE LO CAMBIE, es necesario por la condicion demostrada de B>|G|
G = 0.00833; %gammabw=0.00833;

% %% SET-2
% alfa1 = 1;      % parameter of actuator'dynamic (s)
% alfa2 = 0.001;  % time constant of the actuator' dynamics (s)
% 
% dp = 1.11;      % piezoelectric constant (µm/V)
% p = 1.8;        % compliance (inverse of stiffness of actuator) (µm/mN)
% 
% % Deformable object parameters 
% kob = 1/1.5;    % stiffness of object 2 (mN/µm)
% cob = 1;        % damping parameter of object(mN.s/µm)
% 
% % parameter of hysteresis
% A = 0.11;
% B = 0.0084;
% G = 0.0084;

% %% SET-3
% alfa1 = 1;      % parameter of actuator'dynamic (s)
% alfa2 = 0.001;  % time constant of the actuator' dynamics (s)
% 
% dp = 0.98;      % piezoelectric constant (µm/V)
% p = 1.5;        % compliance (inverse of stiffness of actuator) (µm/mN)
% 
% % Deformable object parameters 
% kob = 1/2;      % stiffness of object 2 (mN/µm)
% cob = 1;        % damping parameter of object(mN.s/µm)
% 
% % parameter of hysteresis
% A = 0.108;
% B = 0.0079;
% G = 0.0079;

% %% SET-4 
% alfa1 = 1;      % parameter of actuator'dynamic (s)
% alfa2 = 0.001;  % time constant of the actuator' dynamics (s)
% 
% dp = 1.0773;      % piezoelectric constant (µm/V)
% p = 1.3;        % compliance (inverse of stiffness of actuator) (µm/mN)
% 
% % Deformable object parameters 
% kob = 1/3.738;      % stiffness of object 2 (mN/µm)
% cob = 1;        % damping parameter of object(mN.s/µm)
% 
% % parameter of hysteresis ( Se tiene que cumplir que B >|G| )
% A = 0.10648;
% B = 0.00843;
% G = 0.00833;

%% Force model parameters [computation]
e1 = alfa1 + p*kob;
e2 = alfa2 + p*cob;
f1 = dp*kob;
f2 = dp*cob;
g1 = kob;
g2 = cob;

a1 = e1/e2;
a2 = f2/e2;
a3 = f1/e2; % 0.28 el problema en el control lo da cuando a3 es diferente de cero
a4 = g2/e2;
a5 = g1/e2;

Gamma = [ -a1, -a5;
           0,   0];

%% Observer parameters
% see file "s para dimension 2.tex" in G:\Mi unidad\CIO\papers\2023\micky-force\simulaciones\calculosMatrizS

ax = 10.2; % este realmente es "lobs" en el paper
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
l = 0.65;   %0.65
k1 = 120;   %120 proportional
k2 = 100;   %100 barrier
% proportional term sat functions
L_p = 7;    %7
M_p = 7.1;  %7.1
% barrier term sat functions
L_b = 15;   % 15
M_b = 16;   % 16

% %% Barrier control parameters [Para comparativa con el par de trabajos de Micky]
% l = 0.15;   %0.65
% k1 = 120/2;   %120 proportional
% k2 = 100/2;   %100 barrier
% % proportional term sat functions
% L_p = 7/3;    %7
% M_p = 7.1/3;  %7.1
% % barrier term sat functions
% L_b = 15/3;   % 15
% M_b = 16/3;   % 16

%% Comparativaparametros
k1_pi = 20;     % termino proporcional
k2_pi = 1.3;    % termino integral

barbeta = 12.5;   % ganancia del termino signo
baralpha1 = 8;  % termino integrativo de la superficie




