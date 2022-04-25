P.gravity = 9.8331;
   
%physical parameters of airframe
P.mass = 1.0230;
P.ct   = 2.4865e-06;
P.Jm   = 3.7882e-06;
P.Jb   = [0.0095,0,0;0,0.0095,0;0,0,0.0186];
P.Jbinv = [105.2643,0,0;0,105.2643,0;0,0,53.8329];
P.dctcq = [...
    0,3.3037e-08,0,-3.3037e-08;...
    -3.3037e-08,0,3.3037e-08,0;...
    -2.9250e-09,2.9250e-09,-2.9250e-09,2.9250e-09...
    ];

% initial conditions
P.pn0    =   0;
P.pe0    =   0;
P.pd0    =   0; % initial Down position (negative altitude)
P.u0     =   0; % initial velocity along body x-axis
P.v0     =   0; % initial velocity along body y-axis
P.w0     =   0; % initial velocity along body z-axis
P.phi0   =   0; % initial roll angle
P.theta0 =   0; % initial pitch angle
P.psi0   =   0; % initial yaw angle
P.p0     =   0; % initial body frame roll rate
P.q0     =   0; % initial body frame pitch rate
P.r0     =   0; % initial body frame yaw rate




