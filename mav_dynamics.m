function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
   % DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 10;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    w1    = uu(7);
    w2    = uu(8);
    w3    = uu(9);
    w4    = uu(10);

    
    W = [w1; w2; w3; w4];
    
    Dist_tau = [ell; m; n];
    Dist_F = [fx; fy; fz];
    
    tau_motorGyro = [q*P.Jm*2*pi/60*(-w1-w3+w2+w4); p*P.Jm*2*pi/60*(w1+w3-w2-w4); 0]; % Note: 2*pi/60 required to convert from RPM to radians/s
    Mb = (P.dctcq*(W.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'

    % Thrust due to motor speed
    % Force should be in units of Newtons for simplicity in calculating
    % the acceleration in the angular velocity state equation
    Fb = [0; 0; sum(P.ct*(W.^2))];   %[0, 0, sum(ct*w.^2)]'

    % Obtain dP dQ dR
    omb_bi = [p; q; r];
    OMb_bi = [ 0,-r, q;
               r, 0,-p;
              -q, p, 0];

    b_omdotb_bi = P.Jbinv*(Mb-OMb_bi*P.Jb*omb_bi);

    Rib = [...
        cos(theta)*cos(psi)...
        (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))...
        (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
        cos(theta)*sin(psi)...
        (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))...
        (cos(phi)*sin(theta)*sin(psi) - sin(phi)*sin(psi));
        -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)...
    ];

    Rbi = Rib';
    ge = [0; 0; -P.gravity];
    gb = Rbi*ge;
    Dist_Fb = Rbi*Dist_F;

    % Compute Velocity and Position derivatives of body frame
    vb = [u;v;w];
    b_dv = (1/P.mass)*Fb+gb+Dist_Fb-OMb_bi*vb; % Acceleration in body frame (FOR VELOCITY)
    i_dp = Rib*vb;
    
    pndot = i_dp(1);
    pedot = i_dp(2);
    pddot = i_dp(3);
    
	udot = b_dv(1);
	vdot = b_dv(2);
	wdot = b_dv(3);
    
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    thetadot = cos(phi)*q - sin(phi)*r;
    psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    pdot = b_omdotb_bi(1);
    qdot = b_omdotb_bi(2);
    rdot = b_omdotb_bi(3);

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
