function robot = build_manual_humanoid_tree_BFreversed()
%BUILD_MANUAL_HUMANOID_TREE Build the humanoid rigidBodyTree manually.
%
% The tree is always constructed in the physical RBT direction:
%       world -> floating base -> torso -> distal bodies
%
% For each Simscape Revolute Joint, enter:
%   1) TbeforeSS: the ordered product of all Rigid Transform blocks from the
%      RBT parent body toward the joint block.
%   2) TafterSS:  the ordered product of all Rigid Transform blocks from the
%      joint block toward the RBT child body.
%   3) axisSS:    the positive Simscape joint axis, expressed in the joint
%      frame after TbeforeSS.
%
% IMPORTANT FOR FLIPPED SIMSCAPE B/F CONNECTIONS
% ------------------------------------------------
% If the RBT parent reaches the Simscape F port and the RBT child leaves the
% Simscape B port, set bfFlipped = true.
%
% Because TbeforeSS and TafterSS are entered while walking from the RBT
% parent to the RBT child, they remain in the same order. Only the joint
% motion is traversed in the opposite direction. This implementation keeps
%
%       qRBT = qSimscape
%
% by using
%
%       axisRBT = -axisSS
%
% for flipped joints. Consequently, the torque mapping for those joints is
%
%       tauSimscape = -tauRBT
%
% assuming the Simscape torque input follows the joint's positive B-to-F
% actuation convention.
%
% After construction, this function also assigns inertial properties and
% joint limits.

    robot = rigidBodyTree( ...
        'DataFormat', 'row', ...
        'MaxNumBodies', 120);

    robot.BaseName = 'world';
    robot.Gravity = [0 0 -9.80665];

    %% Floating base
    addFloatingBase(robot);

    %% Torso, neck, and head
    addFixed(robot, 'torso',   'trunk',    eye(4));
    addFixed(robot, 'trunk',   'shoulder', Ttrans([0 0 -0.0399] * 3.5));
    addFixed(robot, 'shoulder','neck',     Ttrans([0 0  0.01016] * 3.5));
    addFixed(robot, 'neck',    'head',     Ttrans([0 0  0.06414] * 3.5));
    addFixed(robot, 'head',    'cover',    eye(4));

    addFixed(robot, 'torso', 'left_shoulder_base',  eye(4));
    addFixed(robot, 'torso', 'right_shoulder_base', eye(4));

    %% Right arm
    % Simscape B/F direction agrees with RBT parent/child direction.
    T_origin = Ttrans([-0.06413 0 -0.0399] * 3.5) ...
             * TrotXYZ([0 0 pi]);
    T_axis = Taxang([0 1 0], pi/2);
    TbeforeSS = T_origin * T_axis;
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_shoulder_base', ...
        'right_shoulder_frontal_frame', ...
        'right_shoulder_swivel', ...
        'right_shoulder_frontal', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    T_origin = Ttrans([0.01905 0 0] * 3.5) ...
             * TrotXYZ([0 0 -pi/2]);
    T_axis = Taxang([0 1 0], pi/2);
    TbeforeSS = T_origin * T_axis;
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_shoulder_swivel', ...
        'right_shoulder_sagittal_frame', ...
        'right_upper_arm', ...
        'right_shoulder_sagittal', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    addFixed(robot, 'right_upper_arm', 'right_lower_arm', ...
        Ttrans([0 0 -0.0889] * 3.5));
    addFixed(robot, 'right_lower_arm', 'right_hand', ...
        Ttrans([0 0 -0.0762] * 3.5));

    %% Left arm
    % B/F FLIPPED in Simscape for left_shoulder_frontal.
    %
    % Enter TbeforeSS and TafterSS in the visual/physical order from:
    % left_shoulder_base -> joint -> left_shoulder_swivel.
    TbeforeSS = Ttrans([0.06413 0 -0.0399] * 3.5) ...
              * TrotXYZ([0 0 0]) ...
              * Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_shoulder_base', ...
        'left_shoulder_frontal_frame', ...
        'left_shoulder_swivel', ...
        'left_shoulder_frontal', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    % B/F not flipped for left_shoulder_sagittal.
    TbeforeSS = Ttrans([0.01905 0 0] * 3.5) ...
              * TrotXYZ([0 0 -1.57075])...
              * Taxang([0 1 0], pi/2) ;
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_shoulder_swivel', ...
        'left_shoulder_sagittal_frame', ...
        'left_upper_arm', ...
        'left_shoulder_sagittal', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    addFixed(robot, 'left_upper_arm', 'left_lower_arm', ...
        Ttrans([0 0 -0.0889] * 3.5));
    addFixed(robot, 'left_lower_arm', 'left_hand', ...
        Ttrans([0 0 -0.0762] * 3.5));

    %% Hip bases
    addFixed(robot, 'torso', 'left_hip_base',  eye(4));
    addFixed(robot, 'torso', 'right_hip_base', eye(4));

    addFixed(robot, 'right_hip_base', 'right_hip', ...
        Ttrans([-0.01905 0 -0.1849] * 3.5));

    addFixed(robot, 'left_hip_base', 'left_hip', ...
        Ttrans([0.01905 0 -0.1849] * 3.5) ...
        * TrotXYZ([0 0 pi]));

    %% Right leg
    % B/F direction agrees with RBT parent/child direction for all joints.
    TbeforeSS = Taxang([0 0 1], pi/2);
    TafterSS  = Taxang([0 0 -1], pi/2);

    addSimscapeRevolute(robot, ...
        'right_hip', ...
        'right_hip_transverse_frame', ...
        'right_hip_transverse_link', ...
        'right_hip_transverse', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    TbeforeSS = Ttrans([0 0 -0.02286] * 3.5) ...
              * Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_hip_transverse_link', ...
        'right_hip_sagittal_frame', ...
        'right_hip_frontal_link', ...
        'right_hip_frontal', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    TbeforeSS = Taxang([-1 0 0], pi/2);
    TafterSS  = Taxang([1 0 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_hip_frontal_link', ...
        'right_hip_frontal_frame', ...
        'right_upper_leg', ...
        'right_hip_sagittal', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    TbeforeSS = Ttrans([-0.02537 0 -0.1041] * 3.5) ...
              * Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_upper_leg', ...
        'right_knee_frame', ...
        'right_lower_leg', ...
        'right_knee', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    TbeforeSS = Ttrans([0 0 -0.10414] * 3.5) ...
              * Taxang([1 0 0], pi/2);
    TafterSS  = Taxang([-1 0 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_lower_leg', ...
        'right_ankle_roll_frame', ...
        'right_ankle_roll_link', ...
        'right_ankle_roll', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    TbeforeSS = Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'right_ankle_roll_link', ...
        'right_ankle_pitch_frame', ...
        'right_foot', ...
        'right_ankle_pitch', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    %% Left leg
    % B/F FLIPPED:
    %   left_hip_transverse
    %   left_hip_frontal
    %   left_hip_sagittal
    %   left_knee
    %   left_ankle_roll
    %
    % B/F NOT flipped:
    %   left_ankle_pitch
    %
    % Replace each TbeforeSS/TafterSS below with the exact ordered products
    % read from Simscape while walking from the RBT parent toward the child.

    TbeforeSS = Taxang([0 0 1], pi/2);
    TafterSS  = Taxang([0 0 -1], pi/2);

    addSimscapeRevolute(robot, ...
        'left_hip', ...
        'left_hip_transverse_frame', ...
        'left_hip_transverse_link', ...
        'left_hip_transverse', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    TbeforeSS = Ttrans([0 0 -0.02286] * 3.5) ...
              * Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_hip_transverse_link', ...
        'left_hip_sagittal_frame', ...
        'left_hip_frontal_link', ...
        'left_hip_frontal', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    TbeforeSS = Taxang([1 0 0], pi/2);
    TafterSS  = Taxang([-1 0 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_hip_frontal_link', ...
        'left_hip_frontal_frame', ...
        'left_upper_leg', ...
        'left_hip_sagittal', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    TbeforeSS = Ttrans([-0.02537 0 -0.1041] * 3.5) ...
              * Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_upper_leg', ...
        'left_knee_frame', ...
        'left_lower_leg', ...
        'left_knee', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    TbeforeSS = Ttrans([0 0 -0.10414] * 3.5) ...
              * TrotXYZ([0 0 pi]) ...
              * Taxang([1 0 0], pi/2);
    TafterSS  = Taxang([-1 0 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_lower_leg', ...
        'left_ankle_roll_frame', ...
        'left_ankle_roll_link', ...
        'left_ankle_roll', ...
        [0 0 1], TbeforeSS, TafterSS, true);

    TbeforeSS = Taxang([0 1 0], pi/2);
    TafterSS  = Taxang([0 -1 0], pi/2);

    addSimscapeRevolute(robot, ...
        'left_ankle_roll_link', ...
        'left_ankle_pitch_frame', ...
        'left_foot', ...
        'left_ankle_pitch', ...
        [0 0 1], TbeforeSS, TafterSS, false);

    %% Dynamic properties and limits
    robot = addInertialProperties(robot);
    robot = addJointLimits(robot);

    %% Basic structural checks
    validateRobotDefinition(robot);
end


%% ========================================================================
%  Construction helpers
%  ========================================================================

function addFloatingBase(robot)
%ADDFLOATINGBASE Six-DOF serial floating base retained intentionally.
%
% The orientation is represented by three serial revolute joints:
% roll about x, then pitch about the moving y axis, then yaw about the
% moving z axis. This convention must later be checked against Simscape.

    addPrismatic(robot, 'world',      'base_x',     'floating_x',     [1 0 0], eye(4));
    addPrismatic(robot, 'base_x',     'base_y',     'floating_y',     [0 1 0], eye(4));
    addPrismatic(robot, 'base_y',     'base_z',     'floating_z',     [0 0 1], eye(4));

    addRevolute(robot,  'base_z',     'base_roll',  'floating_roll',  [1 0 0], eye(4));
    addRevolute(robot,  'base_roll',  'base_pitch', 'floating_pitch', [0 1 0], eye(4));
    addRevolute(robot,  'base_pitch', 'base_yaw',   'floating_yaw',   [0 0 1], eye(4));

    addFixed(robot, 'base_yaw', 'torso', eye(4));
end


function addSimscapeRevolute(robot, parentName, jointFrameName, ...
    childBodyName, jointName, axisSS, TbeforeSS, TafterSS, bfFlipped)
%ADDSIMSCAPEREVOLUTE Add one Simscape-style revolute connection.
%
% Inputs TbeforeSS and TafterSS must be entered in RBT parent-to-child order:
%
%   parent -- TbeforeSS -- [Revolute Joint] -- TafterSS -- child
%
% For bfFlipped == false:
%   parent is connected to Simscape B and child to Simscape F.
%
% For bfFlipped == true:
%   parent is connected to Simscape F and child to Simscape B.
%
% The latter traverses the joint motion in reverse. To keep qRBT=qSimscape,
% the RBT joint axis is negated. The surrounding transforms are NOT swapped
% or inverted because they were supplied along the parent-to-child path.

    validateTransform(TbeforeSS, [jointName ' TbeforeSS']);
    validateTransform(TafterSS,  [jointName ' TafterSS']);

    axisSS = validateAxis(axisSS, jointName);

    if bfFlipped
        axisRBT = -axisSS;
    else
        axisRBT = axisSS;
    end

    addRevoluteWithReverse(robot, ...
        parentName, jointFrameName, childBodyName, jointName, ...
        axisRBT, TbeforeSS, TafterSS);
end


function addFixed(robot, parentName, bodyName, T)
    validateTransform(T, [bodyName ' fixed transform']);

    body = rigidBody(char(bodyName));
    joint = rigidBodyJoint([char(bodyName) '_fixed'], 'fixed');
    setFixedTransform(joint, T);

    body.Joint = joint;
    addBody(robot, body, char(parentName));
end


function addRevolute(robot, parentName, bodyName, jointName, axis, T)
    validateTransform(T, [jointName ' fixed transform']);
    axis = validateAxis(axis, jointName);

    body = rigidBody(char(bodyName));
    joint = rigidBodyJoint(char(jointName), 'revolute');

    joint.JointAxis = axis;
    setFixedTransform(joint, T);

    body.Joint = joint;
    addBody(robot, body, char(parentName));
end


function addRevoluteWithReverse(robot, parentName, jointFrameName, ...
    childBodyName, jointName, axis, Tbefore, Tafter)

    addRevolute(robot, parentName, jointFrameName, ...
        jointName, axis, Tbefore);

    addFixed(robot, jointFrameName, childBodyName, Tafter);
end


function addPrismatic(robot, parentName, bodyName, jointName, axis, T)
    validateTransform(T, [jointName ' fixed transform']);
    axis = validateAxis(axis, jointName);

    body = rigidBody(char(bodyName));
    joint = rigidBodyJoint(char(jointName), 'prismatic');

    joint.JointAxis = axis;
    setFixedTransform(joint, T);

    body.Joint = joint;
    addBody(robot, body, char(parentName));
end


%% ========================================================================
%  Transform helpers
%  ========================================================================

function T = Ttrans(v)
    validateattributes(v, {'numeric'}, ...
        {'real','finite','vector','numel',3}, mfilename, 'v');

    T = eye(4);
    T(1:3,4) = v(:);
end


function T = TrotXYZ(angles)
%TROTXYZ Homogeneous rotation with the explicit multiplication convention
%
%       R = Rx(rx) * Ry(ry) * Rz(rz)
%
% This function name is retained for compatibility with the existing code.
% For combined nonzero angles, this exact convention must match the intended
% Simscape transform convention.

    validateattributes(angles, {'numeric'}, ...
        {'real','finite','vector','numel',3}, mfilename, 'angles');

    rx = angles(1);
    ry = angles(2);
    rz = angles(3);

    Rx = [1 0 0;
          0 cos(rx) -sin(rx);
          0 sin(rx)  cos(rx)];

    Ry = [ cos(ry) 0 sin(ry);
           0       1 0;
          -sin(ry) 0 cos(ry)];

    Rz = [cos(rz) -sin(rz) 0;
          sin(rz)  cos(rz) 0;
          0        0       1];

    T = eye(4);
    T(1:3,1:3) = Rx * Ry * Rz;
end


function T = Taxang(axis, angle)
    axis = validateAxis(axis, 'Taxang axis');
    validateattributes(angle, {'numeric'}, ...
        {'real','finite','scalar'}, mfilename, 'angle');

    x = axis(1);
    y = axis(2);
    z = axis(3);

    c = cos(angle);
    s = sin(angle);
    C = 1 - c;

    R = [x*x*C + c,     x*y*C - z*s, x*z*C + y*s;
         y*x*C + z*s,   y*y*C + c,   y*z*C - x*s;
         z*x*C - y*s,   z*y*C + x*s, z*z*C + c];

    T = eye(4);
    T(1:3,1:3) = R;
end


%% ========================================================================
%  Inertial properties
%  ========================================================================

function robot = addInertialProperties(robot)
%ADDINERTIALPROPERTIES Assign physical properties to physical bodies.
%
% MATLAB rigidBody inertia vector format:
%   [Ixx Iyy Izz Iyz Ixz Ixy]
%
% Intermediate joint-frame bodies remain massless.

    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        body.Mass = 0;
        body.CenterOfMass = [0 0 0];
        body.Inertia = [0 0 0 0 0 0];
    end

    setInertia(robot, 'head', 0.286647, ...
        [2.98169e-08 0.00620398 2.13925e-11], ...
        [0.00244307 0.00292309 0.00248477 ...
         1.97838e-13 -1.58967e-13 1.60639e-09]);

    setInertia(robot, 'cover', 0.124819, ...
        [6.96832e-18 -0.085344 1.1046e-18], ...
        [0.000367032 0.000733999 0.000367032 ...
         1.2727e-22 1.11376e-20 7.29614e-23]);

    setInertia(robot, 'neck', 0.606552, ...
        [2.68537e-05 -2.09991e-10 0.0658928], ...
        [0.0021397 0.00311945 0.00119046 ...
         1.56603e-10 -1.63667e-06 2.76267e-12]);

    setInertia(robot, 'shoulder', 1.41504, ...
        [-7.94378e-08 1.75447e-06 0.0094544], ...
        [0.00624334 0.0173966 0.0165061 ...
         -5.17626e-08 6.04639e-09 1.26112e-08]);

    setInertia(robot, 'trunk', 10.6217, ...
        [-2.53296e-07 1.30444e-06 -0.383125], ...
        [0.470031 0.681824 0.236631 ...
         -2.98473e-06 8.11326e-07 -4.20428e-08]);

    armSides = {'right','left'};
    for i = 1:numel(armSides)
        side = armSides{i};

        setInertia(robot, [side '_shoulder_swivel'], 0.756449, ...
            [0.0295772 -9.22951e-09 -9.74786e-12], ...
            [0.000721147 0.00163563 0.00214801 ...
             6.61465e-15 -1.96918e-14 -3.93647e-10]);

        setInertia(robot, [side '_upper_arm'], 2.32659, ...
            [6.17021e-08 1.60168e-05 -0.135928], ...
            [0.0459993 0.047961 0.00468923 ...
             -3.93717e-06 -3.09814e-08 -1.38551e-08]);

        setInertia(robot, [side '_lower_arm'], 1.69064, ...
            [-6.30985e-08 1.12643e-05 -0.143204], ...
            [0.0233111 0.0246313 0.00273955 ...
             -1.98282e-06 6.09942e-09 -5.6614e-09]);

        setInertia(robot, [side '_hand'], 1.50779, ...
            [1.8213e-05 0.000275676 -0.0693325], ...
            [0.00706 0.00794255 0.00340665 ...
             -0.000956389 5.97493e-07 -3.97862e-07]);
    end

    legSides = {'right','left'};
    for i = 1:numel(legSides)
        side = legSides{i};

        setInertia(robot, [side '_hip'], 1.28494, ...
            [-0.0140332 -1.4823e-07 -0.0406876], ...
            [0.00627536 0.00496338 0.00362544 ...
             -7.4315e-09 -0.000830537 -5.01291e-09]);

        setInertia(robot, [side '_upper_leg'], 2.74188, ...
            [-0.0866304 -5.0233e-09 -0.219689], ...
            [0.0778737 0.0818775 0.00954932 ...
             -1.55588e-08 -0.0013674 -4.30887e-08]);

        setInertia(robot, [side '_lower_leg'], 2.44251, ...
            [-4.27848e-08 -8.93215e-08 -0.183112], ...
            [0.0700741 0.0718714 0.00630666 ...
             -6.57897e-08 -1.05119e-07 -1.47988e-08]);

        setInertia(robot, [side '_foot'], 1.96977, ...
            [-3.33087e-08 -0.0441461 -0.0401732], ...
            [0.0240119 0.00723767 0.0218786 ...
             -0.00427709 -7.33023e-09 -1.11162e-09]);
    end
end


% function setInertia(robot, bodyName, mass, com, inertia)
%     validateattributes(mass, {'numeric'}, ...
%         {'real','finite','scalar','nonnegative'}, mfilename, 'mass');
%     validateattributes(com, {'numeric'}, ...
%         {'real','finite','vector','numel',3}, mfilename, 'com');
%     validateattributes(inertia, {'numeric'}, ...
%         {'real','finite','vector','numel',6}, mfilename, 'inertia');
% 
%     body = getBody(robot, bodyName);
%     body.Mass = mass;
%     body.CenterOfMass = reshape(com, 1, 3);
%     body.Inertia = reshape(inertia, 1, 6);
% end

function setInertia(robot,bodyName,mass,com,inertiaAtCOM)

    body = getBody(robot,bodyName);

    com = reshape(com,1,3);

    Ic = inertiaVectorToMatrix(inertiaAtCOM);

    r = com(:);

    Io = Ic + mass * ...
        ((r.'*r)*eye(3) - r*r.');

    body.Mass = mass;
    body.CenterOfMass = com;

    body.Inertia = [ ...
        Io(1,1), ...
        Io(2,2), ...
        Io(3,3), ...
        Io(2,3), ...
        Io(1,3), ...
        Io(1,2)];
end
function I = inertiaVectorToMatrix(v)

    I = [v(1) v(6) v(5);
         v(6) v(2) v(4);
         v(5) v(4) v(3)];
end

%% ========================================================================
%  Joint limits
%  ========================================================================

function robot = addJointLimits(robot)
%ADDJOINTLIMITS Limits are specified in the RBT coordinate convention.
%
% Because flipped B/F joints use axisRBT=-axisSS, this script keeps
% qRBT=qSimscape; therefore the same numerical Simscape position limits can
% be entered here without swapping or negating them.

    setLimit(robot, 'right_hip_frontal', -90, 30);
    setLimit(robot, 'left_hip_frontal',  -90, 30);

    setLimit(robot, 'right_knee', 0, 90);
    setLimit(robot, 'left_knee',  0, 90);

    setLimit(robot, 'right_ankle_roll',  -20, 20);
    setLimit(robot, 'left_ankle_roll',   -20, 20);
    setLimit(robot, 'right_ankle_pitch', -20, 20);
    setLimit(robot, 'left_ankle_pitch',  -20, 20);

    setLimit(robot, 'right_shoulder_frontal', -30, 110);
    setLimit(robot, 'left_shoulder_frontal',  -30, 110);

    setLimit(robot, 'right_shoulder_sagittal', -30, 90);
    setLimit(robot, 'left_shoulder_sagittal',  -30, 90);
end


function setLimit(robot, jointName, lowerDeg, upperDeg)
    validateattributes(lowerDeg, {'numeric'}, ...
        {'real','finite','scalar'}, mfilename, 'lowerDeg');
    validateattributes(upperDeg, {'numeric'}, ...
        {'real','finite','scalar','>',lowerDeg}, mfilename, 'upperDeg');

    for i = 1:robot.NumBodies
        joint = robot.Bodies{i}.Joint;

        if strcmp(joint.Name, jointName)
            if ~strcmp(joint.Type, 'revolute')
                error('Joint "%s" is not revolute.', jointName);
            end

            joint.PositionLimits = deg2rad([lowerDeg upperDeg]);
            return
        end
    end

    error('Joint not found: %s', jointName);
end


%% ========================================================================
%  Validation helpers
%  ========================================================================

function axis = validateAxis(axis, label)
    validateattributes(axis, {'numeric'}, ...
        {'real','finite','vector','numel',3}, mfilename, label);

    axis = reshape(axis, 1, 3);
    axisNorm = norm(axis);

    if axisNorm <= 1e-12
        error('%s must be a nonzero 3-vector.', label);
    end

    axis = axis / axisNorm;
end


function validateTransform(T, label)
    validateattributes(T, {'numeric'}, ...
        {'real','finite','size',[4 4]}, mfilename, label);

    tol = 1e-9;

    if norm(T(4,:) - [0 0 0 1], inf) > tol
        error('%s has an invalid homogeneous-transform last row.', label);
    end

    R = T(1:3,1:3);

    if norm(R.' * R - eye(3), 'fro') > 1e-8
        error('%s rotation block is not orthonormal.', label);
    end

    if abs(det(R) - 1) > 1e-8
        error('%s rotation block must have determinant +1.', label);
    end
end


function validateRobotDefinition(robot)
%VALIDATEROBOTDEFINITION Catch common construction mistakes immediately.

    bodyNames = string(robot.BodyNames);
    if numel(unique(bodyNames)) ~= numel(bodyNames)
        error('The rigidBodyTree contains duplicate body names.');
    end

    jointNames = strings(robot.NumBodies,1);
    for i = 1:robot.NumBodies
        jointNames(i) = string(robot.Bodies{i}.Joint.Name);
    end

    if numel(unique(jointNames)) ~= numel(jointNames)
        error('The rigidBodyTree contains duplicate joint names.');
    end

    physicalBodies = [ ...
        "head","cover","neck","shoulder","trunk", ...
        "right_shoulder_swivel","left_shoulder_swivel", ...
        "right_upper_arm","left_upper_arm", ...
        "right_lower_arm","left_lower_arm", ...
        "right_hand","left_hand", ...
        "right_hip","left_hip", ...
        "right_upper_leg","left_upper_leg", ...
        "right_lower_leg","left_lower_leg", ...
        "right_foot","left_foot"];

    for name = physicalBodies
        body = getBody(robot, char(name));

        if body.Mass <= 0
            error('Physical body "%s" has nonpositive mass.', name);
        end

        if any(body.Inertia(1:3) <= 0)
            error('Physical body "%s" has nonpositive principal inertia entries.', name);
        end
    end
end
