%% validate_manual_humanoid_tree.m
% Comprehensive diagnostic script for the manually built humanoid RBT.
%
% Checks construction, naming, transforms, joints, inertias, symmetry,
% perturbation response, and basic dynamics self-consistency.
%
% IMPORTANT: this validates the RBT internally. Exact equivalence to
% Simscape still requires comparing logged Simscape transforms and torques.

clear
clc
close all

%% User options
jointTestAngleDeg = 10;
showZeroConfiguration = true;
showEveryJointFigure = false;
animateEveryJoint = true;
animationPause = 0.12;

% Expected mirror pairs at q = 0. Reflection assumed across YZ plane.
symmetryPairs = {
    'right_shoulder_base',         'left_shoulder_base'
    'right_shoulder_swivel',       'left_shoulder_swivel'
    'right_upper_arm',             'left_upper_arm'
    'right_lower_arm',             'left_lower_arm'
    'right_hand',                  'left_hand'
    'right_hip_base',              'left_hip_base'
    'right_hip',                   'left_hip'
    'right_hip_transverse_link',   'left_hip_transverse_link'
    'right_hip_frontal_link',      'left_hip_frontal_link'
    'right_upper_leg',             'left_upper_leg'
    'right_lower_leg',             'left_lower_leg'
    'right_ankle_roll_link',       'left_ankle_roll_link'
    'right_foot',                  'left_foot'
};

%% Tolerances
tol.transformBottomRow = 1e-10;
tol.rotationOrthonormal = 1e-9;
tol.rotationDeterminant = 1e-9;
tol.massMatrixSymmetry = 1e-9;
tol.symmetryPosition = 1e-4;
tol.inertiaEigenvalue = 1e-10;
tol.inverseDynamics = 1e-8;

checkName = strings(0,1);
status = strings(0,1);
details = strings(0,1);

fprintf('============================================================\n');
fprintf(' MANUAL HUMANOID RBT VALIDATION\n');
fprintf('============================================================\n\n');

%% 1. Build robot
try
    robot = build_manual_humanoid_tree_BFreversed();
    [checkName,status,details] = appendResult(checkName,status,details, ...
        "Robot construction", true, ...
        sprintf('Built successfully with %d bodies.', robot.NumBodies));
catch ME
    [checkName,status,details] = appendResult(checkName,status,details, ...
        "Robot construction", false, ME.message);
    printResults(checkName,status,details);
    rethrow(ME)
end

q0 = homeConfiguration(robot);
nq = numel(q0);

fprintf('Robot built successfully.\n');
fprintf('Bodies: %d\n', robot.NumBodies);
fprintf('Generalized coordinates: %d\n\n', nq);

%% 2. Joint table
jointTable = makeJointTable(robot);
disp(jointTable)

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Unique body names", ...
    numel(unique(string(robot.BodyNames))) == robot.NumBodies, ...
    'Checked all rigid-body names.');

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Unique joint names", ...
    numel(unique(jointTable.JointName)) == height(jointTable), ...
    'Checked all joint names.');

%% 3. Body transform checks at q = 0
transformFailures = strings(0,1);
for i = 1:robot.NumBodies
    bodyName = robot.Bodies{i}.Name;
    try
        T = getTransform(robot, q0, bodyName, robot.BaseName);
        if norm(T(4,:) - [0 0 0 1], inf) > tol.transformBottomRow
            transformFailures(end+1) = bodyName + ": invalid bottom row";
        end
        R = T(1:3,1:3);
        if norm(R.'*R - eye(3), 'fro') > tol.rotationOrthonormal
            transformFailures(end+1) = bodyName + ": rotation not orthonormal";
        end
        if abs(det(R) - 1) > tol.rotationDeterminant
            transformFailures(end+1) = bodyName + ": det(R) not +1";
        end
    catch ME
        transformFailures(end+1) = bodyName + ": " + string(ME.message);
    end
end

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Body transforms", isempty(transformFailures), joinOrOK(transformFailures));

%% 4. Joint axis and limit checks
jointFailures = strings(0,1);
for i = 1:robot.NumBodies
    joint = robot.Bodies{i}.Joint;
    if strcmp(joint.Type,'revolute') || strcmp(joint.Type,'prismatic')
        if abs(norm(joint.JointAxis) - 1) > 1e-12
            jointFailures(end+1) = joint.Name + ": axis is not unit length";
        end
        if strcmp(joint.Type,'revolute')
            lim = joint.PositionLimits;
            if numel(lim) ~= 2 || any(~isfinite(lim)) || lim(1) >= lim(2)
                jointFailures(end+1) = joint.Name + ": invalid position limits";
            end
        end
    end
end

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Joint axes and limits", isempty(jointFailures), joinOrOK(jointFailures));

%% 5. Mass, COM, inertia checks
massFailures = strings(0,1);
physicalBodyCount = 0;
masslessBodyCount = 0;
totalMass = 0;

for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    if body.Mass > 0
        physicalBodyCount = physicalBodyCount + 1;
        totalMass = totalMass + body.Mass;

        if any(~isfinite(body.CenterOfMass))
            massFailures(end+1) = body.Name + ": non-finite COM";
        end

        I = inertiaVectorToMatrix(body.Inertia);
        eigI = eig((I + I.')/2);

        if any(eigI <= tol.inertiaEigenvalue)
            massFailures(end+1) = body.Name + ": inertia not positive definite";
        end

        principalMoments = sort(eigI);
        if principalMoments(3) > principalMoments(1)+principalMoments(2)+1e-10
            massFailures(end+1) = body.Name + ": inertia violates triangle inequality";
        end
    else
        masslessBodyCount = masslessBodyCount + 1;
        if any(abs(body.Inertia) > 0) || any(abs(body.CenterOfMass) > 0)
            massFailures(end+1) = body.Name + ": massless body has nonzero COM/inertia";
        end
    end
end

massDetail = sprintf('Physical bodies: %d, massless frame bodies: %d, total mass: %.6f kg.', ...
    physicalBodyCount, masslessBodyCount, totalMass);
if ~isempty(massFailures)
    massDetail = massDetail + " " + join(massFailures," | ");
end

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Mass, COM, and inertia", isempty(massFailures), massDetail);

%% 6. Zero configuration
if showZeroConfiguration
    figure('Name','Humanoid RBT - Zero Configuration');
    show(robot,q0,'Frames','on','PreservePlot',false);
    axis equal
    grid on
    view(3)
    title('Humanoid RBT - Zero Configuration')
end

%% 7. Perturb every movable joint
movableRows = jointTable.JointType ~= "fixed";
movableJoints = jointTable(movableRows,:);
perturbationFailures = strings(0,1);
motionMagnitude = zeros(height(movableJoints),1);

if animateEveryJoint
    figAnim = figure('Name','All Joint Perturbations');
end

for k = 1:height(movableJoints)
    idx = movableJoints.ConfigurationIndex(k);
    jointName = movableJoints.JointName(k);
    childBody = movableJoints.ChildBody(k);

    qPlus = q0;
    qMinus = q0;

    if movableJoints.JointType(k) == "revolute"
        delta = deg2rad(jointTestAngleDeg);
    else
        delta = 0.02;
    end

    qPlus(idx) = qPlus(idx) + delta;
    qMinus(idx) = qMinus(idx) - delta;

    try
        Tp = getTransform(robot,qPlus,char(childBody),robot.BaseName);
        Tm = getTransform(robot,qMinus,char(childBody),robot.BaseName);

        motionMagnitude(k) = norm(Tp(1:3,4)-Tm(1:3,4)) + ...
            norm(Tp(1:3,1:3)-Tm(1:3,1:3),'fro');

        if motionMagnitude(k) < 1e-10
            perturbationFailures(end+1) = jointName + ": no visible child-body motion";
        end

        if showEveryJointFigure
            figure('Name',char(jointName + " +/- test"));
            tiledlayout(1,3)
            nexttile
            show(robot,qMinus,'Frames','on','PreservePlot',false);
            axis equal; grid on; view(3); title(char(jointName + " negative"))
            nexttile
            show(robot,q0,'Frames','on','PreservePlot',false);
            axis equal; grid on; view(3); title('zero')
            nexttile
            show(robot,qPlus,'Frames','on','PreservePlot',false);
            axis equal; grid on; view(3); title(char(jointName + " positive"))
        end

        if animateEveryJoint
            figure(figAnim)
            show(robot,qMinus,'Frames','off','PreservePlot',false);
            axis equal; grid on; view(3); title(char(jointName + " : negative")); drawnow
            pause(animationPause)
            show(robot,q0,'Frames','off','PreservePlot',false);
            axis equal; grid on; view(3); title(char(jointName + " : zero")); drawnow
            pause(animationPause)
            show(robot,qPlus,'Frames','off','PreservePlot',false);
            axis equal; grid on; view(3); title(char(jointName + " : positive")); drawnow
            pause(animationPause)
        end
    catch ME
        perturbationFailures(end+1) = jointName + ": " + string(ME.message);
    end
end

movableJoints.MotionMetric = motionMagnitude;
disp(movableJoints(:,{'ConfigurationIndex','JointName','ChildBody','MotionMetric'}))

[checkName,status,details] = appendResult(checkName,status,details, ...
    "All-joint perturbation test", isempty(perturbationFailures), ...
    joinOrOK(perturbationFailures));

%% 8. Zero-pose left/right positional symmetry
symmetryFailures = strings(0,1);
symmetryError = zeros(size(symmetryPairs,1),1);

for i = 1:size(symmetryPairs,1)
    rightBody = symmetryPairs{i,1};
    leftBody = symmetryPairs{i,2};
    try
        Tr = getTransform(robot,q0,rightBody,robot.BaseName);
        Tl = getTransform(robot,q0,leftBody,robot.BaseName);
        pr = Tr(1:3,4);
        pl = Tl(1:3,4);
        expectedLeft = [-pr(1); pr(2); pr(3)];
        symmetryError(i) = norm(pl-expectedLeft);
        if symmetryError(i) > tol.symmetryPosition
            symmetryFailures(end+1) = sprintf('%s/%s reflection error = %.3e m', ...
                rightBody,leftBody,symmetryError(i));
        end
    catch ME
        symmetryFailures(end+1) = string(rightBody)+"/"+string(leftBody)+": "+string(ME.message);
    end
end

symmetryTable = table(string(symmetryPairs(:,1)),string(symmetryPairs(:,2)),symmetryError, ...
    'VariableNames',{'RightBody','LeftBody','PositionReflectionError'});
disp(symmetryTable)

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Zero-pose left/right positional symmetry", isempty(symmetryFailures), ...
    joinOrOK(symmetryFailures));

%% 9. Dynamics self-consistency
dynamicsFailures = strings(0,1);
try
    M0 = massMatrix(robot,q0);
    massMatrixSymmetryError = norm(M0-M0.','fro')/max(1,norm(M0,'fro'));
    if massMatrixSymmetryError > tol.massMatrixSymmetry
        dynamicsFailures(end+1) = sprintf('Mass-matrix asymmetry = %.3e',massMatrixSymmetryError);
    end

    eigM = eig((M0+M0.')/2);
    minimumMassMatrixEigenvalue = min(eigM);
    if minimumMassMatrixEigenvalue <= 0
        dynamicsFailures(end+1) = sprintf('Mass matrix not positive definite; min eig = %.3e', ...
            minimumMassMatrixEigenvalue);
    end

    tauG = gravityTorque(robot,q0);
    if any(~isfinite(tauG))
        dynamicsFailures(end+1) = 'gravityTorque contains non-finite values';
    end

    qd0 = zeros(size(q0));
    qdd0 = zeros(size(q0));
    tauID = inverseDynamics(robot,q0,qd0,qdd0);
    inverseDynamicsError = norm(tauID-tauG);
    if inverseDynamicsError > tol.inverseDynamics
        dynamicsFailures(end+1) = sprintf('inverseDynamics-gravityTorque mismatch = %.3e', ...
            inverseDynamicsError);
    end

    fprintf('\nDynamics diagnostics:\n');
    fprintf('  Mass-matrix symmetry ratio: %.3e\n',massMatrixSymmetryError);
    fprintf('  Minimum mass-matrix eigenvalue: %.3e\n',minimumMassMatrixEigenvalue);
    fprintf('  ||inverseDynamics(q,0,0)-gravityTorque(q)||: %.3e\n',inverseDynamicsError);

    gravityTable = table(movableJoints.ConfigurationIndex,movableJoints.JointName,tauG(:), ...
        'VariableNames',{'ConfigurationIndex','JointName','GravityTorque'});
    disp(gravityTable)
catch ME
    dynamicsFailures(end+1) = string(ME.message);
end

[checkName,status,details] = appendResult(checkName,status,details, ...
    "Dynamics self-consistency", isempty(dynamicsFailures), joinOrOK(dynamicsFailures));

%% 10. Final summary
results = table(checkName,status,details,'VariableNames',{'Check','Status','Details'});

fprintf('\n============================================================\n');
fprintf(' VALIDATION SUMMARY\n');
fprintf('============================================================\n');
disp(results)

if any(results.Status == "FAIL")
    fprintf('\nSome checks failed. Inspect the rows marked FAIL.\n');
else
    fprintf('\nAll automated internal RBT checks passed.\n');
end

fprintf('\nStill required for exact Simscape equivalence:\n');
fprintf('  1) Compare logged Simscape body transforms with getTransform().\n');
fprintf('  2) Compare positive/negative motion direction for every joint.\n');
fprintf('  3) Validate fixed-torso gravity torques in both models.\n');
fprintf('  4) Confirm applied torque signs for flipped B/F joints.\n');

%% Local functions
function jointTable = makeJointTable(robot)
    configurationIndex = nan(robot.NumBodies,1);
    jointName = strings(robot.NumBodies,1);
    childBody = strings(robot.NumBodies,1);
    jointType = strings(robot.NumBodies,1);
    movableCounter = 0;

    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        joint = body.Joint;
        jointName(i) = string(joint.Name);
        childBody(i) = string(body.Name);
        jointType(i) = string(joint.Type);
        if ~strcmp(joint.Type,'fixed')
            movableCounter = movableCounter + 1;
            configurationIndex(i) = movableCounter;
        end
    end

    jointTable = table(configurationIndex,jointName,childBody,jointType, ...
        'VariableNames',{'ConfigurationIndex','JointName','ChildBody','JointType'});
end

function I = inertiaVectorToMatrix(v)
% MATLAB rigidBody convention: [Ixx Iyy Izz Iyz Ixz Ixy]
    Ixx=v(1); Iyy=v(2); Izz=v(3); Iyz=v(4); Ixz=v(5); Ixy=v(6);
    I = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
end

function [names,status,details] = appendResult(names,status,details,name,ok,msg)
    names(end+1,1) = string(name);
    if ok
        status(end+1,1) = "PASS";
    else
        status(end+1,1) = "FAIL";
    end
    details(end+1,1) = string(msg);
end

function text = joinOrOK(items)
    if isempty(items)
        text = "No problems detected.";
    else
        text = join(items," | ");
    end
end

function printResults(checkName,status,details)
    results = table(checkName,status,details,'VariableNames',{'Check','Status','Details'});
    disp(results)
end
