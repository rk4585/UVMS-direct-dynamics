function [dksi_dt, dzeta_dt]=forward_dynamics(params,ksi,zeta, control_input)
Kinematics = getKinematics(params,ksi);
M = calculateMassMatrix(params,Kinematics);
C = calculateCoriolisMatrix(params,zeta,Kinematics);
D = calculateHydrodynamicDamping(params,zeta,Kinematics);
N = calculatePersistentForces(params,ksi,Kinematics);
Ja = calculate_velocity_tranformation_matrix(ksi);
%Jw = calculate_workspace_Jacobian(params, ksi, Kinematics);
if isfield(params,'Arms')
    
    tau = blkdiag(params.ThrusterAllocationMatrix,eye(params.NumDoFs-6))* control_input;
else
    tau = params.ThrusterAllocationMatrix * control_input;
end
dksi_dt = Ja*zeta;
dzeta_dt = (M+0.0000000001*eye(14))^-1*(-(C+D)*zeta - N + tau);
end


function [N_p, Fips] = calculatePersistentForces(params,ksi, Kinematics)
Fips = cell(params.NumDoFs,1);
Jvehicle = [eye(6),zeros(6,params.NumDoFs-6)];
mi = params.Mass;
g = params.g;
rho = params.rho;
volume = params.Volume;
Adgici = [eye(3), skew(params.CenterOfMass);zeros(3),eye(3)];
Adgibi = [eye(3), skew(params.CenterOfBuoyancy);zeros(3),eye(3)];
Ri0 = euler2rotmXYZ(ksi(4),ksi(5),ksi(6))';
ez = [0;0;1];
Fip = Adgici'*[-mi*params.g*Ri0*ez ;0;0;0]  + Adgibi'*[rho*volume*g*Ri0*ez ;0;0;0 ];
k=1;
Fips{k} = Fip;
N_p = (Jvehicle')*Fip;
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    start_idx = 7;
    armnames = fieldnames(params.Arms);
    for i=1:N
        end_idx = start_idx + length(fieldnames(params.Arms.(armnames{i}).Joints))-1;
        num_dofs_arms = length(fieldnames(params.Arms.(armnames{i}).Joints));
        link_names = fieldnames(Kinematics.Jacobians(i).Jb);
        for j=2:num_dofs_arms+1
            link_name = link_names{j};
            J = zeros(6,params.NumDoFs);
            J(:,1:6) =  Kinematics.Jacobians(i).Jb.(link_name)(:,1:6);
            J(:,start_idx:end_idx) = Kinematics.Jacobians(i).Jb.(link_name)(:,7:end);
            mi = params.Arms.(armnames{i}).Links.(link_name).Mass;
            volume = params.Arms.(armnames{i}).Links.(link_name).Volume;
            Adgici = [eye(3), skew(params.Arms.(armnames{i}).Links.(link_name).CenterOfMass);zeros(3),eye(3)];
            Adgibi = [eye(3), skew(params.Arms.(armnames{i}).Links.(link_name).CenterOfBuoyancy);zeros(3),eye(3)];
            Ri0 = Kinematics.Transforms(i).R0i.(link_name)';
            Fip = Adgici'*[-mi*g*Ri0*ez ;0;0;0]  + Adgibi'*[rho*volume*g*Ri0*ez ;0;0;0 ];
            N_p = N_p + (J')*Fip;
            k=k+1;
            Fips{k}=Fip;
        end
        start_idx = end_idx + 1;
    end
end
end


function D = calculateHydrodynamicDamping(params, zeta, Kinematics)
Jvehicle = [eye(6),zeros(6,params.NumDoFs-6)];
Adgici = [eye(3), skew(params.CenterOfMass);zeros(3),eye(3)];
Vcg = Adgici * Jvehicle * zeta;
Dcg = params.LinearDamping + params.QuadraticDamping*diag(abs(Vcg));
Di = Adgici'*Dcg;
D = (Jvehicle')*Di*Jvehicle;
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    start_idx = 7;
    armsnames = fieldnames(params.Arms);
    for i=1:N
        end_idx = start_idx + length(fieldnames(params.Arms.(armsnames{i}).Joints))-1;
        num_dofs_arms = length(fieldnames(params.Arms.(armsnames{i}).Joints));
        link_names = fieldnames(Kinematics.Jacobians(i).Jb);
        for j=2:num_dofs_arms+1
            link_name = link_names{j};
            J = zeros(6,params.NumDoFs);
            J(:,1:6) =  Kinematics.Jacobians(i).Jb.(link_name)(:,1:6);
            J(:,start_idx:end_idx) = Kinematics.Jacobians(i).Jb.(link_name)(:,7:end);
            Adgici = [eye(3), skew(params.Arms.(armsnames{i}).Links.(link_name).CenterOfMass);zeros(3),eye(3)];
            Vcg = Adgici * J * zeta;
            Dcg = params.Arms.(armsnames{i}).Links.(link_name).LinearDamping + params.Arms.(armsnames{i}).Links.(link_name).QuadraticDamping*diag(abs(Vcg));
            Di = Adgici'*Dcg;
            D = D + (J')*Di*J;
        end
        start_idx = end_idx + 1;
    end
end
end

function W = calculateW( zeta, I ,Jbi)
v_omega = Jbi * zeta;
v = v_omega(1:3);
omega = v_omega(4:6);
mI = I(1:3,1:3);
Ib = I(4:6,4:6);
mrgb = I(4:6,1:3);
dKv = mI * v - mrgb * omega;
dKomega = Ib * omega + mrgb * v;
W = [zeros(3,3), skew(dKv) ; skew(dKv), skew(dKomega)];
end

function TransformTimeDerivatives = calculateTransformTimeDerivatives(params, zeta, Kinematics)
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    dgbi_dts = cell(1,N);
    dRbi_dts = cell(1,N);
    dpbi_dts = cell(1,N);
    omegabis = cell(1,N);
    dAdgbi_dts = cell(1,N);
    dAdgbi_inv_dts = cell(1,N);
    start_idx = 7;
    armnames = fieldnames(params.Arms);
    for i = 1:N
        end_idx = start_idx+length(fieldnames(params.Arms.(armnames{i}).Joints))-1;
        dgbi_dt = struct('Link1',eye(4),'Link2',eye(4),'Link3',eye(4),'Link4',eye(4),'Link5',eye(4),'EE',eye(4));
        omegabi = struct('Link1',zeros(3,1),'Link2',zeros(3,1),'Link3',zeros(3,1),'Link4',zeros(3,1),'Link5',zeros(3,1),'EE',zeros(3,1));
        dRbi_dt = struct('Link1',eye(3),'Link2',eye(3),'Link3',eye(3),'Link4',eye(3),'Link5',eye(3),'EE',eye(3));
        dpbi_dt = struct('Link1',zeros(3,1),'Link2',zeros(3,1),'Link3',zeros(3,1),'Link4',zeros(3,1),'Link5',zeros(3,1),'EE',zeros(3,1));
        dAdgbi_dt = struct('Link1',eye(6),'Link2',eye(6),'Link3',eye(6),'Link4',eye(6),'Link5',eye(6),'EE',eye(6));
        dAdgbi_inv_dt = struct('Link1',eye(6),'Link2',eye(6),'Link3',eye(6),'Link4',eye(6),'Link5',eye(6),'EE',eye(6));
        field_names = fieldnames(Kinematics.Transforms(i).gbi);
        for j=1:length(field_names)
            field_name = field_names{j};
            pbi = Kinematics.Transforms(i).pbi.(field_name);
            Rbi = Kinematics.Transforms(i).Rbi.(field_name);
            Adgbi_inv = Kinematics.Transforms(i).Adgbi_inv.(field_name);
            Jbi = Kinematics.Jacobians(i).Jb.(field_name);
            v = Kinematics.Transforms(i).Adgbi.(field_name) * Jbi * [zeta(1:6);zeta(start_idx:end_idx)];
            dpbi_dt.(field_name) = Rbi * [eye(3),zeros(3)] * Jbi(:,7:end) * zeta(start_idx:end_idx);
            omegabi.(field_name) = zeta(4:6) - Rbi * [zeros(3),eye(3)] * Jbi * [zeta(1:6);zeta(start_idx:end_idx)];
            dRbi_dt.(field_name) = skew(omegabi.(field_name)) * Rbi;
            dgbi_dt.(field_name) = [dRbi_dt.(field_name),dpbi_dt.(field_name);0,0,0,0];
            dAdgbi_dt.(field_name) = getdAdgbi_dt(pbi,Rbi,dpbi_dt.(field_name),dRbi_dt.(field_name));
            dAdgbi_inv_dt.(field_name) = - Adgbi_inv * dAdgbi_dt.(field_name) * Adgbi_inv;
        end
        dgbi_dts{i}=dgbi_dt;
        dRbi_dts{i}=dRbi_dt;
        dpbi_dts{i}=dpbi_dt;
        omegabis{i}=omegabi;
        dAdgbi_dts{i} = dAdgbi_dt;
        dAdgbi_inv_dts{i} = dAdgbi_dt;
        start_idx = end_idx + 1;
    end
    TransformTimeDerivatives = struct('dRbi_dt',dRbi_dts,'dpbi_dt',dpbi_dts,'dgbi_dt',dgbi_dts,'dAdgbi_dt',dAdgbi_dts,'dAdgbi_inv_dt',dAdgbi_inv_dts);
else
    TransformTimeDerivatives = [];
end
end

function dJbi_dts = calculateJacobiansTimeDerivative(params, zeta, Kinematics)
transforms_dt = calculateTransformTimeDerivatives(params,zeta, Kinematics);
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    dJbi_dts = cell(1,N);
    armnames = fieldnames(params.Arms);
    for i = 1:N
        names = fieldnames(transforms_dt(i).dpbi_dt);
        num_joints = length(fieldnames(params.Arms.(armnames{i}).Joints));
        dJi_dt = struct('Link1',zeros(6,num_joints),'Link2',zeros(6,num_joints),'Link3',zeros(6,num_joints),'Link4',zeros(6,num_joints),'Link5',zeros(6,num_joints),'EE',zeros(6,num_joints));
        dJbi_dt = struct('Link1',zeros(6,6+num_joints),'Link2',zeros(6,6+num_joints),'Link3',zeros(6,6+num_joints),'Link4',zeros(6,6+num_joints),'Link5',zeros(6,6+num_joints),'EE',zeros(6,6+num_joints));
        for j=1:length(names)
            dof_ind=1;
            if j~=length(names)
                for k=2:j
                    dJi_dt.(names{j})(:,dof_ind) = transforms_dt(i).dAdgbi_dt.(names{k-1})*[0;0;0;0;0;1];
                    dof_ind = dof_ind + 1;
                end
            end
            dJi_dt.EE(:,1:num_joints-1)= dJi_dt.(names{j})(:,1:num_joints-1);
            dJi_dt.EE(:,num_joints) = transforms_dt(i).dAdgbi_dt.EE*[0;0;0;0;0;1];

            Adgbi_inv = Kinematics.Transforms(i).Adgbi_inv.(names{j});
            Ji = Kinematics.Jacobians(i).Ji.(names{j});
            dAdgbi_inv_dt = transforms_dt(i).dAdgbi_inv_dt.(names{j});
            dJbi_dt.(names{j}) = [dAdgbi_inv_dt, dAdgbi_inv_dt * Ji + Adgbi_inv * dJi_dt.(names{j})];
        end
        dJbi_dts{i} = dJbi_dt;
    end
else
    dJbi_dts = [];
end
end

function C = calculateCoriolisMatrix(params, zeta, Kinematics)
Jvehicle = [eye(6),zeros(6,params.NumDoFs-6)];
mass = params.Mass;
inertiaTensor = params.Inertia;
I = [diag([mass,mass,mass]),zeros(3);zeros(3),inertiaTensor]+params.AddedMassMatrix;
Wvehicle = calculateW(zeta, I ,Jvehicle);
C = - (Jvehicle')*Wvehicle*Jvehicle;
 %Mdot = zeros(size(C));
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    jacobian_time_derivatives = calculateJacobiansTimeDerivative(params, zeta, Kinematics);
    start_idx = 7;
    armnames = fieldnames(params.Arms);
    for i=1:N
        end_idx = start_idx + length(fieldnames(params.Arms.(armnames{i}).Joints))-1;
        num_dofs_arms = length(fieldnames(params.Arms.(armnames{i}).Joints));
        link_names = fieldnames(Kinematics.Jacobians(i).Jb);
        for j=2:num_dofs_arms+1
            link_name = link_names{j};
            inertiaTensor = params.Arms.(armnames{i}).Links.(link_name).InertiaTensor;
            mass = params.Arms.(armnames{i}).Links.(link_name).Mass;
            I = [diag([mass,mass,mass]),zeros(3);zeros(3),inertiaTensor]  + params.Arms.(armnames{i}).Links.(link_name).AddedMass;
            J = zeros(6,params.NumDoFs);
            J(:,1:6) =  Kinematics.Jacobians(i).Jb.(link_name)(:,1:6);
            J(:,start_idx:end_idx) = Kinematics.Jacobians(i).Jb.(link_name)(:,7:end);
            dJ_dt = zeros(6,params.NumDoFs);
            dJ_dt(:,1:6) = jacobian_time_derivatives{i}.(link_name)(:,1:6);
            dJ_dt(:,start_idx:end_idx) = jacobian_time_derivatives{i}.(link_name)(:,7:end);
            W = calculateW(zeta, I, J);
            %Mdot = Mdot + dJ_dt'*I*J + J'*I*dJ_dt;
            C = C + (J')*I*dJ_dt - (J')*W*J;
        end
        start_idx = end_idx + 1;
    end
end
 %SK=Mdot-2*C;
 %disp(issymmetric(SK,"skew"))
end


function  M = calculateMassMatrix(params, Kinematics)

Jvehicle = [eye(6),zeros(6,params.NumDoFs - 6)];
mass = params.Mass;
inertiaTensor = params.Inertia;
I = [diag([mass,mass,mass]),zeros(3);zeros(3),inertiaTensor]+params.AddedMassMatrix;
M = Jvehicle' * I * Jvehicle;
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    start_idx = 7;
    armnames = fieldnames(params.Arms);
    for i=1:N
        armname = armnames{i};
        end_idx = start_idx + length(fieldnames(params.Arms.(armname).Joints))-1;
        num_dofs_arms = length(fieldnames(params.Arms.(armname).Joints));
        link_names = fieldnames(Kinematics.Jacobians(i).Jb);
        for j=2:num_dofs_arms+1
            link_name = link_names{j};
            inertiaTensor = params.Arms.(armname).Links.(link_name).InertiaTensor;
            mass = params.Arms.(armname).Links.(link_name).Mass;
            I = [diag([mass,mass,mass]),zeros(3);zeros(3),inertiaTensor]  + params.Arms.(armname).Links.(link_name).AddedMass;
            Jbi = zeros(6,params.NumDoFs);
            Jbi(:,1:6) =  Kinematics.Jacobians(i).Jb.(link_name)(:,1:6);
            Jbi(:,start_idx:end_idx) = Kinematics.Jacobians(i).Jb.(link_name)(:,7:end);
            M = M + Jbi' * I * Jbi;
        end
        start_idx = end_idx + 1;
    end
end
end

function Kinematics = getKinematics(params,ksi)
[Jacobians, transforms] = calculateJacobians(params,ksi);
Kinematics = struct('Jacobians', Jacobians, 'Transforms', transforms);
end

function [Jacobians, transforms, Ji] = calculateJacobians(params,ksi)
[Ji, transforms] = calculateArmJacobians(params,ksi);
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    Jbs = cell(1,N);
    armnames = fieldnames(params.Arms);
    for i=1:N
        num_joints = length(fieldnames(params.Arms.(armnames{i}).Joints));
        names = fieldnames(transforms(i).pbi);
        Jb= struct('Link1',zeros(6,6+num_joints),'Link2',zeros(6,6+num_joints),'Link3',zeros(6,6+num_joints),'Link4',zeros(6,6+num_joints),'Link5',zeros(6,6+num_joints),'EE',zeros(6,6+num_joints));
        for j=1:length(names)
            Jb.(names{j}) = [transforms(i).Adgbi_inv.(names{j}), transforms(i).Adgbi_inv.(names{j}) * Ji{i}.(names{j})];
        end
        Jbs{i}=Jb;
    end
    Jacobians = struct('Jb',Jbs, 'Ji', Ji);
else
    Jacobians = [];
end
end


function [Jis, transforms] = calculateArmJacobians(params,ksi)
transforms = calculateTransforms(params,ksi);
if isfield(params,'Arms')
    N = length(fieldnames(params.Arms));
    Jis = cell(1,N);
    armsfieldnames = fieldnames(params.Arms);
    for i = 1:N
        names = fieldnames(transforms(i).pbi);
        num_joints = length(fieldnames(params.Arms.(armsfieldnames{i}).Joints));
        Ji= struct('Link1',zeros(6,num_joints),'Link2',zeros(6,num_joints),'Link3',zeros(6,num_joints),'Link4',zeros(6,num_joints),'Link5',zeros(6,num_joints),'EE',zeros(6,num_joints));
        for j=1:length(names)
            dof_ind = 1;
            if j~=length(names)
                for k=2:j
                    Ji.(names{j})(:,dof_ind) = transforms(i).Adgbi.(names{k-1})*[0;0;0;0;0;1];
                    dof_ind = dof_ind + 1;
                end
            end
        end
        Ji.EE(:,1:num_joints-1) = Ji.(names{j})(:,1:num_joints-1);
        Ji.EE(:,num_joints) = transforms(i).Adgbi.EE*[0;0;0;0;0;1];
        Jis{i} = Ji;
    end
else
    Jis = [];
end
end

function transforms = calculateTransforms(params, ksi)
R0b = euler2rotmXYZ(ksi(4),ksi(5),ksi(6));
g0b = [R0b, (ksi(1:3)); 0,0,0,1];
if isfield(params, 'Arms')
    N = length(fieldnames(params.Arms));
    gbis = cell(1,N);
    g0is = cell(1,N);
    Rbis = cell(1,N);
    R0is = cell(1,N);
    pbis = cell(1,N);
    p0is = cell(1,N);
    Adgbis = cell(1,N);
    Adg0is = cell(1,N);
    Adgbi_invs = cell(1,N);
    for i = 1:N
        gbi = calculateArmTransforms(params, ksi, eye(4),i);
        g0i = calculateArmTransforms(params, ksi, g0b, i);
        field_names = fieldnames(g0i);
        Rbi = struct('Link1',eye(3),'Link2',eye(3),'Link3',eye(3),'Link4',eye(3),'Link5',eye(3),'EE',eye(3));
        R0i = struct('Link1',eye(3),'Link2',eye(3),'Link3',eye(3),'Link4',eye(3),'Link5',eye(3),'EE',eye(3));
        pbi = struct('Link1',zeros(3,1),'Link2',zeros(3,1),'Link3',zeros(3,1),'Link4',zeros(3,1),'Link5',zeros(3,1),'EE',zeros(3,1));
        p0i = struct('Link1',zeros(3,1),'Link2',zeros(3,1),'Link3',zeros(3,1),'Link4',zeros(3,1),'Link5',zeros(3,1),'EE',zeros(3,1));
        Adgbi = struct('Link1',eye(6),'Link2',eye(6),'Link3',eye(6),'Link4',eye(6),'Link5',eye(6),'EE',eye(6));
        Adg0i = struct('Link1',eye(6),'Link2',eye(6),'Link3',eye(6),'Link4',eye(6),'Link5',eye(6),'EE',eye(6));
        Adgbi_inv = struct('Link1',eye(6),'Link2',eye(6),'Link3',eye(6),'Link4',eye(6),'Link5',eye(6),'EE',eye(6));
        for j=1:length(field_names)
            field_name = field_names{j};
            Rbi.(field_name) = gbi.(field_name)(1:3,1:3);
            pbi.(field_name) = gbi.(field_name)(1:3,4);
            R0i.(field_name) = g0i.(field_name)(1:3,1:3);
            p0i.(field_name) = g0i.(field_name)(1:3,4);
            Adgbi.(field_name) = getAdgbi(Rbi.(field_name),pbi.(field_name));
            Adg0i.(field_name) = getAdgbi(R0i.(field_name),p0i.(field_name));
            Adgbi_inv.(field_name) = (getAdgbi(Rbi.(field_name),pbi.(field_name)))^-1;
        end
        gbis{i} = gbi;
        g0is{i} = g0i;
        Rbis{i} = Rbi;
        R0is{i} = R0i;
        pbis{i} = pbi;
        p0is{i} = p0i;
        Adgbis{i} = Adgbi;
        Adg0is{i} = Adg0i;
        Adgbi_invs{i} = Adgbi_inv;
    end
    transforms = struct('g0b',g0b,'Rbi',Rbis,'pbi',pbis,'R0i',R0is,'p0i',p0is,'gbi',gbis,'g0i',g0is,'Adgbi',Adgbis,'Adg0i',Adg0is,'Adgbi_inv',Adgbi_invs);
else
    transforms = struct('g0b',g0b);
end
end

function arm_transforms = calculateArmTransforms(params,ksi, T_global, arm_index)
% Calculate transformations for each link in the arm
T = T_global;  % Start with the base transform
arm_transforms = struct('Link1',eye(4),'Link2',eye(4),'Link3',eye(4),'Link4',eye(4),'Link5',eye(4),'EE',eye(4));
armsfieldnames = fieldnames(params.Arms);
armname = armsfieldnames{arm_index};
linknames = fieldnames(params.Arms.(armname).Links);

for i=1:arm_index
    if i==1
        k=7;
    else
        k = k + length(fieldnames(params.Arms.(armsfieldnames{i}).Joints));
    end
end
joint_idx = 1;
%jointnames = fieldnames(params.Arms.(armname).Joints);
for i = 1:length(fieldnames(params.Arms.(armname).Links))
    if i==1
        T = T * params.Arms.(armname).TransformVehicleToArmBase;
    else
        dh_row = [params.Arms.(armname).DHParameters.d(joint_idx),...
            params.Arms.(armname).DHParameters.theta(joint_idx)+ksi(k),params.Arms.(armname).DHParameters.a(joint_idx),params.Arms.(armname).DHParameters.alpha(joint_idx)];
        DH = dh_transform(dh_row);
        T = T * DH; %axang2tform([0 0 1 ksi(k)]) * params.Arms.(armname).Joints.(jointnames{i-1}).TransformParentToChild; %its the same
        k=k+1;
        joint_idx = joint_idx+1;
    end
    linkName = linknames{i};
    arm_transforms.(linkName) = T;
end
arm_transforms.EE = T * params.Arms.(armname).LastLinkToEE;
end


function mat = getAdgbi(R,p)
ps = skew(p);
mat = [R , ps*R ; zeros(3), R ];
end

function mat = getdAdgbi_dt(p,R,dp_dt,dRdt)
ps = skew(p);
dps = skew(dp_dt);
mat = [dRdt , ps*dRdt + dps * R ; zeros(3,3), dRdt];
end
function S = skew(v)
% Ensure the vector is a 1x3 or 3x1 vector
if length(v) ~= 3
    error('Input must be a 1x3 or 3x1 vector.');
end
% Convert any vector to a 1x3 row vector for uniformity
v = reshape(v, 1, []);
% Create the skew-symmetric matrix
S = [0, -v(3), v(2);
    v(3), 0, -v(1);
    -v(2), v(1), 0];
end

function R_0_B = calculate_linear_vel2der_transform(ksi)
R_0_B = [cos(ksi(6))*cos(ksi(5)), cos(ksi(6))*sin(ksi(5))*sin(ksi(4)) - sin(ksi(6))*cos(ksi(4)), cos(ksi(6))*cos(ksi(4))*sin(ksi(5)) + sin(ksi(6))*sin(ksi(4));
    sin(ksi(6))*cos(ksi(5)),  sin(ksi(4))*sin(ksi(5))*sin(ksi(6)) + cos(ksi(6))*cos(ksi(4)), sin(ksi(5))*sin(ksi(6))*cos(ksi(4)) - cos(ksi(6))*sin(ksi(4));
    -sin(ksi(5))        ,   cos(ksi(5))*sin(ksi(4))      , cos(ksi(5))*cos(ksi(4))];
end
function S_0_B = calculate_angular_vel2eulder_transform(ksi)
S_0_B = [1,   sin(ksi(4))*tan(ksi(5)), cos(ksi(4))*tan(ksi(5));
    0,   cos(ksi(4)),           -sin(ksi(4));
    0,   sin(ksi(4))/cos(ksi(5)), cos(ksi(4))/cos(ksi(5))];
end
function Ja = calculate_velocity_tranformation_matrix(ksi)
R_0_B = calculate_linear_vel2der_transform(ksi);
S_0_B = calculate_angular_vel2eulder_transform(ksi);
n = length(ksi)-6;
if n==0
    Ja = [R_0_B,      zeros(3);
        zeros(3),   S_0_B];
else
    Ja = [R_0_B,      zeros(3),   zeros(3,n);
        zeros(3),   S_0_B,      zeros(3,n);
        zeros(n,3), zeros(n,3), eye(8)];
end
end


function Jw = calculate_workspace_Jacobian(params, ksi, Kinematics)
R_0_B = calculate_linear_vel2der_transform(ksi);
S_0_B = calculate_angular_vel2eulder_transform(ksi);
Jb = blkdiag(R_0_B,S_0_B);

   AdgEEb1 = Kinematics.Transforms(1).Adgbi_inv.EE;
   R0EE1 = Kinematics.Transforms(1).R0i.EE;
   AdgEEb2 = Kinematics.Transforms(2).Adgbi_inv.EE;
   R0EE2 = Kinematics.Transforms(2).R0i.EE;
   eta0EE21 = rotm2eul(R0EE1,"ZYX");
   eta0EE22 = rotm2eul(R0EE2,"ZYX");
   S0EE1 = calculate_angular_vel2eulder_transform([0;0;0;eta0EE21']);
   S0EE2 = calculate_angular_vel2eulder_transform([0;0;0;eta0EE22']);
   JEE1 = blkdiag(R0EE1,S0EE1);
   JEE2 = blkdiag(R0EE2,S0EE2);
   Jmg1 = Kinematics.Jacobians(1).Ji.EE;
   Jmg2 = Kinematics.Jacobians(2).Ji.EE;
Jw = [Jb,zeros(6,4),zeros(6,4);
      JEE1*AdgEEb1,JEE1*Jmg1,zeros(6,4);
      JEE2*AdgEEb2,zeros(6,4),JEE2*Jmg2];
end



function T = dh_transform(dh_row)
% Extract parameters from the input row
d = dh_row(1);
theta = dh_row(2);
a = dh_row(3);
alpha = dh_row(4);

% Define the transformation matrix using the DH parameters
T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),            d;
    0,           0,                      0,                     1];
end

function R=euler2rotmXYZ(phi,theta,psi)
Rx = [1,0,0;
      0,cos(phi),-sin(phi);
      0,sin(phi),cos(phi)];
Ry = [cos(theta),0,sin(theta);
      0, 1, 0;
      -sin(theta),0,cos(theta)];
Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0,0,1];

R = Rz * Ry * Rx;
end
