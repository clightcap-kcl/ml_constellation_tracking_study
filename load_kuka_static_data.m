
% load KUKA static data
function kuka_T_ee = load_kuka_static_data(kuka_static_data_filename,pose_order)

% kuka_static_data_filename = 'kuka_static_poses.csv';
kuka_static_data = csvread(kuka_static_data_filename);

% KUKA KR10 Model
DegToRad = pi / 180;
flange_offset_deg = 22;
pedestal_height = 350;
base_offset = 400;
a1 = 25;
a2 = 560;
a3 = 25;
d4 = 515;
d6 = 90;

qmin = DegToRad * [-170, -190,  -120, -185, -120, -350]; % min joint angle limit for each joint
qmax = DegToRad * [ 170,   45,   156,  185,  120,  350]; % max joint angle limit for each joint

% DH parameters for each link
L(1) = Revolute('d',-base_offset,'a',a1,'alpha',pi/2,'offset',0,...
    'qlim',[qmin(1) qmax(1)]);
L(2) = Revolute('d',0,'a',a2,'alpha',0,'offset',0,...
    'qlim',[qmin(2) qmax(2)]);
L(3) = Revolute('d',0,'a',a3,'alpha',pi/2,'offset',-pi/2,...
    'qlim',[qmin(3) qmax(3)]);
L(4) = Revolute('d',-d4,'a',0,'alpha',-pi/2,'offset',0,...
    'qlim',[qmin(4) qmax(4)]);
L(5) = Revolute('d',0,'a',0,'alpha',pi/2,'offset',0,...
    'qlim',[qmin(5) qmax(5)]);
L(6) = Revolute('d',-d6,'a',0,'alpha',0,'offset',0,...
    'qlim',[qmin(6) qmax(6)]);

% transformation of totem left camera wrt flange/rig
flange_T_world_left = [...
        -0.325635094842246        -0.493846038306704        -0.806274069690856         -131.212613945592; ...
         0.149908180893914         0.814999641809131        -0.559734866836135           34.980152757643; ...
         0.933535924487194        -0.303136395537985        -0.191360924412689          111.525627794308; ...
                         0                         0                         0                         1];
flange_p_world_left = transl(flange_T_world_left);
flange_p_rig = [flange_p_world_left(1) 0 flange_p_world_left(3) + 15];
flange_T_rig = [eye(3) flange_p_rig';...
                0 0 0 1];

KR10 = SerialLink(L,'name', 'KR10');
KR10.base = transl(0,0,pedestal_height) * trotx(pi); % rotation about x axis by pi
KR10.tool = troty(pi) * trotz(-DegToRad*flange_offset_deg) *  flange_T_rig;

q_home = DegToRad*[0,-90,90,0,-90,180-flange_offset_deg];       
% figure, KR10.plot(q_home);

number_of_poses = length(pose_order);
kuka_T_ee = cell(1,number_of_poses);
kuka_p_ee = nan(number_of_poses,3);
kuka_r_ee_deg = nan(number_of_poses,3);
for index_i=1:number_of_poses
    pose_i = pose_order(index_i);
    
    kuka_T_ee{index_i} = KR10.fkine(kuka_static_data(pose_i,:)).T;
    kuka_p_ee(index_i,:) = kuka_T_ee{index_i}(1:3,4)';
    
    [th,r] = tr2angvec(kuka_T_ee{index_i},'deg');
    kuka_r_ee_deg(index_i,:) = r * th;
end
