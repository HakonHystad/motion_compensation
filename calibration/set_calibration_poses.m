% Run once before first calibration, manipulator poses will be made and
% stored for further calibration routines.


clc
clear
close all

addpath('./matlab_support');

%% calculate camera placement for total view of trajectory

camera= [   1305.599976, 0.000000, 505.700012, 0.000000;
            0.000000, 1306.599976, 371.399994, 0.000000;
            0.000000, 0.000000, 1.000000, 0.000000 ];
K = camera(1:3,1:3);

% image size
w = 1024;% pixels
h = 768;% pixels

resolution = 60;% controls how many configurations are made, hard to say the end result

squareSz = 0.03;% m
n_col = 10;% must be even
n_row = 8;% must be even

boardSize = [ n_row n_col ];% m

boardSz = max(boardSize)*squareSz; % max chessboard size
cameraHeight = 1;% m
offset_x = 0.2;% m


% field of view
f_h = K(1,1);% focal length in  vertical pixel units
f_w = K(2,2);% focal length in  horizontal pixel units

% trajectory is assumed in the y-direction
trajectoryLength = 2.5;% m

gamma = f_w/w;
dist = (trajectoryLength/2)*( gamma - 1/(4*gamma) );
offset_y = (trajectoryLength/2)*( gamma + 1/(4*gamma) -1 );

dist_z = dist*(h/f_h);
height = dist_z/2 + cameraHeight;




%% add robot
addpath('robot');

robot = importrobot('kr120r2500pro.urdf');

% add chess board
chessBoard = robotics.RigidBody('board');
board_origin = [ eye(3) [-boardSz/2 -boardSz/2 0]'; 0 0 0 1];
addVisual( chessBoard, 'Mesh', 'board.STL', board_origin );


% child to joint transform
Rboard = roty(90)'*rotz(90);
tboard = [0 0 0.25]';%boardSz/2]';
Tboard = [ Rboard tboard; 0 0 0 1];
setFixedTransform(chessBoard.Joint, Tboard );
addBody( robot, chessBoard, 'tool0' );


robotFig = figure();
figure(robotFig);
show(robot);

lim = [ -3, 3, -3, 3, -0.2, 3 ];
axis(lim);
%% add cameras
R1 = rotz(90)*roty(-90)*rotz(45);
t1 = [ offset_x (trajectoryLength/2)+offset_y cameraHeight ]';
Tcam1 = [ R1 -R1*t1; 0 0 0 1];


R2 = rotz(90)*roty(-90)*rotz(-45);
t2 = t1; t2(2) = -t2(2);
Tcam2 = [ R2 -R2*t2; 0 0 0 1];


hold on
cam1 = plotCamera('Location',t1, 'Orientation', R1, 'Opacity', 0.6, 'Color', 'blue', 'Size', 0.25, 'AxesVisible', true );
cam2 = plotCamera('Location',t2, 'Orientation', R2, 'Opacity', 0.6, 'Color', 'green', 'Size', 0.25, 'AxesVisible', true );

%% add hanger trajectory

trajectory = [ dist dist; -trajectoryLength/2 trajectoryLength/2; height height ];
plot3( trajectory(1,:), trajectory(2,:), trajectory(3,:), 'c--', 'Linewidth', 3 ); 

%% add calibration trajectory

a = trajectoryLength/2 - boardSz/2;% - 0.25;% within half of hanger trajectory
h_ = a*tand(10);% restrict the max angle between camera axis and chess board

r = (a^2 + h_^2)/(2*h_);% sphere radius
sphereCenter = [ ( h_+dist )-r, mean( trajectory(2,:) ), cameraHeight ];


[x, y, z] = sphere(resolution);% controls resolution

x= r*x + sphereCenter(1);
y = r*y + sphereCenter(2);
z = r*z + sphereCenter(3);


limx = max( trajectory(1,:) );
% limy = trajectory(2,:);
limy = [-1 1]*0.92;% hardcoded based on experimental setup
limz = max(0.5,cameraHeight-dist_z/2);% some safe distance above ground level


mask = x>limx & y<max(limy) & y>min(limy) & z>limz & z<height;
x = x(mask);
y = y(mask);
z = z(mask);

% sort by height
[z, sortIdx] = sort(z);%,'descend');
x = x(sortIdx);
y = y(sortIdx);

% % plot points
% scatter3( x, y, z, 'blue');
% 
% % plot path
% plot3(x,y,z,'r');

% make poses
target = sphereCenter;


positions = [ x y z ];
maxY = max(positions(:,2));

len = length(x);
poses = cell( len, 1 );
plot3(target(1), target(2), target(3),'ro', 'Markersize', 10);


for i=1:len
    
    
    n = target-positions(i,:); n = n'./norm(n);% get z-axis/ tangent plane normal
    v = [0 0 1]' - n(3)*n; v = v./norm(v);% project y-axis into plane
    u = cross( v, n );% calc last dir
    R = [u v n];
    
    % add random offset of angle based on y-position
    angleAxis = vrrotmat2vec( R );
   
    maxOffset = deg2rad(25 - abs(positions(i,2))*(15/maxY) );    
    angleAxis(end) = angleAxis(end) + genRandomOffset( maxOffset );
    
    R = vrrotvec2mat( angleAxis );

    T = eye(4);
    T(1:3,4) = positions(i,:)';


    T(1:3,1:3) = R;
    
    hold on
    v = T(1:3,3);
    quiver3(positions(i,1),positions(i,2),positions(i,3), v(1),v(2),v(3))

    
    poses{i} = T;
  
end

%% validate to be within image

valid = makeBoard( poses, Tcam1, camera, [h w], trajectory, boardSize, squareSz );
valid = valid & makeBoard( poses, Tcam2, camera, [h w], trajectory, boardSize, squareSz );

fprintf('Made %d configurations\n', sum( valid ) );

poses = poses(valid);

figure(robotFig);

x = x(valid);
y = y(valid);
z = z(valid);
% plot points
scatter3( x, y, z, 'blue', 'filled');

% plot path
plot3(x,y,z,'r');
%% move robot

ik = robotics.InverseKinematics('RigidBodyTree',robot);
% start with bent arm
qn = homeConfiguration(robot);
tmp = num2cell( [qn.JointPosition] );
tmp{2} = -pi/2;
tmp{3} = pi/2;
tmp{5} = pi/2;
tmp{6} = pi;
[qn.JointPosition] = tmp{:};
show(robot, qn,'PreservePlot',false);

% getTransform(robot, qn, 'tool0')
% return;

link2_org = stlread('robot/link_2.stl');
link3_org = stlread('robot/link_3.stl');

link2 = link2_org;
link3 = link3_org;

len = length( poses );
joints = cell(1,len);
final_poses = cell(1,len);
for i=1:len
    
    collision = zeros(2, 2);% 2 fovs, 2 links

    qn = ik('board',poses{i},[0.6 0.6 0.6 1 1 1],qn);
    joints{i} = rad2deg( [qn.JointPosition] );
    final_poses{i} = getTransform(robot, qn, 'tool0');
    show(robot, qn,'PreservePlot',false);

    % make view path
    fov_1 = makeViewPath( positions(i,:)', t1, boardSz*0.7, 'blue' );
    fov_2 = makeViewPath( positions(i,:)', t2, boardSz*0.7, 'green' );
    
%     Trobot = getTransform(robot,qn,'board');
%     plot3( [target(1) Trobot(1,4)], [target(2) Trobot(2,4)], [target(3) Trobot(3,4)] );

    % check for collisions link 2
    T2 = getTransform(robot,qn,'link_2');
    pts = [link2_org.vertices, ones(length(link2.vertices),1)];
    pts = pts*T2';
    link2.vertices = pts(:,1:3);
    l2 = patch(link2);
    
    collision(1,1) = GJK(l2,fov_1,10);
    collision(1,2) = GJK(l2,fov_2,10);
    
    % check for collisions link 3
    T3 = getTransform(robot,qn,'link_3');
    pts = [link3_org.vertices, ones(length(link3.vertices),1)];
    pts = pts*T3';
    link3.vertices = pts(:,1:3);
    l3 = patch(link3);
    
    collision(2,1) = GJK(l3,fov_1,10);
    collision(2,2) = GJK(l3,fov_2,10);

    if any(collision)
        warning(['Collision at ' num2str(i)]);
        
        if any( collision(1,:) )
            l2.FaceColor = 'red';
            fov1.FaceColor = 'blue';
        end
        if any( collision(2,:) )
            l3.FaceColor = 'red';
            fov2.FaceColor = 'green';
        end
    else
        delete(l2);
        delete(l3);
        delete(fov_1);
        delete(fov_2);
    end
    
%     waitforbuttonpress()
%     pause(0.5);
end

%% finish
fprintf('Place cameras at: \n\t[ %.2f, %.2f, %.2f ]\n\t[ %.2f, %.2f, %.2f ]\n', t1, t2 )
fprintf('And point them at [ %.2f, %.2f, %.2f ]\n', trajectory(1,1), 0, t1(end) )

generateKRL(t1, t2, [trajectory(1,1), 0, t1(end)], joints);
save('./data/scene.mat', 'dist', 'poses','final_poses', 'Tcam1', 'sphereCenter', 'trajectory','Tcam2', 'camera');

% additionally save poses to file
fid = fopen('data/calibration_poses.txt', 'wt' );

if fid<0
    error('Could not write calibration poses');
end

% number of images/poses to take
fprintf(fid, '%d\n', length(final_poses) );
% each pose as X Y Z A B C
% func = @(T) T*[rotz(-90) [0 0 0]'; 0 0 0 1 ];
% final_poses = cellfun( func, final_poses, 'UniformOutput', false );

for i=1:length(final_poses)
    ABC = rad2deg( rotm2eul( final_poses{i}(1:3,1:3), 'ZYX' ) );
    fprintf(fid, '%.4f %.4f %.4f %.4f %.4f %.4f\n', final_poses{i}(1:3,4)*1000, ABC );
end

fclose(fid);

hold off

function h = makeViewPath( position, camera, pathSz, color )
    [xc, yc, zc] = cylinder;
    
    v = camera-position;
    lenV = norm(v);
    
    % rotate towards points
    angleAxis = vrrotvec([0 0 1],v./lenV);
    
    R = vrrotvec2mat( angleAxis );
    
    tmp = [pathSz*xc(:) pathSz*yc(:) lenV*zc(:)]*R.';
    sz = size(xc);
    
    xc = reshape(tmp(:,1),sz);
    yc = reshape(tmp(:,2),sz);
    zc = reshape(tmp(:,3),sz);
    
    % move to camera center
    xc = xc + position(1);
    yc = yc + position(2);
    zc = zc + position(3);
    
    h = surface( xc, yc, zc, 'FaceColor','none','EdgeColor',color );

end

function r = genRandomOffset( n )
    % random nr between -1,1
    s = (rand()-0.5 )*2;
    % scale to max
    r = s*n;
end