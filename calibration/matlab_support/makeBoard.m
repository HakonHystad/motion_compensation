function valid = makeBoard( poses, Tcam1, camera, imSz, trajectory, boardSize, squareSz )

valid = zeros(1,length(poses), 'logical' );
% load scene.mat% dist, poses, Tcam1, trajectory

% transform to camera reference frame
func = @(T) Tcam1*T;
poses_cam = cellfun( func, poses, 'UniformOutput', false );

% make board

n_row = boardSize(1);
n_col = boardSize(2);

% 
% camera= [   1305.599976, 0.000000, 505.700012, 0.000000;
%             0.000000, 1306.599976, 371.399994, 0.000000;
%             0.000000, 0.000000, 1.000000, 0.000000 ];
        

worldCheckerPts = generateCheckerboardPoints([n_row n_col],squareSz*1e3)/1e3;% m
% move to center midle
centroid = mean( worldCheckerPts );
worldCheckerPts = worldCheckerPts - centroid;

worldCheckerPts = [worldCheckerPts zeros( length(worldCheckerPts), 1 )];


trajectory = Tcam1*[ trajectory; 1 1 ];

im = zeros( imSz(1), imSz(2) );


figure, subplot(1,2,1);
cam1 = plotCamera('Opacity', 0.6, 'Color', 'blue', 'Size', 0.25, 'AxesVisible', true );
hold on
plot3( trajectory(1,:), trajectory(2,:), trajectory(3,:), 'c--' ); 

% angle = 42/2;
% scale = 4;
% plot3( [0 sind(angle)]*scale, [0 0], [0 cosd(angle)]*scale )
% plot3( [0 -sind(angle)]*2, [0 0], [0 cosd(angle)]*2 )

hold off

trajectory = camera*trajectory;
trajectory = trajectory./repmat( trajectory(3,:), 3, 1 );

subplot(1,2,2)
imshow(im);
hold on
plot( trajectory(1,:), trajectory(2,:), 'c--' );
hold off

for i=1:length(poses)
    
    worldPts = poses_cam{i}*[worldCheckerPts ones(length(worldCheckerPts),1)]';

    pts = camera*worldPts;
    pts = pts./repmat( pts(3,:), 3, 1);
    
    
    if any( pts(1,:)<0 | pts(1,:)>imSz(2) | pts(2,:)<0 | pts(2,:)>imSz(1) )
        continue;
    end
    
    subplot(1,2,1)
    hold on
    h = plot3( worldPts(1,:), worldPts(2,:), worldPts(3,:), '.' );
    text( worldPts(1,1), worldPts(2,1), worldPts(3,1), num2str(i) );
    hold off
    
    subplot(1,2,2)
    hold on
    plot( pts(1,:), pts(2,:), '.', 'Color', h.Color );
    hold off
    
    valid(i) = true;
    
    
end

hold off

end
