function [parameters, T_eo, T_cw] = estimateHH( initialParameters, imPts, T_eo, T_we, T_cw, correctPoints )
    if nargin<8
        correctPoints = false;
    end
    
    pattern = initialParameters.WorldPoints;
    len = length( pattern );
    pattern = [ pattern'; zeros(1, len); ones(1,len) ];
    
    initialParameters = toStruct( initialParameters );

    
    optVec = pack( initialParameters, T_eo, T_cw );
    
    options = optimset('Algorithm','levenberg-marquardt','Display','off');
    optVec = lsqnonlin(@(p)optimizeHH(p,pattern, imPts, T_we, correctPoints),optVec,[],[],options);
    
    [R, t, K1, k1, K2, k2, T_eo, T_cw] = unpack( optVec );
    parameters = initialParameters;
    
    parameters.RotationOfCamera2 = R';
    parameters.TranslationOfCamera2 = t';
    
    parameters.CameraParameters1.IntrinsicMatrix = K1';
    parameters.CameraParameters1.RadialDistortion = k1;
    parameters.CameraParameters2.IntrinsicMatrix = K2';
    parameters.CameraParameters2.RadialDistortion = k2;
    
    parameters = stereoParameters( parameters );


end% estimateHH

function cost = optimizeHH( optVec, pattern, imPts, T_we, correctPoints )
    len = length(T_we);
    npts = length( pattern );
    cost = zeros( 1, len*npts );

    [R, t, K1, k1, K2, k2, T_eo, T_cw] = unpack( optVec );
    
    if correctPoints
        F = K2'\Skew(t)*R/K1;

        for i=1:npts
            for j=1:len
                [x, xp] = correctedCorrespondance([imPts(i,:,j,1),1],[imPts(i,:,j,2),1],F);
                imPts(i,:,j,1) = x(1:2);
                imPts(i,:,j,2) = xp(1:2);
            end
        end

        % TODO undistort?
    end
    
   
    cs = 1;
    ce = npts;
    for i=1:len

        % build cameras
        T = T_cw*T_we{i}*T_eo;
        cam1 = [ K1 [0 0 0]' ]*T;
        cam2 = [ K2 [0 0 0]' ]*[R t; 0 0 0 1]*T;
        
        pattern1 = project( cam1, pattern );
        pattern2 = project( cam2, pattern );
        diff = sqrt( sum( ( pattern1(1:2,:) - imPts(:,:,i,1)' ).^2 , 1 ) );% norm
        diff = diff + sqrt( sum( ( pattern2(1:2,:) - imPts(:,:,i,2)' ).^2 , 1 ) );
        
        cost( cs:ce ) = diff;
        cs = ce +1;
        ce = cs + npts-1;
    end
    

end% optimizeHH

function pts = project( cam, wpts )
    pts = cam*wpts;
    pts = pts./repmat( pts(3,:), 3, 1);
end

function optVec = pack( initialParameters, T_eo, T_cw )
% R, t, K1,k1, K2,k2, R_eo, T_eo, R_wc, T_cw
    optVec = zeros( 2*(4+2) + 3*(3+3), 1 );
    
    cs = 1;
    ce = 3;
    
    % R
    optVec(cs:ce) = rotationMatrixToVector( initialParameters.RotationOfCamera2' )';
    % t
    cs = ce+1;
    ce = cs +3 -1;
    optVec(cs:ce) = initialParameters.TranslationOfCamera2';
    % K1
    cs = ce+1;
    ce = cs+4-1;
    K = initialParameters.CameraParameters1.IntrinsicMatrix';
    optVec(cs:ce) = [ K(1,1); K(2,2); K(1:2,3) ];
    % k1
    cs = ce+1;
    ce = cs+2-1;
    optVec(cs:ce) = initialParameters.CameraParameters1.RadialDistortion';
    % K2
    cs = ce+1;
    ce = cs+4-1;
    K = initialParameters.CameraParameters2.IntrinsicMatrix';
    optVec(cs:ce) = [ K(1,1); K(2,2); K(1:2,3) ];
    % k2
    cs = ce+1;
    ce = cs+2-1;
    optVec(cs:ce) = initialParameters.CameraParameters2.RadialDistortion';
    % R_oe
    cs = ce+1;
    ce = cs+3-1;
    optVec(cs:ce) = rotationMatrixToVector( T_eo(1:3,1:3) );
    % T_eo
    cs = ce+1;
    ce = cs+3-1;
    optVec(cs:ce) = T_eo(1:3,4);
    % R_wc
    cs = ce+1;
    ce = cs+3-1;
    optVec(cs:ce) = rotationMatrixToVector( T_cw(1:3,1:3) );
    % T_cw
    cs = ce+1;
    ce = cs+3-1;
    optVec(cs:ce) = T_cw(1:3,4);

    
end% pack

function [R, t, K1, k1, K2, k2, T_eo, T_cw] = unpack( optVec  )
    
    % R
    cs = 1;
    ce = 3;
    R = rotationVectorToMatrix( optVec(cs:ce)' );
    % t
    cs = ce +1;
    ce = cs+3 -1;
    t = optVec(cs:ce);
    % K1
    cs = ce +1;
    ce = cs+4 -1;
    K1 = [  optVec(cs)   0           optVec(cs+2);...
        0           optVec(cs+1)  optVec(cs+3);...
        0           0           1];
    % k1
    cs = ce +1;
    ce = cs+2 -1;
    k1 = optVec(cs:ce);
    % K2
    cs = ce +1;
    ce = cs+4 -1;
    K2 = [  optVec(cs)   0           optVec(cs+2);...
        0           optVec(cs+1)  optVec(cs+3);...
        0           0           1];
    % k2
    cs = ce +1;
    ce = cs+2 -1;
    k2 = optVec(cs:ce);
    % R_oe
    cs = ce +1;
    ce = cs+3-1;
    R_oe = rotationVectorToMatrix( optVec(cs:ce) );
    % T_eo
    cs = ce +1;
    ce = cs+3-1;
    T_eo = optVec(cs:ce);
    % R_wc
    cs = ce +1;
    ce = cs+3-1;
    R_wc = rotationVectorToMatrix( optVec(cs:ce) );
    % T_cw
    cs = ce +1;
    ce = cs+3-1;
    T_cw = optVec(cs:ce);
    
    T_eo = [ R_oe T_eo; 0 0 0 1];
    T_cw = [ R_wc T_cw; 0 0 0 1];
    
end% unpack