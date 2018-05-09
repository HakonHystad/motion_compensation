function [T_wc1, T_wc2, T_eo, parameters] = findTransformations( stereoParam, T_we )

    len = length(T_we);
    
    T_co1 = cell(len,1);
    T_co2 = T_co1;
    for k = 1:len
        T_co1{k} = [ stereoParam.CameraParameters1.RotationMatrices(:,:,k)',...
                   stereoParam.CameraParameters1.TranslationVectors(k,:)'; 0 0 0 1];
        T_co2{k} = [ stereoParam.CameraParameters2.RotationMatrices(:,:,k)',...
                   stereoParam.CameraParameters2.TranslationVectors(k,:)'; 0 0 0 1];
    end
    %% find world transformation to camera 1
    [A1, B1] = packTwc( T_we, T_co1 );
    T_wc1 = solveAXXB( A1, B1 );
    
    %% find camera 2 to world transformation
    R = stereoParam.RotationOfCamera2';
    t = stereoParam.TranslationOfCamera2';
    
    [A2, B2] = packTwc( T_we, T_co2 );
    
%         T_wc2 = solveAXXB( A2, B2 );

    
    [T_wc1, T_wc2, R, t] = refineRelationship( T_wc1, R, t, A1, B1, A2, B2 );

    
    %% find end-effector to object transformation
    [A, B] = packTeo( T_we, T_co1 );
    T_eo = solveAXXB( A, B );
    
    T_eo = refineX( T_eo, A, B );
    
    % update R,t
    if nargout>3
        parameters = stereoParameters( stereoParam.CameraParameters1,...
                        stereoParam.CameraParameters2, R', t );
    end
    
    
end

function [A,B] = packTwc( T_we, T_co )

    len = length( T_we );
    
    A = cell(len/2,1);
    B = A;
    
    count = 1;
    for i=1:2:len
        A{count} = T_we{i+1}/T_we{i};
        B{count} = T_co{i+1}/T_co{i};
        
        
        count = count +1;
    end
end

function [A,B] = packTeo( T_we, T_co )

    len = length( T_we );
    
    A = cell(len/2,1);
    B = A;
    
    count = 1;
    for i=1:2:len
        A{count} = T_we{i+1}\T_we{i};
        B{count} = T_co{i+1}\T_co{i};

        count = count +1;
    end
end

function T = solveAXXB( A, B )
    len = length(A);
    K_A = zeros( 3, len );
    K_B = K_A;
    for i=1:len
       K_A(:,i) = rotationMatrixToVector( A{i}(1:3,1:3) )';
       K_B(:,i) = rotationMatrixToVector( B{i}(1:3,1:3) )'; 
    end
    
    [ U, ~, V] = svd( K_B*K_A' );
    
    S = [ 1 0 0; 0 1 0; 0 0 det( U*V' ) ];
    R_X = V*S*U';
    
    C = zeros( 3*len, 3 );
    d = zeros( 3*len, 1 );
    count = 1;
    for i=1:len
        C(count:count+2,:) = A{i}(1:3,1:3) - eye(3);
        d(count:count+2) = R_X*B{i}(1:3,4) - A{i}(1:3,4);
        count = count+3;
    end
    
    t_X = C\d;
    
   T = [ R_X t_X; 0 0 0 1 ]; 
end

function [T_wc1, T_wc2, R, t] = refineRelationship( T_wc1, R, t, A1,B1, A2,B2 )
    
    optVec = zeros( 1,12);
    % R_cw1
    optVec(1:3) =  rotationMatrixToVector(T_wc1(1:3,1:3));
    % t_cw1
    optVec(4:6) = T_wc1(1:3,4)';
    % R
    optVec(7:9) = rotationMatrixToVector(R);
    % t
    optVec(10:12) = t';
    
    options = optimset('Algorithm','levenberg-marquardt','Display','off');
    optVec = lsqnonlin(@(p)optimizeDoubleX(p, A1, B1, A2, B2),optVec,[],[],options);
    
    T_wc1 = [ rotationVectorToMatrix(optVec(1:3)) optVec(4:6)'; 0 0 0 1];
    
    R = rotationVectorToMatrix( optVec(7:9) );
    t = optVec(10:12)';

    
%     R_wc2 = R'*T_wc1(1:3,1:3);
%     t_wc2 = -R'*T_wc1(1:3,4) + t;
% %     
%     T_wc2 = [R_wc2 t_wc2; 0 0 0 1];
    T_wc2 = T_wc1*inv([ R t; 0 0 0 1]);
    
    
end

function e = optimizeDoubleX( optVec, A1, B1, A2, B2 )
        X1 = [ rotationVectorToMatrix(optVec(1:3)) optVec(4:6)'; 0 0 0 1];
        
        R = rotationVectorToMatrix( optVec(7:9) );
        t = optVec(10:12)';
        
%         R_wc2 = R'*X1(1:3,1:3);
%         t_wc2 = -R'*X1(1:3,4) + t;
%         X2 = [R_wc2 t_wc2; 0 0 0 1];
          X2 = X1*inv( [R t; 0 0 0 1] );
        
        len = length(A1);
        e = zeros(len,1);
        for i=1:len
            e(i) = norm( A1{i}*X1 - X1*B1{i}, 'fro' ) + norm( A2{i}*X2 - X2*B2{i}, 'fro' );
        end
end

function X = refineX( X, A, B )
    optVec = [ rotationMatrixToVector(X(1:3,1:3)), X(1:3,4)' ];
    
    options = optimset('Algorithm','levenberg-marquardt','Display','off');
    optVec = lsqnonlin(@(p)optimizeX(p, A, B),optVec,[],[],options);
    
    X(1:3,1:3) = rotationVectorToMatrix( optVec(1:3) );
    X(1:3,4) = optVec(4:6)';
end

function e = optimizeX( Xvec, A, B )
    X = [ rotationVectorToMatrix(Xvec(1:3)) Xvec(4:6)'; 0 0 0 1];
    len = length(A);
    
    e = zeros(len,1);
    for i=1:len
        e(i) = norm( A{i}*X - X*B{i}, 'fro' );
    end
end
