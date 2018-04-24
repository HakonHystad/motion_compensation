function generateKRL( t1, t2, aim, joints )

    fid = fopen('data/joint_configs.txt', 'wt' );

    if fid<0
        error('Could not write joint_configs');
    end

    fprintf(fid, 'DEF calibrate()\n');
    fprintf(fid, 'INI\n');
    
    % home
    qh = [0    30   -30         0         0         0];
    fprintf(fid, 'PTP {AXIS: A1 %.4f, A2 %.4f, A3 %.4f, A4 %.4f, A5 %.4f, A6 %.4f}\n', qh);
    
    % show where to place camera 1
    fprintf(fid, 'PTP {X %.4f, Y %.4f, Z %.4f}\n', t1 );
    % show where to place camera 2
    fprintf(fid, 'PTP {X %.4f, Y %.4f, Z %.4f}\n', t2 );
    % show where to aim cameras
    % show where to place camera 1
    fprintf(fid, 'PTP {X %.4f, Y %.4f, Z %.4f}\n', aim );
    
    
    % move to all calibration positions
    for i=1:length(joints)
        fprintf(fid, 'PTP {AXIS: A1 %.4f, A2 %.4f, A3 %.4f, A4 %.4f, A5 %.4f, A6 %.4f}\n', joints{i} );
    end
    
    fprintf(fid,'END');
    
    fclose(fid);
end