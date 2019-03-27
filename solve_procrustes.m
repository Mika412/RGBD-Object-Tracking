function [R, T] = solve_procrustes(im1_rgb, im2_rgb, im1_depth, im2_depth, peak_thresh, dist_ratio_thresh, errorthresh)
    %% SIFT POINTS
    % Convert into grayscales
    I1 = single(rgb2gray(im1_rgb));
    I2 = single(rgb2gray(im2_rgb));

    % SIFT points
    [f1,d1] = vl_sift(I1, 'PeakThresh', peak_thresh) ;
    [f2,d2] = vl_sift(I2, 'PeakThresh', peak_thresh) ;


    %% MATCH SIFT POINTS IN BOTH IMAGES
    [matches, scores] = vl_ubcmatch(d1, d2, dist_ratio_thresh); 

    
    % Align u,v matches
    uv1_matched = round(f1(1:2, matches(1,:)));
    uv2_matched = round(f2(1:2, matches(2,:)));

    %% Find the corresponding XYZ matches
    K=[525 0 319.5;
        0 525 239.5;
        0 0 1];


    xyz1=get_xyzasus(im1_depth(:),[480 640],1:640*480,K,1,0);
    xyz2=get_xyzasus(im2_depth(:),[480 640],1:640*480,K,1,0);

    load calib_asus
    uv1_from_xyz = get_uv_from_xyz(xyz1, im1_rgb, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    uv2_from_xyz = get_uv_from_xyz(xyz2, im2_rgb, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);

    % Among the uv projected from xyz, find which ones correspond to SIFT
    % points
    % for IMAGE 1
    indices_sift1 = zeros(size(uv1_matched, 2), 1);
    n_points = size(uv1_from_xyz, 1);
    for i=1:size(uv1_matched, 2)
        repeted = repmat(uv1_matched(:,i)', n_points, 1);
        diff = abs(repeted - uv1_from_xyz);
        diff_norm2 = sum(diff .* diff, 2)';
        [min, idx] = max(-diff_norm2);
        indices_sift1(i) = idx;
    end

    % for IMAGE 2
    indices_sift2 = zeros(size(uv2_matched, 2), 1);
    n_points = size(uv2_from_xyz, 1);
    for i=1:size(uv1_matched, 2)
        repeted = repmat(uv2_matched(:,i)', n_points, 1);
        diff = abs(repeted - uv2_from_xyz);
        diff_norm2 = sum(diff .* diff, 2)';
        [min, idx] = max(-diff_norm2);
        indices_sift2(i) = idx;
    end

    % Find the corresponding xyz
    xyz1_matched = xyz1(indices_sift1, :);
    xyz2_matched = xyz2(indices_sift2, :);

    


    %%
    xyz_final = xyz1_matched;
    xyz2_final = xyz2_matched;

    n_total = size(xyz_final, 1);

    % Apply ransac + procrustes
    niter=500;
    %generate sets of 4 points (randomly selected)
    random_indexes = zeros(niter, 4);
    for i=1:niter,
        random_indexes(i,:) = randperm(n_total, 4);
    end

    %begin
    numinliers=[];
    for i=1:niter-4,
        xyz_sample=xyz_final(random_indexes(i,:),:);
        xyz2_sample=xyz2_final(random_indexes(i,:),:);

        [d, Z, transform] = procrustes(xyz_sample,xyz2_sample, 'scaling',false,'reflection',false);
        xyz_reco = xyz2_final*transform.T + repmat(transform.c(1,:), n_total, 1);

        error_vector=xyz_final - xyz_reco;
        err_norm = sqrt(error_vector(:,1).^2 + error_vector(:,2).^2 + error_vector(:,3).^2);
        inds=find(err_norm<errorthresh);


        numinliers=[numinliers length(inds)];
    end

    % get index for which we have maximum number of inliers

    [m, idx] = max(numinliers);

    % find the inliers according to this model
    xyz_sample=xyz_final(random_indexes(idx,:),:);
    xyz2_sample=xyz2_final(random_indexes(idx,:),:);

    [d, Z, transform] = procrustes(xyz_sample,xyz2_sample, 'scaling',false,'reflection',false);
    xyz_reco = xyz2_final*transform.T + repmat(transform.c(1,:), n_total, 1);

    error_vector=xyz_final - xyz_reco;
    err_norm = sqrt(error_vector(:,1).^2 + error_vector(:,2).^2 + error_vector(:,3).^2);
    ind_inliers=find(err_norm<errorthresh);
    n_inliers_final = length(ind_inliers);

    % FINAL MODEL WITH INLIERS
    xyz_inliers = xyz_final(ind_inliers,:);
    xyz2_inliers = xyz2_final(ind_inliers,:);

    [d, Z, transform] = procrustes(xyz_inliers,xyz2_inliers, 'scaling',false,'reflection',false);
    R = transform.T;
    T = transform.c(1,:);
    
    % NB : to transform xyz in cam2 to xyz in cam1:
    % xyz2_in_cam1_coord = xyz2*R + repmat(T, size(xyz2,1), 1);
end