function [xyz, idx] = merge_objects_and_cluster(objects1, objects2, R, T)
    % Get the number of object in each cam
    n_obj1 = size(objects1, 2);
    n_obj2 = size(objects2, 2);

        
    %% Transform objects from cam2 in cam1 frame
    objects2_in_frame1 = [];
    for n_obj = 1:n_obj2,
        xyz2 = objects2(n_obj).pos;
        objects2_in_frame1(n_obj).pos = xyz2*R + repmat(T, size(xyz2,1), 1);
    end
    % objects1(i).objects(n_obj) <===> objects2_in_frame1(n_obj)

    %% Stack data
    pos_data1 = [];
    for n_obj = 1:n_obj1,
        pos_data1 = [pos_data1; objects1(n_obj).pos];
    end

    pos_data2 = [];
    for n_obj = 1:n_obj2,
        pos_data2 = [pos_data2; objects2_in_frame1(n_obj).pos];
    end

    pos_data = [];
    pos_data = [pos_data1; pos_data2];


    %% Manage number of points
    gridStep = 0.03;    % in practice, divides size of dataset by 10
                        % by considering boxes of 3x3x3 cm
    flag_bigcloud = 0;
    if size(pos_data, 1) > 200000
        gridStep = 0.1; % need to reduce a lot
        flag_bigcloud = 1;
    end
    ptCloud = pointCloud(pos_data);
    ptCloudOut = pcdownsample(ptCloud,'gridAverage',gridStep);

    pos_data = ptCloudOut.Location;

    %% DBSCAN
    addpath(genpath('DBSCAN_clustering/DBSCAN_implem'))
    epsilon = 0.1; % 10cm
    if flag_bigcloud == 1;
        epsilon = 0.2;
    end
    MinPts = 10;

    IDX=DBSCAN(pos_data,epsilon,MinPts);
    
    xyz = pos_data;
    idx = IDX;
end
