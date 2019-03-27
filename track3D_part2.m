%path='maizena2/data_rgb/';
function [objects, cam2toW]= track3D_part2(imgseq1, imgseq2, cam_params)

    
    % SET PARAMETERS
    bgdist=0.25;    % Background distance to remove
    maxDistanceRemove = 4000; % Max distance of objects to consider
    objsize=2000;   % Minimum size of the objects
    percentageBiggestThreshold = 0.15;  % Threshold percentage of the biggest object
    distanceBetwenObjectsThreshold = 0.6;   %   Threshold to consider as the same object between the images
    peak_thresh = 3;         % SIFT parameter
    dist_ratio_thresh = 1.3; % distance ratio treshold for ubcmatch
    errorthresh=0.3;        % error threshold for inliers acceptance in RANSAC

    objects = struct('X',{},'Y',{},'Z',{},'frames_tracked',{});
    cam2toW = struct('R','T');
    
    
    
    
    %% Solve the Procrustes to be able to put all points in same frame
    % BUT FOR A SINGLE IMAGE
    % IMPORTS 
    run('VLFEATROOT/toolbox/vl_setup')

    i1_rgb = imgseq1(1).rgb;
    i2_rgb = imgseq2(1).rgb;

    i1_depth = imgseq1(1).depth;
    i2_depth = imgseq2(1).depth;

    % Load the 2 images
    im1_rgb=imread(i1_rgb);
    im2_rgb=imread(i2_rgb);

    
    load(i1_depth)
    im1_depth=depth_array;
    load(i2_depth)
    im2_depth=depth_array;

    % RUN SIFT + RANSAC + PROCRUSTES
    [R, T] = solve_procrustes(im1_rgb, im2_rgb, im1_depth, im2_depth, peak_thresh, dist_ratio_thresh, errorthresh);
    
    cam2toW.R = R;
    cam2toW.T = T;
    
    %% GET THE BACKGROUND
    background1 = get_bg_bins(imgseq1);
    background2 = get_bg_bins(imgseq2);
    
    %% Get the objects xyz coordinates and their respective box boundaries
    
    % For camera 1
    [savedImageObjects1, savedObjectCloud1] = get3D_objects(imgseq1, cam_params, background1, bgdist, maxDistanceRemove, objsize, percentageBiggestThreshold);

    % For camera 1
    [savedImageObjects2, savedObjectCloud2] = get3D_objects(imgseq2, cam_params, background2, bgdist, maxDistanceRemove, objsize, percentageBiggestThreshold);
    
    %% JOIN ALL BOTH POINT CLOUD OBJECTS
    savedImageObjects = struct('objects',{});
    
    
    for i=1:length(imgseq1)
        n_obj1 = size(savedImageObjects1(i).objects, 1);
        n_obj2 = size(savedImageObjects2(i).objects, 1);

        %% Call merge and cluster
        objects1 = savedObjectCloud1(i).objects;
        objects2 = savedObjectCloud2(i).objects;

        savedImageObjects(i).objects = [];
        
        if length(objects1) ~= 0 && length(objects2) ~= 0
        [xyz, idx] = merge_objects_and_cluster(objects1, objects2, R, T);

            allUnique = unique(idx);
            for j=1:numel(allUnique)
                k = allUnique(j);

                indsObj=find(idx(:) == k);


                xyz_object=xyz(indsObj,:);
                p1=pointCloud(xyz_object);
                
                p = get_BoxPoints(xyz_object);
                savedObjects(j,:,:)= p;
            end
            savedImageObjects(i).objects = savedObjects;
        end
    end
    
    %% COPY FIRST IMAGE OBJECTS TO THE STRUCTURE
    %Copies the objects of the first image into the ´objects´ structure
    for objIndex=1:size(savedImageObjects(1).objects,1)
        objects(objIndex).X(1,:) = savedImageObjects(1).objects(objIndex,:,1).';
        objects(objIndex).Y(1,:) = savedImageObjects(1).objects(objIndex,:,2).';
        objects(objIndex).Z(1,:) = savedImageObjects(1).objects(objIndex,:,3).';

        objects(objIndex).frames_tracked = [1];
    end
     %% PROCESS AND SEND TO FINAL STRUCTURE
    for i=2:length(imgseq1)
        
        colCorrespondance= struct('cor',{});
        rowCorrespondance= struct('cor',{});
        
        %Gets the previous image objects
        previousImageObjectsIndexes = [];
        for objIndex = 1: length(objects)
            if any(objects(objIndex).frames_tracked == (i-1))
                previousImageObjectsIndexes = [previousImageObjectsIndexes; objIndex];
            end
        end
        
        difMatrix = zeros(length(previousImageObjectsIndexes), size(savedImageObjects(i).objects,1));
        
        % Create the euclidean difference matrix between the current
        % image's object and the previous image's objects
        if size(savedImageObjects(i).objects,1) >0 && length(previousImageObjectsIndexes) >0
            for objeApos=1:length(previousImageObjectsIndexes)
                %Calculates the center point of the previous image objects
                xyz_arrayPos = find(objects(previousImageObjectsIndexes(objeApos)).frames_tracked == (i-1));
                objAcenterX = max(objects(previousImageObjectsIndexes(objeApos)).X(xyz_arrayPos,:)) - min(objects(previousImageObjectsIndexes(objeApos)).X(xyz_arrayPos,:));
                objAcenterY = max(objects(previousImageObjectsIndexes(objeApos)).Y(xyz_arrayPos,:)) - min(objects(previousImageObjectsIndexes(objeApos)).Y(xyz_arrayPos,:));
                objAcenterZ = max(objects(previousImageObjectsIndexes(objeApos)).Z(xyz_arrayPos,:)) - min(objects(previousImageObjectsIndexes(objeApos)).Z(xyz_arrayPos,:));

                for objeBpos=1:size(savedImageObjects(i).objects,1)
                    
                    %Calculates the center point of the current image
                    %objects
                    objBcenterX = max(savedImageObjects(i).objects(objeBpos,:,1)) - min(savedImageObjects(i).objects(objeBpos,:,1));
                    objBcenterY = max(savedImageObjects(i).objects(objeBpos,:,2)) - min(savedImageObjects(i).objects(objeBpos,:,2));
                    objBcenterZ = max(savedImageObjects(i).objects(objeBpos,:,3)) - min(savedImageObjects(i).objects(objeBpos,:,3));

                    %Calculate the euclidean distance between the objects
                    distance = (objAcenterX-objBcenterX).^2 + (objAcenterY-objBcenterY).^2 + (objAcenterZ - objBcenterZ).^2;
                  
                    difMatrix(objeApos, objeBpos) = double(distance);
                end
            end
        end
        
        %Check if distance between frame objects is bigger than threshold
        
        %Check threshold for rows
        rowsToRemove = [];
        colsToRemove = [];
        rowCount = 1;
        for iterPos=1:size(difMatrix,1)
            if ~any(difMatrix(difMatrix(iterPos, :) < distanceBetwenObjectsThreshold))
                rowsToRemove = [rowsToRemove; iterPos];
            else
                rowCorrespondance(rowCount).cor = iterPos;
                rowCount = rowCount + 1;
            end
        end
        %Delete previously defined rows
        difMatrix(rowsToRemove, :) = [];
        
        %Check threshold for cols
        colCount = 1;
        for iterPos=1:size(difMatrix,2)
            if ~any(difMatrix(difMatrix(:, iterPos) < distanceBetwenObjectsThreshold))
                
                colsToRemove = [colsToRemove; iterPos];
                %Copy elements to the final object structure
                newPosIdx = length(objects)+1;
                objects(newPosIdx).X(1,:) = savedImageObjects(i).objects(iterPos,:,1);
                objects(newPosIdx).Y(1,:) = savedImageObjects(i).objects(iterPos,:,2);
                objects(newPosIdx).Z(1,:) = savedImageObjects(i).objects(iterPos,:,3);
                objects(newPosIdx).frames_tracked = [objects(newPosIdx).frames_tracked;i];
            else
                colCorrespondance(colCount).cor = iterPos;
                colCount = colCount + 1;
            end
        end
        
        %Delete previously defined cols
        difMatrix(:, colsToRemove) = [];
        
        %Run the Hungarian Algorithm to assign previous image's objects to
        %current image object
        [assignment,cost] = munkres(difMatrix);
        
        %Save the new objects into the ´objects´ structure
        for colIndex=1:size(colCorrespondance,2)
            
            if colIndex == 0
                continue
            end
            % Get the image object index from the assignment matrix
            objePos = colCorrespondance(colIndex).cor;
            indxObjectPosition = find(assignment(:,colIndex)==1);
            if ~isempty(indxObjectPosition)
                indxObjectPosition = rowCorrespondance(indxObjectPosition).cor;
            end
            
            %If the object exists, add to the existing one, if not create
            %new object index
            if ~isempty(indxObjectPosition)
                posPos = length(objects(previousImageObjectsIndexes(indxObjectPosition)).frames_tracked)+1;
                objects(previousImageObjectsIndexes(indxObjectPosition)).X(posPos,:) = savedImageObjects(i).objects(objePos,:,1);
                objects(previousImageObjectsIndexes(indxObjectPosition)).Y(posPos,:) = savedImageObjects(i).objects(objePos,:,2);
                objects(previousImageObjectsIndexes(indxObjectPosition)).Z(posPos,:) = savedImageObjects(i).objects(objePos,:,3);
                objects(previousImageObjectsIndexes(indxObjectPosition)).frames_tracked = [objects(previousImageObjectsIndexes(indxObjectPosition)).frames_tracked; i];
            else
                newPosIdx = length(objects)+1;
                objects(newPosIdx).X(1,:) = savedImageObjects(i).objects(objePos,:,1);
                objects(newPosIdx).Y(1,:) = savedImageObjects(i).objects(objePos,:,2);
                objects(newPosIdx).Z(1,:) = savedImageObjects(i).objects(objePos,:,3);
                objects(newPosIdx).frames_tracked = [objects(newPosIdx).frames_tracked;i];
            end
        end
    end