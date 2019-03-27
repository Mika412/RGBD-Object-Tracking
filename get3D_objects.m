function [savedImageObjects, savedObjectCloud] = get3D_objects(imgseq1, cam_params, background, bgdist, maxDistanceRemove, objsize, percentageBiggestThreshold)
    for i=1:length(imgseq1)
        %Load image and images depth data
        load(imgseq1(i).depth);
        main_image =imread(imgseq1(i).rgb);
        
        %Remove the background from the images
        image=remove_bg(depth_array, background , bgdist, maxDistanceRemove);

        %Remove all connected points that're smaller than the threshold
        image = bwareaopen(image,objsize);
        image = imfill(image,'holes');
        image = image.*double(depth_array);
        
        %Calculate the gradient of the depth data
        gradient = imgradient(image);
        gradient(gradient < 300) = 0;
        
        %Increase the gradient to all 8-connected points
        h = [1 1 1]; h2 = [1; 1; 1;];
        gradient = imfilter(gradient,h2);
        gradient = imfilter(gradient,transpose(h2));
        
        %
        image = image.*~gradient;

        % Calculate all 8-connected objects 
        [L1, objectsCount]=bwlabel(image,8);
        image = imfill(L1,'holes');


        tempCount = zeros(5,1);
        for c = 1:objectsCount
            tempCount(c) = sum(L1(:) == c);
        end
        
        % Remove all objects that are smaller then % of the biggest object
        % in the scene
        biggestNumPoints = max(tempCount);
        pointsThreshold = biggestNumPoints * percentageBiggestThreshold;
        idxPossible= find(tempCount > pointsThreshold);
        for c = 1:objectsCount
            if ~any(idxPossible(:) == c)
                L1(L1 == c) = 0;
            end
        end

        savedObjects = zeros(length(idxPossible), 8, 3);
        count = 1;

        savedObjectCloud(i).objects = [];
        savedImageObjects(i).objects = [];
        %Get images all object boundaries
        for c = 1:numel(idxPossible)
            
            %Get only the ´c´ number object
            k = idxPossible(c);
            objectsD = L1 == k;
            objectsD(objectsD > 0) = 1;
            objectsD = objectsD .* double(depth_array);
            
            %Convert to 3D cloud
            xyz_object = get_xyzasus(objectsD(:),[480 640],1:480*640,cam_params.Kdepth,1,0);
            indsObj=find(xyz_object(:,3)~=0);
            xyz_object=xyz_object(indsObj,:);
            
            %Get the box boundaries
            p = get_BoxPoints(xyz_object);
            
            %Save to temporary object
            savedObjects(count,:,:)= p;
            savedObjectCloud(i).objects(count).pos = xyz_object;
            count = count+1;
        end
        savedImageObjects(i).objects = savedObjects;
    end