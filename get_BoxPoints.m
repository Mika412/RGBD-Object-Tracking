function p = get_BoxPoints(xyz_object)
    minX = min(xyz_object(:,1));
    maxX = max(xyz_object(:,1));

    minY = min(xyz_object(:,2));
    maxY = max(xyz_object(:,2));

    minZ = min(xyz_object(:,3));
    maxZ = max(xyz_object(:,3));
    
    p=[[minX, minY , minZ];[minX, minY , maxZ];[minX, maxY , minZ];[minX, maxY , maxZ];[maxX, minY , minZ];[maxX, minY , maxZ];[maxX, maxY , minZ];[maxX, maxY , maxZ]];

   