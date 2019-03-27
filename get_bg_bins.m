function fim=get_bg_bins(path)

imgmed=zeros(480,640,length(path));

for i=1:length(path)
    load(path(i).depth);
    imgmed(:,:,i)=double(depth_array)/1000;
end


fam=zeros(480,640);
for i= 1:480
    for j = 1:640
        [N,edges] = histcounts(imgmed(i,j,:), 20);
        [m, idx] = max(N);
        f = (edges(idx)+edges(idx+1))/2;
        fam(i,j) = f;
    end
end
fim = fam;
