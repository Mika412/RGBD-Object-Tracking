function image=remove_bg(depth, bg, bgdist, distanceLimit)
    image=abs((double(depth)/1000-bg)) > bgdist;
    image(depth > distanceLimit)=0;