% Between Frame Matching
for i = 0:633
    if i < 9
        num1 = sprintf('00%i',i);
        num2 = sprintf('00%i',i+1);
    elseif i == 9
        num1 = sprintf('00%i',i);
        num2 = sprintf('0%i',i+1);
    elseif i < 99
        num1 = sprintf('0%i',i);
        num2 = sprintf('0%i',i+1);
    elseif i == 99
        num1 = sprintf('0%i',i);
        num2 = sprintf('%i',i+1);
    else
        num1 = sprintf('%i',i);
        num2 = sprintf('%i',i+1);
    end
    fl1 = sprintf('../datasets/cmu_16662_p2/sensor_data/left%s.jpg',num1);
    fl2 = sprintf('../datasets/cmu_16662_p2/sensor_data/left%s.jpg',num2);
    fr1 = sprintf('../datasets/cmu_16662_p2/sensor_data/right%s.jpg',num1);
    fr2 = sprintf('../datasets/cmu_16662_p2/sensor_data/right%s.jpg',num2);
    Il1 = rgb2gray(imread(fl1));
    Il2 = rgb2gray(imread(fl2));
    Ir1 = rgb2gray(imread(fr1));
    Ir2 = rgb2gray(imread(fr2));
    pointsl1 = detectSURFFeatures(Il1);
    pointsl2 = detectSURFFeatures(Il2);
    pointsr1 = detectSURFFeatures(Ir1);
    pointsr2 = detectSURFFeatures(Ir2);
    [fl1, vptsl1] = extractFeatures(Il1, pointsl1);
    [fl2, vptsl2] = extractFeatures(Il2, pointsl2);
    [fr1, vptsr1] = extractFeatures(Ir1, pointsr1);
    [fr2, vptsr2] = extractFeatures(Ir2, pointsr2);
    [iP1, matchmetric1] = matchFeatures(fl1, fr1);
    [iP2, matchmetric2] = matchFeatures(fl1, fl2);
    [iP3, matchmetric3] = matchFeatures(fl1, fr2);
    for j = 1:size(iP11,1)
        if(isempty(iP2(iP2(:,1)==iP1(j,1)))||isempty(iP3(iP3(:,1)==iP1(j,1))))
            index1 = [index1;j];
        end
    end
    for j = 1:size(iP2,1)
        if(isempty(iP1(iP1(:,1)==a2(j,1)))||isempty(iP3(iP3(:,1)==iP2(j,1))))
            index2 = [index2;j];
        end
    end
    for j = 1:size(iP3,1)
        if(isempty(iP2(iP2(:,1)==iP3(j,1)))||isempty(iP1(iP1(:,1)==iP3(j,1))))
            index3 = [index3;j];
        end
    end
    c1 = removerows(iP1,'ind',index1);
    c2 = removerows(iP2,'ind',index2);
    c3 = removerows(iP3,'ind',index3);
    matchedPoints1 = vpts1(c1(:, 1), :);
    matchedPoints2 = vpts2(c2(:, 2), :);
    if(i>0)
        clf('reset')
    end
    figure(1); ax = axes;
    showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2,'Parent',ax);
%     title(sprintf('left frame %i',i));
%     getframe(gcf);
end