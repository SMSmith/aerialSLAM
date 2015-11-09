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
    pre = '/home/ben/Documents/MATLAB/sensor_data/';
    fl1 = sprintf('%sleft%s.jpg',pre,num1);
    fl2 = sprintf('%sleft%s.jpg',pre,num2);
    fr1 = sprintf('%sright%s.jpg',pre,num1);
    fr2 = sprintf('%sright%s.jpg',pre,num2);
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
    index1 = [];
    index2 = [];
    index3 = [];
    for j = 1:size(iP1,1)
        t = iP1(j,1);
        if(sum(iP2(:,1)==t)==0||sum(iP3(:,1)==t)==0)
            index1 = [index1;j];
        end
    end
    iP1 = removerows(iP1,'ind',index1);
    for j = 1:size(iP2,1)
        t = iP2(j,1);
        if(sum(iP1(:,1)==t)==0||sum(iP3(:,1)==t)==0)
            index2 = [index2;j];
        end
    end
    iP2 = removerows(iP2,'ind',index2);
    for j = 1:size(iP3,1)
        t = iP3(j,1);
        if(sum(iP1(:,1)==t)==0||sum(iP2(:,1)==t)==0)
            index3 = [index3;j];
        end
    end
    iP3 = removerows(iP3,'ind',index3);
    % iP - [fl1,fr1,fl2,fr2]
    iP = [iP1,iP2(:,2),iP3(:,2)];
    matchedPointsl1 = vptsl1(iP(:, 1), :);
    matchedPointsr1 = vptsr1(iP(:, 2), :);
    matchedPointsl2 = vptsl2(iP(:, 3), :);
    matchedPointsr2 = vptsr2(iP(:, 4), :);
    l1 = matchedPointsl1;
    r1 = matchedPointsr1;
    for j = 1:10
        fprintf('Left Scale:%.3f Sign:%i Orien:%.3f LocX:%.3f LocY:%.3f Metric:%.3f\n',l1.Scale(j),l1.SignOfLaplacian(j),l1.Orientation(j),l1.Location(j,1),l1.Location(j,2),l1.Metric(j));
        fprintf('Right Scale:%.3f Sign:%i Orien:%.3f LocX:%.3f LocY:%.3f Metric:%.3f\n',r1.Scale(j),r1.SignOfLaplacian(j),r1.Orientation(j),r1.Location(j,1),r1.Location(j,2),r1.Metric(j));
    end
    fprintf('line %i 0f 634\n',i+1);
    if(i>0)
        clf('reset')
    end
    figure(1); 
    ax = axes;
    showMatchedFeatures(Il1,Ir1,matchedPointsl1(1:10),matchedPointsr1(1:10),'montage','Parent',ax);
% %     ax1 = axes;
%     subplot(2,1,1)
%     showMatchedFeatures(Il1,Ir1,matchedPointsl1,matchedPointsr1);
% %     showMatchedFeatures(Il1,Ir1,matchedPointsl1,matchedPointsr1,'Parent',ax1);
% %     ax2 = axes;
%     subplot(2,1,2)
%     showMatchedFeatures(Il2,Ir2,matchedPointsl2,matchedPointsr2);
% %     figure(3); ax = axes;
% %     showMatchedFeatures(Il1,Il2,matchedPointsl1,matchedPointsl2,'Parent',ax);
% %     figure(4); ax = axes;
% %     showMatchedFeatures(Ir1,Ir2,matchedPointsr1,matchedPointsr2,'Parent',ax);
%     pause;
end