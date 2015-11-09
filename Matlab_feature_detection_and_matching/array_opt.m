a1 = [1,1;2,3;4,4;7,5];
a2 = [1,1;3,2;4,6;5,7;7,8];
a3 = [1,2;2,4;4,7;5,8];
b1 = [1,1;4,4];
b2 = [1,1;4,6];
b3 = [1,2;4,7];
un = [a1(:,1);a2(:,1);a3(:,1)];
in1 = [];
for j = 1:size(a1,1)
    t = a1(j,1);
    if(sum(a2(:,1)==t)==0||sum(a3(:,1)==t)==0)
        in1 = [in1;j];
    end
end
a1 = removerows(a1,'ind',in1);
in2 = [];
for j = 1:size(a2,1)
    t = a2(j,1);
    if(sum(a1(:,1)==t)==0||sum(a3(:,1)==t)==0)
        in2 = [in2;j];
    end
end
a2 = removerows(a2,'ind',in2);
in3 = [];
for j = 1:size(a3,1)
    t = a3(j,1);
    if(sum(a1(:,1)==t)==0||sum(a2(:,1)==t)==0)
        in3 = [in3;j];
    end
end
a3 = removerows(a3,'ind',in3);
c = [a1,a2(:,2),a3(:,2)]
% 
% m12 = a1(:,1) == a2(:,1);
% ind12 = find(m12==0);
% m12 = removerows(m12,'ind',ind12);
% m13 = a1(:,1) == a3(:,1);
% ind12 = find(m12==0);
% m12 = removerows(m12,'ind',ind12);
% m23 = a1(:,1) == a3(:,1);
