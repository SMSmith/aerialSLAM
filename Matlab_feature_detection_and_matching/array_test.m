a1 = [1,1;2,3;4,4;7,5];
a2 = [1,1;3,2;4,6;7,8];
a3 = [1,2;2,4;4,7;5,8];
b1 = [1,1;4,4];
b2 = [1,1;4,6];
b3 = [1,2;4,7];
index1 = [];
index2 = [];
index3 = [];
for i = 1:size(a1,1)
    if(isempty(a2(a2(:,1)==a1(i,1)))||isempty(a3(a3(:,1)==a1(i,1))))
        index1 = [index1;i];
    end
end
c1 = removerows(a1,'ind',index1);
for i = 1:size(a2,1)
    if(isempty(a1(a1(:,1)==a2(i,1)))||isempty(a3(a3(:,1)==a2(i,1))))
        index2 = [index2;i];
    end
end
c2 = removerows(a2,'ind',index2);
for i = 1:size(a3,1)
    if(isempty(a2(a2(:,1)==a3(i,1)))||isempty(a1(a1(:,1)==a3(i,1))))
        index3 = [index3;i];
    end
end
c3 = removerows(a3,'ind',index3);