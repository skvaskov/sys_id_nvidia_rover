function [position] = centroid_topoint(centroid,R,offset)
%take poisition of centroid and convert it to another poistion on the rove
szR=size(centroid);
position=zeros(3,szR(2));

for i=1:szR(2)
    position(:,i)=centroid(:,i)+R{i}*offset;
end

end

