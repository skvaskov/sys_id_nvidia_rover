function [ varfilter] = filterdiff(var,time,threshold)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vard=diff(var);
idxs=find(abs(vard)>threshold);
idxs=unique([idxs-1 idxs idxs+1]);
timerm=time;
idxs=idxs(idxs>0);
timerm(idxs)=[];
var(idxs)=[];
varfilter=interp1(timerm,smooth(timerm,var),time)';

end

