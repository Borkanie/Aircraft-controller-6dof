function [currentPoint] = FindCurrentOperatingPoint(winds,operatingPointList,tol)
for i=1:length(operatingPointList)
    if round(winds,tol)==round(operatingPointList(i).PointOfOperation.Winds,tol)
        currentPoint=operatingPointList(i);
    end
end
end