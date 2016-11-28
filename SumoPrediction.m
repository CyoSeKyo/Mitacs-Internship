function [Trace, FinaLocation, Prob] = SumoPrediction(City, PD_likehood, CrossRoad, ParaRoad, StreetWidth, VehicleID, CurrentLocation, PastRoad, RemainTime)

Len = length(PastRoad);    %Length of Pastrace
StrNum = City.NumOfStreets;

nowRoad = zeros(1, Len);   %Transform Intersections to Streets
lenow = Len;

for i = 1 : Len - 1
    from = PastRoad(i);
    to = PastRoad(i + 1);
    for j = 1 : StrNum
        if (City.Street(j, 1) == from && City.Street(j, 2) == to)
            nowRoad(i) = j;
        end
    end
end

%Current_Street = Get_Street(City.IntsCoordinates, City.Street, CurrentLocation(1), CurrentLocation(2), StrNum, StreetWidth);
%sx = City.IntsCoordinates(PastRoad(Len), 1);
%sy = City.IntsCoordinates(PastRoad(Len), 2);
%if (City.IntsCoordinates(City.Street(Current_Street, 1), 1) == City.IntsCoordinates(City.Street(Current_Street, 2), 1))
%    index = 1;
%else
%    index = 2;
%end
%Current_Street = Current_Street - Get_Direction(index, sx, CurrentLocation(1), sy, CurrentLocation(2));
from = PastRoad(Len);
com = -1000; index = 0;
for i = 1 : StrNum    
    if (City.Street(i, 1) == from)
        si = City.IntsCoordinates(City.Street(i, 1), :);
        ti = City.IntsCoordinates(City.Street(i, 2), :);
        sx = si(1); sy = si(2); ex = ti(1); ey = ti(2);
        covA = [ex - sx, ey - sy];
        covB = [CurrentLocation(1) - sx, CurrentLocation(2) - sy];
        angle = covA(1) * covB(1) + covA(2) * covB(2);
        angle = angle / sqrt(covA(1) * covA(1) + covA(2) * covA(2));
        angle = angle / sqrt(covB(1) * covB(1) + covB(2) * covB(2));
        if (angle > com)
            com = angle; index = i;
        end
    end
end

nowRoad(Len) = index;

SufTime = RemainTime;
diffx = abs(CurrentLocation(1) - sx); diffy = abs(CurrentLocation(2) - sy);

Trace = nowRoad(lenow);
FinaLocation = zeros(1, 2); %First -- X_coordinate; Second -- Y_coordinate
Prob = 1; LastProb = 1;

while(1)
    
    %Predict next Street
    curStreet = nowRoad(lenow);
    nextStreet = 0;  %The next street the vehicle will go with the highest probability
    nextSta = 0;
    for j = max(1, lenow - 2) : lenow - 1
        nextSta = nextSta * 4 + Find_Direction(City.IntsCoordinates, City.Street, nowRoad(j), nowRoad(j + 1));
    end
    MaxPro = 0; sumPro = 0;
    for l = 1 : 3
        suf = Find_Street(City.IntsCoordinates, City.Street, nowRoad(lenow), l, StrNum);
        if (suf == 0)
            continue;
        end
        if (PD_likehood(suf, nextSta * 4 + l) > MaxPro)
            MaxPro = PD_likehood(suf, nextSta * 4 + l);
            nextStreet = suf;
        end
        sumPro = sumPro + PD_likehood(suf, nextSta * 4 + l);
    end
    
    %Calculate the Position of the Vehicle
    Next_Position = 0;
    if (lenow == Len)
        RemainTime = ParaRoad(curStreet, 1) + ParaRoad(curStreet, 2) + ParaRoad(curStreet, 3) - max(diffx, diffy) / ParaRoad(curStreet, 4);
    else
        RemainTime = ParaRoad(curStreet, 1) + ParaRoad(curStreet, 2) + ParaRoad(curStreet, 3);
    end
    
    if (SufTime <= RemainTime)
        En = min(SufTime, ParaRoad(curStreet, 3));
        Accelerate = ParaRoad(curStreet, 4) / ParaRoad(curStreet, 3);
        Next_Position = Accelerate * En * En / 2;
        SufTime = SufTime - En;
        if (lenow > Len)
            Bg = min(SufTime, ParaRoad(curStreet, 1));
            Accelerate = ParaRoad(curStreet, 4) / ParaRoad(curStreet, 1);
            Next_Position = Next_Position + Accelerate * Bg * Bg / 2;
            SufTime = SufTime - Bg;
        end
        Next_Position = Next_Position + SufTime * ParaRoad(curStreet, 4);
        %Final Location
        FinaLocation = Get_FlLocation(City.IntsCoordinates(City.Street(nowRoad(lenow), 1), :), City.IntsCoordinates(City.Street(nowRoad(lenow), 2), :), Next_Position);
        break;
    else
        if (nextStreet == 0)
            FinaLocation(1) = -1; FinaLocation(2) = -1; %Beyond the Map
            break;
        else
            SufTime = SufTime - RemainTime;
            CurrentIns = City.Street(curStreet, 2);
            d1 = Get_InsStreet(City.IntsCoordinates(CurrentIns, :), City.IntsCoordinates(City.Street(curStreet, 1), :), City.IntsCoordinates(City.Street(curStreet, 2), :));
            d2 = Get_InsStreet(City.IntsCoordinates(CurrentIns, :), City.IntsCoordinates(City.Street(nextStreet, 1), :), City.IntsCoordinates(City.Street(nextStreet, 2), :));
            if (SufTime < CrossRoad(CurrentIns, d1, d2))
                %Final Location is this Instersection
                FinaLocation = City.IntsCoordinates(CurrentIns, :);
                break;
            else
                SufTime = SufTime - CrossRoad(CurrentIns, d1, d2);
                lenow = lenow + 1;
                nowRoad = [nowRoad, nextStreet];
                Trace = [Trace, nextStreet];
                Prob = [Prob, MaxPro / sumPro * LastProb];
                LastProb = MaxPro / sumPro * LastProb;
                %Prob
            end
        end
    end
end

