function [PD_likehood, CrossRoad, ParaRoad, StreetWidth, Roads] = Transform(City, sumoTrace)

%%%%%%%%%%%%%%%The First Step%%%%%%%%%%%%%%%%%%%%
%Predict the next street of the vehicle%

VehNum = length(sumoTrace.id); %The number of vehicles
StrNum = City.NumOfStreets; %The number of streets
InsCnt = City.NumOfIntersections; %The number of intersection 
LaneWidth = 3.3;
width = max(City.StreetLane) * LaneWidth;

MaxExistTime = 0;
for i = 1 : VehNum
    start_time = sumoTrace.arrivalTSL(i);
    end_time = sumoTrace.departureTSL(i);
    if (MaxExistTime < end_time - start_time + 1)
        MaxExistTime = end_time - start_time + 1;
    end
end

%PreProcessing
StreetWidth = ones(StrNum, 4) * width; 

%The transformation of vehicle's trace;
LocaToStrt = zeros(VehNum, MaxExistTime + 5); 
Roads = zeros(VehNum, StrNum + 5);
lenRoad = zeros(1, VehNum);
cnt = 0;

for i = 1 : VehNum        
    start_time = sumoTrace.arrivalTSL(i);
    end_time = sumoTrace.departureTSL(i);
    lenRoad(i) = 0;
    if (i == 67)
        -1;
    end
    cur_street = Get_Street(City.IntsCoordinates, City.Street, sumoTrace.x(start_time, i), sumoTrace.y(start_time, i), StrNum, StreetWidth);
    LocaToStrt(i, 1) = cur_street;
    if (cur_street > 0)
        lenRoad(i) = 1;
        if (City.IntsCoordinates(City.Street(cur_street, 1), 1) == City.IntsCoordinates(City.Street(cur_street, 2), 1))
            index = 1;
        else
            index = 2;
        end
        cur_street = cur_street - Get_Direction(index, sumoTrace.x(start_time, i), sumoTrace.x(start_time + 1, i), sumoTrace.y(start_time, i), sumoTrace.y(start_time + 1, i));
        Roads(i, lenRoad(i)) = cur_street;
    end
    for j = start_time + 1 : end_time - 1
        cur_street = Get_Street(City.IntsCoordinates, City.Street, sumoTrace.x(j, i), sumoTrace.y(j, i), StrNum, StreetWidth); %%Find the corresponding street to the given point
        LocaToStrt(i, j - start_time + 1) = cur_street;
        if (cur_street == 0) 
            continue;
        end
        if (lenRoad(i) == 0 || abs(Roads(i, lenRoad(i)) - cur_street) > 1)
            lenRoad(i) = lenRoad(i) + 1;
            if (City.IntsCoordinates(City.Street(cur_street, 1), 1) == City.IntsCoordinates(City.Street(cur_street, 2), 1))
                index = 1;
            else
                index = 2;
            end
            cur_street = cur_street - Get_Direction(index, sumoTrace.x(j, i), sumoTrace.x(j + 1, i), sumoTrace.y(j, i), sumoTrace.y(j + 1, i));
            Roads(i, lenRoad(i)) = cur_street;
        end
    end
    cur_street = Get_Street(City.IntsCoordinates, City.Street, sumoTrace.x(end_time, i), sumoTrace.y(end_time, i), StrNum, StreetWidth);
    LocaToStrt(i, end_time - start_time + 1) = cur_street;
 %   Roads(i, lenRoad(i) + 1) = Roads(i, lenRoad(i));  %Consider the disappearance of vehicle.
 %   lenRoad(i) = lenRoad(i) + 1;
end

%The prediction of vehicle's direction
maxl = 6;                          %Maximum prediction length
PD_prior = zeros(StrNum, 4 ^ maxl);    %Use quaternary to record the past road. "1" means "Left", "2" means "Forward" and "3" means "Right".
                                  %For example, "12" means the vehicle first
                                  %turns Left, then Forwards.
                               
PD1 = zeros(1, StrNum); sum = zeros(1, maxl + 2);
for i = 1 : VehNum
    if (lenRoad(i) == 0)
        continue;
    end
    sta = zeros(1, maxl);            %Record the past route of vehicles
    for j = 1 : lenRoad(i)
        cur_street = Roads(i, j);
        PD1(cur_street) = PD1(cur_street) + 1; sum(1) = sum(1) + 1;
        if (j > 1)
            new_direction = Find_Direction(City.IntsCoordinates, City.Street, Roads(i, j - 1), Roads(i, j));    %Direction: Street1 -> Street2
            for k = 1 : maxl
                sta(k) = mod(sta(k), 4 ^ (k - 1));
                sta(k) = sta(k) * 4 + new_direction;
                if (j - 1 >= k)
                    PD_prior(cur_street, sta(k)) = PD_prior(cur_street, sta(k)) + 1;
                    sum(k + 1) = sum(k + 1) + 1;
                end
            end
        end
    end
end

%Training - ML
PD_likehood = zeros(StrNum, 4 ^ maxl);
for i = 1 : VehNum
    if (lenRoad(i) == 0)
        continue;
    end
    sta = zeros(1, maxl);
    for j = 1 : lenRoad(i)
        cur_street = Roads(i, j);
        if (j > 1)
            new_direction = Find_Direction(City.IntsCoordinates, City.Street, Roads(i, j - 1), Roads(i, j));
            PD_likehood(cur_street, new_direction) = (PD_prior(cur_street, new_direction) / sum(2)) / (PD1(Roads(i, j - 1)) / sum(1)); %* PD1(cur_street) / sum(1);
            for k = 2 : maxl
                sta(k) = mod(sta(k), 4 ^ (k - 1));
                pre = sta(k);
                sta(k) = sta(k) * 4 + new_direction;
                if (k < j)
                    PD_likehood(cur_street, sta(k)) = (PD_prior(cur_street, sta(k)) / sum(k + 1)) / (PD_prior(Roads(i, j - 1), pre) / sum(k)); %* PD1(cur_street) / sum(1);
                end
            end
        end
    end
end

%%%%%%%%%%%%%%%The Second Step%%%%%%%%%%%%%%%%%%%%
%Learn the property of each street and intersection%
CrossRoad = zeros(InsCnt, 4, 4);         %Calculate the time for vehicles to pass an intersection, W-1, N-2, E-3, S-4
CntCrossRoad = zeros(InsCnt, 4, 4);

ParaRoad = zeros(StrNum, 4);                  %Get the property of Streets : Begin--bg, Middle--mid, End--en, Speed 
CntPara = zeros(StrNum, 4);

InsCost = 0;

for i = 1 : VehNum
    start_time = sumoTrace.arrivalTSL(i);
    end_time = sumoTrace.departureTSL(i);
    LeftFlag = start_time; 
    while(start_time <= end_time && LocaToStrt(i, start_time - LeftFlag + 1) == 0)
        start_time = start_time + 1;
    end
    cs = 1;
    Velocity = zeros(1, MaxExistTime); Position = zeros(1, MaxExistTime); LengthVP = 0;
    Flag = 0;
    InsCost = 0;
    for j = start_time : end_time
         if (j > start_time && LocaToStrt(i, j - LeftFlag + 1) ~= 0 && LocaToStrt(i, j - LeftFlag) ~= LocaToStrt(i, j - LeftFlag + 1))
            Flag = 1;
        else
            Flag = 0;
        end
        if (LocaToStrt(i, j - LeftFlag + 1) == 0)
            %Flag = 1;
            InsCost = InsCost + 2;
        else
            if (Flag == 1)
                if (LengthVP > 0)
                    if (LengthVP < 4)
                        Flag = 0;
                        Velocity(1) = sumoTrace.speed(j, i); Position(1) = sumoTrace.position(j, i); LengthVP = 1;
                        cs = cs + 1;
                        InsCost = 0;
                        continue;
                    end
                    [Bg, Mid, En, Speed, In] = Calc_Property(Velocity, Position, LengthVP, width);
                    InsCost = InsCost + In;
                    if (cs < lenRoad(i))
                        CurrentIns = City.Street(Roads(i, cs), 2);
                        d1 = Get_InsStreet(City.IntsCoordinates(CurrentIns, :), City.IntsCoordinates(City.Street(Roads(i, cs), 1), :), City.IntsCoordinates(City.Street(Roads(i, cs), 2), :));
                        d2 = Get_InsStreet(City.IntsCoordinates(CurrentIns, :), City.IntsCoordinates(City.Street(Roads(i, cs + 1), 1), :), City.IntsCoordinates(City.Street(Roads(i, cs + 1), 2), :));
                        CrossRoad(CurrentIns, d1, d2) = CrossRoad(CurrentIns, d1, d2) + InsCost;
                        CntCrossRoad(CurrentIns, d1, d2) = CntCrossRoad(CurrentIns, d1, d2) + 1;
                    end
                    if (Bg > 0) 
                        CntPara(Roads(i, cs), 1) = CntPara(Roads(i, cs), 1) + 1;
                        ParaRoad(Roads(i, cs), 1) = ParaRoad(Roads(i, cs), 1) + Bg;   
                    end
                    if (Mid > 0) 
                        CntPara(Roads(i, cs), 2) = CntPara(Roads(i, cs), 2) + 1;
                        ParaRoad(Roads(i, cs), 2) = ParaRoad(Roads(i, cs), 2) + Mid;
                    end
                    if (En > 0) 
                        CntPara(Roads(i, cs), 3) = CntPara(Roads(i, cs), 3) + 1;
                        ParaRoad(Roads(i, cs), 3) = ParaRoad(Roads(i, cs), 3) + En;
                    end
                    if (Speed > 0)
                        CntPara(Roads(i, cs), 4) = CntPara(Roads(i, cs), 4) + 1;
                        ParaRoad(Roads(i, cs), 4) = ParaRoad(Roads(i, cs), 4) + Speed;
                    end
                end
                Flag = 0;
                Velocity(1) = sumoTrace.speed(j, i); Position(1) = sumoTrace.position(j, i); LengthVP = 1;
                cs = cs + 1;
                InsCost = 0;
            else
                LengthVP = LengthVP + 1;
                Velocity(LengthVP) = sumoTrace.speed(j, i); Position(LengthVP) = sumoTrace.position(j, i);
            end              
        end
    end
    if (cs == lenRoad(i))
        [Bg, Mid, En, Speed, In] = Calc_Property(Velocity, Position, LengthVP, width);
        if (Bg > 0)
            CntPara(Roads(i, cs), 1) = CntPara(Roads(i, cs), 1) + 1;
            ParaRoad(Roads(i, cs), 1) = ParaRoad(Roads(i, cs), 1) + Bg; 
        end
      %  if (Mid > 0) 
      %      CntPara(Roads(i, cs), 2) = CntPara(Roads(i, cs), 2) + 1;
      %      ParaRoad(Roads(i, cs), 2) = ParaRoad(Roads(i, cs), 2) + Mid;
      %  end
      %  if (En > 0) 
      %      CntPara(Roads(i, cs), 3) = CntPara(Roads(i, cs), 3) + 1;
      %      ParaRoad(Roads(i, cs), 3) = ParaRoad(Roads(i, cs), 3) + En;
      %  end
        if (Speed > 0)
            CntPara(Roads(i, cs), 4) = CntPara(Roads(i, cs), 4) + 1;
            ParaRoad(Roads(i, cs), 4) = ParaRoad(Roads(i, cs), 4) + Speed;
        end 
    end
end

for i = 1 : StrNum
    if (CntPara(i, 1) > 0)
        ParaRoad(i, 1) = ParaRoad(i, 1) / CntPara(i, 1);
    end
    if (CntPara(i, 2) > 0)
        ParaRoad(i, 2) = ParaRoad(i, 2) / CntPara(i, 2);
    end
    if (CntPara(i, 3) > 0)
        ParaRoad(i, 3) = ParaRoad(i, 3) / CntPara(i, 3);
    end
    if (CntPara(i, 4) > 0)
        ParaRoad(i, 4) = ParaRoad(i, 4) / CntPara(i, 4);
    end
end

for i = 1 : StrNum
    if (mod(i, 2) == 1)
        opp = i + 1;
    else
        opp = i - 1;
    end
    if (abs(ParaRoad(i, 1)) < 1e-5)
        ParaRoad(i, 1) = ParaRoad(opp, 1);
    end
    if (abs(ParaRoad(i, 2)) < 1e-5)
        ParaRoad(i, 2) = ParaRoad(opp, 2);
    end
    if (abs(ParaRoad(i, 3)) < 1e-5)
        ParaRoad(i, 3) = ParaRoad(opp, 3);
    end
    if (abs(ParaRoad(i, 4)) < 1e-5)
        ParaRoad(i, 4) = ParaRoad(opp, 4);
    end
end

for i = 1 : InsCnt
    for d1 = 1 : 4
        for d2 = 1 : 4
            if (CntCrossRoad(i, d1, d2) > 0)
                CrossRoad(i, d1, d2) = CrossRoad(i, d1, d2) / CntCrossRoad(i, d1, d2);
            end
        end
    end
end

for i = 1 : InsCnt
    for d1 = 1 : 4
        for d2 = 1 : 4
            if (mod(d1, 2) == 1)
                opp1 = 4 - d1;
            else
                opp1 = 6 - d1;
            end
            if (mod(d2, 2) == 1)
                opp2 = 4 - d2;
            else
                opp2 = 6 - d2;
            end
            if (abs(CrossRoad(i, d1, d2)) < 1e-5)
                CrossRoad(i, d1, d2) = CrossRoad(i, opp1, opp2);
            end
        end
    end
end




