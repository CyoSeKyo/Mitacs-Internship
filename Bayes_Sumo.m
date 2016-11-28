%My algorithm of Predicting the Position at a given time to a specific vehicle.

%***First step: According to past data, predict the next road the vehicle will
%   go, including disappearing in this street.
%***Second step: For each street(directed), learn its time cost.
%***Third step: Combine the first two step and get the result.

%%%%%%%%%%%%%%%The First Step & The Second Step%%%%%%%%%%%%%%%%%%%%

[LikeHood, CrossRoad, ParaRoad, StreetWidth, Roads] = Transform(City, sumoTrace);

%%%%LikeHood(Street, PastTracs)
%  -- the Probability we use to predict the next street
%Use quaternary to record the past road. "1" means "Left", "2" means "Forward" and "3" means "Right".
%For example, "12" means the vehicle first turns Left, then Forwards

%%%%CrossRoad(Intersection, Direction1, Direction2)
%  -- the Time Cost of Each directions in Given street
%  West-1, North-2, East-3, South-4

%%%%ParaRoad(Street, Property)
%  -- the Property of each Street
%  1 -- Begin (The first part Time Cost)
%  2 -- Mid (The second part Time Cost)
%  3 -- End (The third part Time Cost)
%  4 -- Speed (The average Speed)

%%%%%%%%%%%%%%%The Third Step%%%%%%%%%%%%%%%%%%%%
Avg = 0; ttt = 0;
for id = 1 : 1000;
    StrNum = City.NumOfStreets;
    VehicleID = id;
    RemainTime = 100;
    CurrentLocation = zeros(1, 2);
    for j = 1 : 4
        PastRoad = 0; Last = 0;
        start_time = sumoTrace.arrivalTSL(VehicleID);
        end_time = sumoTrace.departureTSL(VehicleID);
        test_time = floor(rand(1) * (end_time - start_time + 1)) + start_time;
        %test_time = 124;
        cs = 1; Last = 0;
        for i = start_time : test_time
            cur = Get_Street(City.IntsCoordinates, City.Street, sumoTrace.x(i, VehicleID), sumoTrace.y(i, VehicleID), StrNum, StreetWidth);
            if (cur == 0)
                continue;
            end
            if (abs(cur - Roads(VehicleID, cs)) < 2)
                if (PastRoad == 0)
                    PastRoad = City.Street(Roads(VehicleID, cs), 1);
                    cs = cs + 1;
                else
                    if (Last ~= cur)
                        PastRoad = [PastRoad, City.Street(Roads(VehicleID, cs), 1)];
                        Last = Roads(VehicleID, cs);
                        cs = cs + 1;
                    end
                end
            end
            Last = cur;
        end
        CurrentLocation(1) = sumoTrace.x(test_time, VehicleID);
        CurrentLocation(2) = sumoTrace.y(test_time, VehicleID);
        %PastRoad = [2, 10, 19, 20, 21, 22, 23];
        Accurate = zeros(1, 2);
        Accurate(1) = sumoTrace.x(test_time + RemainTime / 2, VehicleID);
        Accurate(2) = sumoTrace.y(test_time + RemainTime / 2, VehicleID);
        
        if (PastRoad(1) == 0)
            continue;
        end
        [Trace, FinaLocation, Prob] = SumoPrediction(City, LikeHood, CrossRoad, ParaRoad, StreetWidth, VehicleID, CurrentLocation, PastRoad, RemainTime);
        
        Avg = Avg + max(FinaLocation - Accurate);
        ttt = ttt + 1;
    end
end

Avg = Avg / ttt;
%%%%Trace -- Vector: Future Trace with Highest Probability

%%%%FinaLoaction -- Vector: FinaLocation(1) - X_coordinate, FinaLocation(2) - Y_coordinate

%%%%Prob -- Vector: Probability of Trace

