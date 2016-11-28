function Ans = Find_Street(IntsCoordinates, Street, cur, l, m)
Ans = 0;
if (l == 2) 
    l = 0;
else if (l == 3) 
        l = -1;
    end
end
for i = 1 : m
    if (Street(i, 1) == Street(cur, 2) && Street(i, 2) ~= Street(cur, 1))
        vecA = IntsCoordinates(Street(cur, 2), :) - IntsCoordinates(Street(cur, 1), :);
        vecB = IntsCoordinates(Street(i, 2), :) - IntsCoordinates(Street(i, 1), :);
        cro = vecA(1) * vecB(2) - vecA(2) * vecB(1);
        if (cro > 0 && l > 0) 
            Ans = i;
            break;
        end
        if (cro < 0 && l < 0) 
            Ans = i;
            break;
        end
        if (abs(cro) < 1e-5 && abs(l) < 1e-5) 
            Ans = i;
            break;
        end
    end
end
