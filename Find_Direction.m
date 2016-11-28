function Ans = Find_Direction(IntsCoordinates, Street, bs, es)  %Use cross to find the direction of "s1 -> s2"
vecA = IntsCoordinates(Street(bs, 2), :) - IntsCoordinates(Street(bs, 1), :);
vecB = IntsCoordinates(Street(es, 2), :) - IntsCoordinates(Street(es, 1), :);
cro = vecA(1) * vecB(2) - vecA(2) * vecB(1);
if (cro > 0) 
    Ans = 1;
else if (cro < 0)
        Ans = 3;
    else
        Ans = 2;
    end
end


